//! A high-level USB host: enumeration, control transfers and a CDC-ACM byte stream, run against the device side of
//! a [`UsbOtg`] while the host drives the machine itself.

use std::fmt;

use super::{Token, UsbOtg};
use crate::device::{DeviceCx, DeviceId};
use crate::machine::Machine;
use crate::stop::{Limit, Stop, StopReason};

/// How long an operation waits for the device, in microseconds, before [`HostError::Timeout`]. guess: half a
/// second, far longer than any driver here needs, short enough that a test of a silent device ends quickly.
pub const TIMEOUT_US: u64 = 500_000;
/// How often a NAKed or unanswered token is tried again, in microseconds. guess: 10 µs, a few high-speed
/// microframes' worth of retries, fine enough that the host never makes a driver wait long.
pub const POLL_US: u64 = 10;
/// How long the host lets the bus settle after the pull-up appears and after the bus reset, in microseconds.
/// guess: 100 µs each, shorter than a real host's debounce (USB 2.0 7.1.7.3) so runs stay short.
pub const SETTLE_US: u64 = 100;
/// The longest the host holds a bus reset waiting for the driver's clean-up (EPFLUSH), in microseconds. guess: the
/// reset ends once the driver has acknowledged it, or after 3 ms, shorter than USB 2.0's 10 ms minimum
/// (7.1.7.5): a driver that acknowledges sees no difference.
pub const RESET_US: u64 = 3_000;
/// EP0's max packet size until the device descriptor gives it. guess: 64, the high-speed size, as a host reads
/// the first 64 bytes of the device descriptor before it knows.
const MPS0: u16 = 64;
/// A full-speed bulk endpoint's largest packet (USB 2.0 5.8.3).
const FULL_SPEED_BULK: u16 = 64;

/// Why a [`UsbHost`] operation failed.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum HostError {
    /// The device did not answer (NAK or no answer) within the timeout; what the host waited for.
    Timeout(&'static str),
    /// The device stalled the request or endpoint.
    Stall,
    /// The device answered something the protocol does not allow (a buffer error, a status stage with data, a
    /// descriptor that does not parse, more data than asked for).
    Protocol(String),
    /// The run stopped for a reason other than the host's own slice: a device halted, the core faulted, or the
    /// caller's time limit ([`Machine::set_time_limit`]) was reached.
    Stopped(StopReason),
}

impl fmt::Display for HostError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            HostError::Timeout(what) => write!(f, "usb host: timed out waiting for {what}"),
            HostError::Stall => write!(f, "usb host: the device stalled"),
            HostError::Protocol(why) => write!(f, "usb host: {why}"),
            HostError::Stopped(r) => {
                write!(f, "usb host: the run stopped: {:?} {}", r.kind, r.detail)
            }
        }
    }
}

impl std::error::Error for HostError {}

/// The endpoints of configuration descriptor `config` (with its interface and endpoint descriptors, USB 2.0
/// 9.6.3-9.6.6), in order: (bEndpointAddress, bmAttributes, wMaxPacketSize's packet size, bits 10:0). Stops at a
/// descriptor whose length is 0 or runs past the end.
pub fn endpoints(config: &[u8]) -> Vec<(u8, u8, u16)> {
    let mut out = Vec::new();
    let mut at = 0;
    while at + 2 <= config.len() {
        let len = usize::from(config[at]);
        if len == 0 || at + len > config.len() {
            break;
        }
        if config[at + 1] == 5 && len >= 7 {
            let mps = u16::from_le_bytes([config[at + 4], config[at + 5]]) & 0x7FF;
            out.push((config[at + 2], config[at + 3], mps));
        }
        at += len;
    }
    out
}

/// A USB host on the other end of a [`UsbOtg`]'s cable that drives the machine itself: each operation makes its
/// transactions with the controller's host-side primitives ([`UsbOtg::setup`], [`UsbOtg::in_token`],
/// [`UsbOtg::out_packet`], ...) through [`Machine::with_device`], and between them runs the machine in slices of
/// [`POLL_US`] microseconds until the device answers or [`TIMEOUT_US`] passes (both settable with
/// [`UsbHost::set_timeouts`]). No generators, no script: the caller calls the operations in order.
///
/// Transactions, as the host side of each:
/// - A token answered NAK or not at all ([`Token::Nak`], [`Token::NoAnswer`]) is tried again after a slice, until
///   the timeout ([`HostError::Timeout`] with what was waited for); the timeout starts again with each answer.
/// - [`Token::Stall`] is [`HostError::Stall`]; [`Token::Error`] (an OUT that does not fit the device's buffer) is
///   [`HostError::Protocol`].
/// - Control transfers use EP0's max packet size from the device descriptor (64 until it is read); bulk transfers
///   use the first bulk endpoint of each direction in the configuration, its max packet size clamped to 64 when the
///   port is at full speed ([`UsbOtg::high_speed`]), as a full-speed host does.
/// - Bulk IN data is collected as a read URB of [`UsbHost::set_urb`] bytes would collect it (as Linux's cdc_acm
///   does): held until a short packet ends the transfer or the URB is full. With no URB (the default) every packet
///   reaches [`UsbHost::read`] at once.
///
/// Slices: each runs the machine until the clock is [`POLL_US`] on (a [`Stop::TimeLimit`] the host sets and
/// removes again). A core waiting after STOP with nothing able to interrupt it ([`Stop::HaltedForever`]) is
/// waiting for the host: the slice moves the clock on with [`Machine::idle_until`] instead. Any other stop, and the
/// caller's own time limit, ends the operation with [`HostError::Stopped`]. A halt the controller asks for during a
/// transaction is reported by the next slice's run.
#[derive(Debug, Clone)]
pub struct UsbHost {
    usb: DeviceId,
    ticks_per_us: u64,
    timeout_us: u64,
    poll_us: u64,
    address: u8,
    mps0: u16,
    endpoints: Vec<(u8, u8, u16)>,
    urb: Option<usize>,
    /// Bulk IN bytes delivered to the reader.
    rx: Vec<u8>,
    /// Bulk IN bytes held in the URB.
    held: Vec<u8>,
}

impl UsbHost {
    /// A host for the controller `usb` (a [`UsbOtg`]) on a machine with `ticks_per_us` ticks per microsecond, with
    /// VBUS off, at address 0, with the default timeouts and no URB. Panics when `ticks_per_us` is 0.
    pub fn new(usb: DeviceId, ticks_per_us: u64) -> UsbHost {
        assert!(
            ticks_per_us > 0,
            "UsbHost needs at least one tick per microsecond"
        );
        UsbHost {
            usb,
            ticks_per_us,
            timeout_us: TIMEOUT_US,
            poll_us: POLL_US,
            address: 0,
            mps0: MPS0,
            endpoints: Vec::new(),
            urb: None,
            rx: Vec::new(),
            held: Vec::new(),
        }
    }

    /// Set how long an operation waits for the device and how often it tries again, in microseconds. Panics when
    /// `poll_us` is 0.
    pub fn set_timeouts(&mut self, timeout_us: u64, poll_us: u64) {
        assert!(
            poll_us > 0,
            "UsbHost needs a poll interval of at least 1 µs"
        );
        self.timeout_us = timeout_us;
        self.poll_us = poll_us;
    }

    /// Hold bulk IN data until a short packet ends the transfer or `bytes` have arrived, as a read URB of that
    /// size does; `None` delivers every packet at once (and releases anything held).
    pub fn set_urb(&mut self, bytes: Option<usize>) {
        self.urb = bytes;
        if bytes.is_none() {
            self.rx.append(&mut self.held);
        }
    }

    /// The address the host talks to (0 until [`UsbHost::enumerate`] sets one).
    pub fn address(&self) -> u8 {
        self.address
    }

    /// The endpoints of the configuration [`UsbHost::enumerate`] read (see [`endpoints`]).
    pub fn endpoint_list(&self) -> &[(u8, u8, u16)] {
        &self.endpoints
    }

    // bus -------------------------------------------------------------------------------------------------------

    /// Plug in: VBUS on, wait for the driver's pull-up (USBCMD\[RS\]), [`SETTLE_US`], then a bus reset held until
    /// the driver has flushed its endpoints ([`UsbOtg::reset_acked`]) or [`RESET_US`] passes, then [`SETTLE_US`].
    /// The host is at address 0 afterwards with nothing received.
    pub fn connect(&mut self, m: &mut Machine) -> Result<(), HostError> {
        self.on(m, |u, cx| u.vbus(true, cx));
        self.until(m, "device pull-up (USBCMD[RS])", UsbOtg::running)?;
        self.wait(m, SETTLE_US)?;
        self.on(m, |u, cx| u.bus_reset_start(cx));
        let end = m.now() + RESET_US * self.ticks_per_us;
        while !m.device::<UsbOtg>(self.usb).reset_acked() && m.now() < end {
            self.slice(m, self.poll_us)?;
        }
        self.on(m, |u, cx| u.bus_reset_end(cx));
        self.address = 0;
        self.mps0 = MPS0;
        self.rx.clear();
        self.held.clear();
        self.wait(m, SETTLE_US)
    }

    /// Enumerate as a host does: the first 64 bytes of the device descriptor at address 0 (for EP0's max packet
    /// size), SET_ADDRESS `address` and wait for DEVICEADDR to take it, the whole device descriptor, the
    /// configuration descriptor's first 9 bytes and then all of it, SET_CONFIGURATION with its
    /// bConfigurationValue. Returns (device descriptor, configuration descriptor); the configuration's endpoints
    /// are kept for the bulk transfers.
    pub fn enumerate(
        &mut self,
        m: &mut Machine,
        address: u8,
    ) -> Result<(Vec<u8>, Vec<u8>), HostError> {
        let first = self.control_in(m, [0x80, 6, 0, 1, 0, 0, 64, 0])?;
        if first.len() < 8 || first[1] != 1 {
            return Err(HostError::Protocol(format!(
                "device descriptor {first:02x?} does not parse"
            )));
        }
        self.mps0 = u16::from(first[7]);
        self.control_out(m, [0, 5, address, 0, 0, 0, 0, 0], &[])?;
        self.until(m, "DEVICEADDR after SET_ADDRESS", move |u| {
            u.address() == address
        })?;
        self.address = address;
        let device = self.control_in(m, [0x80, 6, 0, 1, 0, 0, 18, 0])?;
        if device.len() != 18 || device[0] != 18 || device[1] != 1 {
            return Err(HostError::Protocol(format!(
                "device descriptor {device:02x?} does not parse"
            )));
        }
        let head = self.control_in(m, [0x80, 6, 0, 2, 0, 0, 9, 0])?;
        if head.len() != 9 || head[1] != 2 {
            return Err(HostError::Protocol(format!(
                "configuration descriptor {head:02x?} does not parse"
            )));
        }
        let total = u16::from_le_bytes([head[2], head[3]]);
        let [lo, hi] = total.to_le_bytes();
        let config = self.control_in(m, [0x80, 6, 0, 2, 0, 0, lo, hi])?;
        if config.len() != usize::from(total) {
            return Err(HostError::Protocol(format!(
                "configuration descriptor is {} bytes, wTotalLength {total}",
                config.len()
            )));
        }
        self.control_out(m, [0, 9, config[5], 0, 0, 0, 0, 0], &[])?;
        self.endpoints = endpoints(&config);
        Ok((device, config))
    }

    // control transfers ----------------------------------------------------------------------------------------

    /// A control transfer with an IN data stage (or none, when wLength is 0): the setup packet, IN tokens until
    /// wLength bytes or a short packet, then the zero-length OUT status stage. Returns the data.
    pub fn control_in(&mut self, m: &mut Machine, setup: [u8; 8]) -> Result<Vec<u8>, HostError> {
        let wlen = usize::from(u16::from_le_bytes([setup[6], setup[7]]));
        if wlen == 0 {
            self.control_out(m, setup, &[])?;
            return Ok(Vec::new());
        }
        let addr = self.address;
        self.transact(m, "setup", false, move |u, cx| u.setup(setup, addr, cx))?;
        let mut data = Vec::new();
        while data.len() < wlen {
            let p = self.packet(m, "control IN data", false, move |u, cx| {
                u.in_token(0, addr, cx)
            })?;
            let short = p.len() < usize::from(self.mps0);
            data.extend_from_slice(&p);
            if short {
                break;
            }
        }
        if data.len() > wlen {
            return Err(HostError::Protocol(format!(
                "control IN of {setup:02x?} returned {} bytes, more than wLength",
                data.len()
            )));
        }
        self.transact(m, "control status (OUT)", false, move |u, cx| {
            u.out_packet(0, &[], addr, cx)
        })?;
        Ok(data)
    }

    /// A control transfer with an OUT data stage of `data` (none when empty): the setup packet, `data` in EP0
    /// packets, then the zero-length IN status stage. Panics when `data`'s length is not wLength.
    pub fn control_out(
        &mut self,
        m: &mut Machine,
        setup: [u8; 8],
        data: &[u8],
    ) -> Result<(), HostError> {
        assert_eq!(
            data.len(),
            usize::from(u16::from_le_bytes([setup[6], setup[7]])),
            "control OUT data must be wLength bytes"
        );
        let addr = self.address;
        self.transact(m, "setup", false, move |u, cx| u.setup(setup, addr, cx))?;
        for chunk in data.chunks(usize::from(self.mps0.max(1))) {
            self.transact(m, "control OUT data", false, move |u, cx| {
                u.out_packet(0, chunk, addr, cx)
            })?;
        }
        let status = self.packet(m, "control status (IN)", false, move |u, cx| {
            u.in_token(0, addr, cx)
        })?;
        if !status.is_empty() {
            return Err(HostError::Protocol(format!(
                "status stage of {setup:02x?} returned {status:02x?}"
            )));
        }
        Ok(())
    }

    // CDC-ACM ----------------------------------------------------------------------------------------------------

    /// Open a CDC-ACM port on communication interface `interface` as a terminal program does: SET_LINE_CODING
    /// 115200 baud 8N1, then SET_CONTROL_LINE_STATE with DTR and RTS (the class's "connected").
    pub fn cdc_open(&mut self, m: &mut Machine, interface: u8) -> Result<(), HostError> {
        self.control_out(
            m,
            [0x21, 0x20, 0, 0, interface, 0, 7, 0],
            &[0x00, 0xC2, 0x01, 0x00, 0, 0, 8],
        )?;
        self.control_out(m, [0x21, 0x22, 3, 0, interface, 0, 0, 0], &[])
    }

    /// Send `data` on the bulk OUT endpoint in max-packet pieces (one zero-length packet when `data` is empty).
    /// While a packet is NAKed the host keeps an IN request going on the bulk IN endpoint, as a real host's
    /// outstanding read does; what arrives there is kept for [`UsbHost::read`].
    pub fn write(&mut self, m: &mut Machine, data: &[u8]) -> Result<(), HostError> {
        let (ep, mps) = self.bulk(m, false)?;
        let addr = self.address;
        let chunks: Vec<&[u8]> = if data.is_empty() {
            vec![&[]]
        } else {
            data.chunks(usize::from(mps)).collect()
        };
        for chunk in chunks {
            self.transact(m, "bulk OUT", true, move |u, cx| {
                u.out_packet(ep, chunk, addr, cx)
            })?;
        }
        Ok(())
    }

    /// The next `n` bytes from the bulk IN endpoint, waiting for them (see [`UsbHost::set_urb`]).
    pub fn read(&mut self, m: &mut Machine, n: usize) -> Result<Vec<u8>, HostError> {
        let (ep, _) = self.bulk(m, true)?;
        let addr = self.address;
        while self.rx.len() < n {
            let p = self.packet(m, "bulk IN", false, move |u, cx| u.in_token(ep, addr, cx))?;
            self.take(m, p)?;
        }
        Ok(self.rx.drain(..n).collect())
    }

    // waiting ----------------------------------------------------------------------------------------------------

    /// Run the machine until `cond` holds of the controller, checking before each slice; [`HostError::Timeout`]
    /// with `what` when it still does not after the timeout.
    pub fn until(
        &mut self,
        m: &mut Machine,
        what: &'static str,
        mut cond: impl FnMut(&UsbOtg) -> bool,
    ) -> Result<(), HostError> {
        let end = m
            .now()
            .saturating_add(self.timeout_us.saturating_mul(self.ticks_per_us));
        loop {
            if cond(m.device::<UsbOtg>(self.usb)) {
                return Ok(());
            }
            if m.now() >= end {
                return Err(HostError::Timeout(what));
            }
            self.slice(m, self.poll_us)?;
        }
    }

    /// Run the machine for `us` microseconds.
    pub fn wait(&mut self, m: &mut Machine, us: u64) -> Result<(), HostError> {
        self.slice(m, us)
    }

    // internals --------------------------------------------------------------------------------------------------

    fn on<R>(&self, m: &mut Machine, f: impl FnOnce(&mut UsbOtg, &mut DeviceCx) -> R) -> R {
        m.with_device::<UsbOtg, R>(self.usb, f)
    }

    /// Run the machine until the clock is `us` microseconds on (at least one tick).
    fn slice(&self, m: &mut Machine, us: u64) -> Result<(), HostError> {
        let end = m
            .now()
            .saturating_add(us.saturating_mul(self.ticks_per_us).max(1));
        let theirs = m.time_limit();
        let limit = theirs.map_or(end, |t| t.min(end));
        m.set_time_limit(Some(limit));
        let r = loop {
            let r = m.run(Limit::default());
            if r.kind == Stop::HaltedForever {
                // Waiting after STOP for an interrupt only the host can cause: move the clock on. The next run
                // stops at the limit before its first instruction.
                match m.idle_until(limit.max(m.now())) {
                    None => continue,
                    Some(stop) => break stop,
                }
            }
            break r;
        };
        m.set_time_limit(theirs);
        let theirs_reached = theirs.is_some_and(|t| m.now() >= t);
        if r.kind == Stop::TimeLimit && !theirs_reached {
            Ok(())
        } else {
            Err(HostError::Stopped(r))
        }
    }

    /// Make a transaction with `f` until it is answered: NAK and no answer are tried again after a slice (polling
    /// the bulk IN endpoint in between when `poll_in`), STALL and a buffer error fail.
    fn transact(
        &mut self,
        m: &mut Machine,
        what: &'static str,
        poll_in: bool,
        mut f: impl FnMut(&mut UsbOtg, &mut DeviceCx) -> Token,
    ) -> Result<Token, HostError> {
        let end = m
            .now()
            .saturating_add(self.timeout_us.saturating_mul(self.ticks_per_us));
        loop {
            match self.on(m, &mut f) {
                Token::Stall => return Err(HostError::Stall),
                Token::Error => {
                    return Err(HostError::Protocol(format!(
                        "{what}: the device reported a buffer error"
                    )));
                }
                Token::Nak | Token::NoAnswer => {}
                answer => return Ok(answer),
            }
            if m.now() >= end {
                return Err(HostError::Timeout(what));
            }
            if poll_in {
                self.poll_in(m)?;
            }
            self.slice(m, self.poll_us)?;
        }
    }

    /// [`UsbHost::transact`] for a token answered with data.
    fn packet(
        &mut self,
        m: &mut Machine,
        what: &'static str,
        poll_in: bool,
        f: impl FnMut(&mut UsbOtg, &mut DeviceCx) -> Token,
    ) -> Result<Vec<u8>, HostError> {
        match self.transact(m, what, poll_in, f)? {
            Token::Data(p) => Ok(p),
            other => Err(HostError::Protocol(format!(
                "{what}: answered {other:?}, not data"
            ))),
        }
    }

    /// One IN token on the bulk IN endpoint, keeping any data it brings.
    fn poll_in(&mut self, m: &mut Machine) -> Result<(), HostError> {
        let (ep, _) = self.bulk(m, true)?;
        let addr = self.address;
        if let Token::Data(p) = self.on(m, |u, cx| u.in_token(ep, addr, cx)) {
            self.take(m, p)?;
        }
        Ok(())
    }

    /// A bulk IN packet: to the reader, or into the URB until a short packet or a full URB releases it.
    fn take(&mut self, m: &Machine, packet: Vec<u8>) -> Result<(), HostError> {
        let Some(urb) = self.urb else {
            self.rx.extend(packet);
            return Ok(());
        };
        let (_, mps) = self.bulk(m, true)?;
        let short = packet.len() < usize::from(mps);
        self.held.extend(packet);
        if short || self.held.len() >= urb {
            self.rx.append(&mut self.held);
        }
        Ok(())
    }

    /// The first bulk endpoint of the direction (IN when `inward`): its number and max packet size, clamped to 64
    /// at full speed.
    fn bulk(&self, m: &Machine, inward: bool) -> Result<(u8, u16), HostError> {
        let &(a, _, mps) = self
            .endpoints
            .iter()
            .find(|&&(a, attr, _)| attr & 3 == 2 && (a & 0x80 != 0) == inward)
            .ok_or_else(|| {
                HostError::Protocol(format!(
                    "no bulk {} endpoint in the configuration",
                    if inward { "IN" } else { "OUT" }
                ))
            })?;
        let mps = if m.device::<UsbOtg>(self.usb).high_speed() {
            mps
        } else {
            mps.min(FULL_SPEED_BULK)
        };
        Ok((a & 0x0F, mps))
    }
}
