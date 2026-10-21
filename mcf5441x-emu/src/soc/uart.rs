//! The UARTs UART0-UART9 (RM ch. 41).

use std::fmt;

use std::any::Any;

use crate::device::{Device, DeviceCx};
use crate::mem::Size;
use crate::stop::{Stop, StopReason};

const UMR: u32 = 0x00;
const USR: u32 = 0x04;
const UCR: u32 = 0x08;
const URB: u32 = 0x0C;
const UIPCR: u32 = 0x10;
const UISR: u32 = 0x14;
const UIP: u32 = 0x34;

/// USR bits (RM Table 41-6).
const RXRDY: u8 = 0x01;
const TXRDY: u8 = 0x04;
const TXEMP: u8 = 0x08;
const OE: u8 = 0x10;
const FE: u8 = 0x40;

/// UMR1\[ERR\]: block error mode; UMR1\[RxIRQ\]: FFULL as the receive request (RM Table 41-4).
const ERR: u8 = 0x20;
const RXIRQ: u8 = 0x40;
/// UMR1\[PM\] = 11: multidrop mode.
const PM: u8 = 0x18;

/// Ticks from a UTB write to the character's move into an idle shift register: status reads at +1 and +2 see
/// TXRDY clear, at +3 set. Measured on an MCF54415 as 2 to 9 instructions, varying from run to run (TXRDY first
/// seen on the 3rd to the 10th back-to-back USR read); guess: the shortest measured.
const HANDOVER: u64 = 3;

/// How a UART is wired and clocked: board values.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UartConfig {
    /// The interrupt controller its request goes to (its [`Intc::index`](super::Intc::index)).
    pub controller: usize,
    /// Its source on that controller (RM Table 17-15, Table 17-16: UART0-UART3 are INTC0
    /// sources 26-29, UART4-UART9 INTC1 sources 48-53).
    pub source: u8,
    /// The DMA request line of its receive request (RM Table 19-6), if it is wired to the eDMA.
    pub rx_dma: Option<u8>,
    /// The ticks one character takes on the line, start bit to stop bit: the board's baud rate in ticks. 0 makes
    /// sending and receiving instant.
    pub ticks_per_byte: u64,
}

/// Whatever sits at the far end of a UART's TX and RX lines: a board's other microcontroller, a console.
///
/// A peer is [attached](Uart::attach) to its UART, which calls it as its characters go out and at the peer's own
/// events; through [`PeerCx`] it sends characters to the UART's receiver and can end the run.
pub trait SerialPeer: Any {
    /// A character left the UART's shift register: it began shifting out at `cx.now`.
    fn transmitted(&mut self, byte: u8, cx: &mut PeerCx);
    /// The peer's own next event (absolute tick), if any.
    fn next_event(&self) -> Option<u64> {
        None
    }
    /// Called at an instruction boundary once the clock has reached [`SerialPeer::next_event`] (`cx.now` may be
    /// past it). Afterwards the event should lie after `cx.now`, as for [`Device::advance`].
    fn advance(&mut self, _cx: &mut PeerCx) {}
}

/// What a [`SerialPeer`] can reach: the receiver of its UART, and the run.
pub struct PeerCx<'a> {
    /// The clock: the tick a character began shifting out, or the tick being advanced to.
    pub now: u64,
    rx_pending: &'a mut Vec<(u64, Option<u8>)>,
    ticks_per_byte: u64,
    stop: &'a mut Option<StopReason>,
}

impl PeerCx<'_> {
    /// Send a character to the UART's receiver, its start bit at `start`, as [`Uart::receive`] does: it arrives
    /// [`UartConfig::ticks_per_byte`] later. `None` is a character with a framing error. Panics when `start` is
    /// before `now`.
    pub fn send(&mut self, start: u64, byte: Option<u8>) {
        assert!(
            start >= self.now,
            "a peer sends at tick {start}, before the clock ({})",
            self.now
        );
        schedule(self.rx_pending, self.ticks_per_byte, start, byte);
    }

    /// End the run, as [`DeviceCx::halt`] does: it stops at the next instruction boundary, with PC on the next
    /// instruction. The first request in an instruction wins.
    pub fn halt(&mut self, kind: Stop, detail: impl Into<String>) {
        if self.stop.is_none() {
            *self.stop = Some(StopReason {
                kind,
                pc: 0,
                detail: detail.into(),
            });
        }
    }
}

/// Put a character with its start bit at `start` on the receive line `rx_pending`.
fn schedule(
    rx_pending: &mut Vec<(u64, Option<u8>)>,
    ticks_per_byte: u64,
    start: u64,
    b: Option<u8>,
) {
    let at = start + ticks_per_byte;
    let i = rx_pending.partition_point(|&(t, _)| t <= at);
    rx_pending.insert(i, (at, b));
}

/// One UART (RM ch. 41), with a 1-deep receiver and a transmitter with a holding and a shift register that sends
/// one character per [`UartConfig::ticks_per_byte`].
///
/// The host drives the receive line with [`Uart::receive`] and sees what is sent in [`Uart::sent`] (or through a
/// hook, [`Uart::on_byte`]). A [`SerialPeer`] [attached](Uart::attach) to it is the other end of both lines: it
/// hears each character as it begins shifting out, sends characters to the receiver, has events of its own (the
/// UART's [`Device::next_event`] includes them) and can end the run. The receiver and the transmitter start disabled (RM 41.3.5): a board whose boot
/// stages left them on writes UCR.
///
/// Registers (byte registers; the access size is not checked and only the low byte of a write is used):
/// - UMR1/UMR2 (+0x00, reset 0): stored and read back through the mode register pointer (RM 41.3.1, RM
///   41.3.2). Of their fields only UMR1\[ERR\] acts; parity, character length, stop bits and RTS/CTS control
///   are stored only. Setting UMR1\[RxIRQ\] (FFULL as the receive request: the FIFO is not modelled), UMR1\[PM\]
///   multidrop mode (the receiver then loads address characters even while disabled, RM 41.4.4) or a UMR2\[CM\]
///   channel mode other than normal (automatic echo, local loopback, remote loopback) is refused with
///   [`Stop::Unmodelled`], the detail naming the UART and the mode. guess: a UMR1 write starts block mode's
///   accumulated status afresh.
/// - USR (+0x04 read): RXRDY, TXRDY, TXEMP, OE, FE. FFULL, PE and RB are never set. UCSR (+0x04 write) is ignored:
///   the timing is [`UartConfig::ticks_per_byte`].
/// - UCR (+0x08 write): the RC and TC commands, then MISC (so 0x21 ends with the receiver disabled). MISC 1 resets
///   the mode register pointer, 2 resets the receiver (disables it and empties the holding register; bytes still
///   on the line arrive later and are lost), 3 resets the transmitter (disables it and abandons the characters in
///   it), 4 resets the error status (OE and FE); the break commands are ignored.
/// - URB (+0x0C read): the received character, which the read pops with its status. An empty URB reads the last
///   character again (guess: RM Table 41-3 gives only its reset value, 0xFF).
/// - UTB (+0x0C write): loads the transmitter holding register. The character is dropped ([`Uart::dropped`])
///   when the transmitter is off or TXRDY is clear (RM 41.3.7: the holding register accepts a character only
///   with TXRDY set).
/// - UISR (+0x14 read): TXRDY (bit 0) and RXRDY (bit 1), unmasked; COS and DB are never set. UIMR (+0x14 write):
///   bit 0 TXRDY and bit 1 RXRDY are the interrupt mask.
/// - UIPCR (+0x10) reads 0x0F and UIP (+0x34) reads 0xFF: CTS is never asserted (guess: no device drives it).
///   UACR, UBG1/UBG2 and UOP1/UOP0 take writes and do nothing. The rest of the window reads 0.
///
/// The transmitter (RM 41.3.7, RM 41.4.2.1; timing measured on an MCF54415): a UTB write fills the holding
/// register, clearing TXRDY and TXEMP. The character moves to the shift register 3 ticks after the write when
/// that is idle (status reads at +1 and +2 still see TXRDY clear; guess: the shortest of the measured 2 to 9
/// instructions), else when the shift register finishes its character; the move sets TXRDY again. A second
/// character written before the move is dropped (RM 41.3.7), as on the chip. TXEMP sets when the last character has been shifted out, `ticks_per_byte` after it
/// began. TRANSMITTER DISABLE lets the character being shifted finish; one still in the holding register is not
/// sent (guess: RM Table 41-6 says characters loaded while the transmitter is disabled are not sent). Each
/// character is recorded ([`Uart::sent`], [`Uart::on_byte`]) with the tick it begins shifting out: when it
/// starts on the line, which is what a peer on the line sees. The transmit DMA request is not driven.
///
/// The receiver: a character arrives `ticks_per_byte` after its start bit, and lands in the holding register,
/// setting RXRDY, when the receiver is on and the register is empty. Otherwise it is lost ([`Uart::lost`]): with
/// the receiver off, with no status (RM 41.3.5), or over a held character, setting OE (RM Table 41-6); the framing
/// error of a lost character is lost with it. The three-deep FIFO is not modelled. A framing error is received as
/// 0xFF with FE: in character mode FE belongs to the character held and is popped with it, in block mode
/// (UMR1\[ERR\]) it holds until RESET ERROR STATUS (RM 41.4.2.3).
///
/// Requests: the interrupt (source [`UartConfig::source`] of controller [`UartConfig::controller`]) is
/// `UIMR[0] & TXRDY | UIMR[1] & RXRDY`; the receive DMA request (line [`UartConfig::rx_dma`]) is RXRDY (RM
/// 41.5.1.2), so the eDMA's read of URB empties the holding register and lowers it.
pub struct Uart {
    config: UartConfig,
    umr: [u8; 2],
    /// The mode register pointer is on UMR2.
    umr2: bool,
    uimr: u8,
    rx_on: bool,
    /// The holding register: the character and its framing error.
    rbr: Option<(u8, bool)>,
    /// The last character read from URB.
    last: u8,
    oe: bool,
    /// Block mode's FE: any character with a framing error since RESET ERROR STATUS.
    block_fe: bool,
    /// Characters on the line: (arrival tick, character or `None` for a framing error), in arrival order.
    rx_pending: Vec<(u64, Option<u8>)>,
    tx_on: bool,
    /// The holding register: the character and the earliest tick it can move to the shift register.
    holding: Option<(u8, u64)>,
    /// The tick the shift register finishes its character (idle when not after the clock).
    shift_end: u64,
    sent: Vec<(u64, u8)>,
    hook: Option<Box<dyn FnMut(u64, u8)>>,
    peer: Option<Box<dyn SerialPeer>>,
    lost: u64,
    dropped: u64,
}

impl fmt::Debug for Uart {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Uart")
            .field("config", &self.config)
            .field("rx_on", &self.rx_on)
            .field("tx_on", &self.tx_on)
            .field("rbr", &self.rbr)
            .field("sent", &self.sent.len())
            .field("lost", &self.lost)
            .field("dropped", &self.dropped)
            .finish_non_exhaustive()
    }
}

impl Uart {
    /// A UART at reset, wired and clocked as `config` says.
    pub fn new(config: UartConfig) -> Uart {
        Uart {
            config,
            umr: [0; 2],
            umr2: false,
            uimr: 0,
            rx_on: false,
            rbr: None,
            last: 0xFF,
            oe: false,
            block_fe: false,
            rx_pending: Vec::new(),
            tx_on: false,
            holding: None,
            shift_end: 0,
            sent: Vec::new(),
            hook: None,
            peer: None,
            lost: 0,
            dropped: 0,
        }
    }

    /// Characters on the receive line: (tick of the start bit, character), `None` for a character with a framing
    /// error. Each arrives [`UartConfig::ticks_per_byte`] after its start; characters arriving at the same tick
    /// arrive in the order given.
    pub fn receive(&mut self, bytes: &[(u64, Option<u8>)]) {
        for &(start, b) in bytes {
            schedule(&mut self.rx_pending, self.config.ticks_per_byte, start, b);
        }
    }

    /// Attach `peer` to the far end of the lines, replacing any earlier peer.
    pub fn attach(&mut self, peer: Box<dyn SerialPeer>) {
        self.peer = Some(peer);
    }

    /// The attached peer. Panics when there is none or it is not a `T`.
    pub fn peer<T: SerialPeer>(&self) -> &T {
        let peer: &dyn Any = self.peer.as_deref().expect("no peer attached");
        peer.downcast_ref()
            .unwrap_or_else(|| panic!("the peer is not a {}", std::any::type_name::<T>()))
    }

    /// The attached peer, mutably. Panics when there is none or it is not a `T`.
    pub fn peer_mut<T: SerialPeer>(&mut self) -> &mut T {
        let peer: &mut dyn Any = self.peer.as_deref_mut().expect("no peer attached");
        peer.downcast_mut()
            .unwrap_or_else(|| panic!("the peer is not a {}", std::any::type_name::<T>()))
    }

    /// The peer's context at `now`.
    fn peer_cx<'a>(
        rx_pending: &'a mut Vec<(u64, Option<u8>)>,
        ticks_per_byte: u64,
        now: u64,
        cx: &'a mut DeviceCx,
    ) -> PeerCx<'a> {
        PeerCx {
            now,
            rx_pending,
            ticks_per_byte,
            stop: &mut *cx.stop,
        }
    }

    /// The characters sent, each with the tick it began shifting out onto the line.
    pub fn sent(&self) -> &[(u64, u8)] {
        &self.sent
    }

    /// The characters sent, as UTF-8 (invalid sequences replaced).
    pub fn text(&self) -> String {
        let bytes: Vec<u8> = self.sent.iter().map(|&(_, b)| b).collect();
        String::from_utf8_lossy(&bytes).into_owned()
    }

    /// Call `f` with the tick and the character of every character sent from now on: a board's peer on the line.
    pub fn on_byte(&mut self, f: Box<dyn FnMut(u64, u8)>) {
        self.hook = Some(f);
    }

    /// Received characters lost: arriving with the receiver off or over a held character.
    pub fn lost(&self) -> u64 {
        self.lost
    }

    /// Characters written to UTB and never sent: written with the transmitter off, written with the holding
    /// register full (TXRDY clear), or left in the holding register by TRANSMITTER DISABLE.
    pub fn dropped(&self) -> u64 {
        self.dropped
    }

    fn usr(&self, now: u64) -> u8 {
        let mut v = 0;
        if let Some((_, fe)) = self.rbr {
            v |= RXRDY;
            if fe && self.umr[0] & ERR == 0 {
                v |= FE;
            }
        }
        if self.block_fe && self.umr[0] & ERR != 0 {
            v |= FE;
        }
        if self.oe {
            v |= OE;
        }
        if self.tx_on && self.holding.is_none() {
            v |= TXRDY;
            if now >= self.shift_end {
                v |= TXEMP;
            }
        }
        v
    }

    fn uisr(&self, now: u64) -> u8 {
        let usr = self.usr(now);
        u8::from(usr & TXRDY != 0) | u8::from(usr & RXRDY != 0) << 1
    }

    fn drive(&self, cx: &mut DeviceCx) {
        let on = self.uisr(cx.now) & self.uimr & 3 != 0;
        cx.irq.set(self.config.controller, self.config.source, on);
        if let Some(ch) = self.config.rx_dma {
            cx.dma.set(ch, self.rbr.is_some());
        }
    }

    fn arrive(&mut self, b: Option<u8>) {
        if !self.rx_on {
            self.lost += 1;
        } else if self.rbr.is_some() {
            self.oe = true;
            self.lost += 1;
        } else {
            let fe = b.is_none();
            self.rbr = Some((b.unwrap_or(0xFF), fe));
            self.block_fe |= fe;
        }
    }

    fn command(&mut self, v: u8) {
        match v & 3 {
            1 => self.rx_on = true,
            2 => self.rx_on = false,
            _ => {}
        }
        match v >> 2 & 3 {
            1 => self.tx_on = true,
            2 => {
                self.tx_on = false;
                if self.holding.take().is_some() {
                    self.dropped += 1;
                }
            }
            _ => {}
        }
        match v >> 4 & 7 {
            1 => self.umr2 = false,
            2 => {
                self.rx_on = false;
                self.rbr = None;
            }
            3 => {
                self.tx_on = false;
                self.holding = None;
                self.shift_end = 0;
            }
            4 => {
                self.oe = false;
                self.block_fe = false;
                if let Some((_, fe)) = &mut self.rbr {
                    *fe = false;
                }
            }
            _ => {}
        }
    }

    fn send(&mut self, b: u8, now: u64) {
        if !self.tx_on || self.holding.is_some() {
            self.dropped += 1;
            return;
        }
        self.holding = Some((b, now + HANDOVER));
    }

    /// The tick the character in the holding register moves to the shift register.
    fn move_at(&self) -> Option<u64> {
        self.holding.map(|(_, at)| at.max(self.shift_end))
    }

    /// Bring the transmitter to `cx.now`: move a held character into the shift register when it is due, and tell
    /// the peer.
    fn catch_up(&mut self, cx: &mut DeviceCx) {
        if let (Some(at), Some((b, _))) = (self.move_at(), self.holding)
            && at <= cx.now
        {
            self.holding = None;
            self.shift_end = at + self.config.ticks_per_byte;
            self.sent.push((at, b));
            if let Some(f) = &mut self.hook {
                f(at, b);
            }
            if let Some(p) = &mut self.peer {
                let tpb = self.config.ticks_per_byte;
                p.transmitted(b, &mut Self::peer_cx(&mut self.rx_pending, tpb, at, cx));
            }
        }
    }

    /// Refuse a mode register write selecting a mode this model leaves out.
    fn check_mode(&self, v: u8, cx: &mut DeviceCx) -> bool {
        let what = if !self.umr2 {
            if v & RXIRQ != 0 {
                Some("UMR1 RxIRQ (FFULL as the receive request)")
            } else if v & PM == PM {
                Some("UMR1 PM multidrop mode")
            } else {
                None
            }
        } else {
            match v >> 6 {
                1 => Some("UMR2 CM automatic echo mode"),
                2 => Some("UMR2 CM local loopback mode"),
                3 => Some("UMR2 CM remote loopback mode"),
                _ => None,
            }
        };
        if let Some(what) = what {
            cx.refuse(Stop::Unmodelled, format!("UART: {what} is not modelled"));
        }
        what.is_none()
    }
}

impl Device for Uart {
    fn read(&mut self, offset: u32, _size: Size, cx: &mut DeviceCx) -> u32 {
        self.catch_up(cx);
        let v = match offset {
            UMR => {
                let i = usize::from(self.umr2);
                self.umr2 = true;
                self.umr[i]
            }
            USR => self.usr(cx.now),
            URB => {
                if let Some((b, _)) = self.rbr.take() {
                    self.last = b;
                }
                self.drive(cx);
                self.last
            }
            UIPCR => 0x0F,
            UISR => self.uisr(cx.now),
            UIP => 0xFF,
            _ => 0,
        };
        u32::from(v)
    }

    fn write(&mut self, offset: u32, _size: Size, value: u32, cx: &mut DeviceCx) {
        let v = value as u8;
        self.catch_up(cx);
        match offset {
            UMR if !self.check_mode(v, cx) => {}
            UMR => {
                let i = usize::from(self.umr2);
                self.umr2 = true;
                self.umr[i] = v;
                if i == 0 {
                    self.block_fe = false;
                }
            }
            UCR => self.command(v),
            URB => self.send(v, cx.now),
            UISR => self.uimr = v,
            _ => {}
        }
        self.drive(cx);
    }

    fn next_event(&self) -> Option<u64> {
        let rx = self.rx_pending.first().map(|&(t, _)| t);
        let tx = self.move_at();
        let peer = self.peer.as_ref().and_then(|p| p.next_event());
        [rx, tx, peer].into_iter().flatten().min()
    }

    /// The peer's event when due, then the transmitter (whose character the peer may answer), then the characters
    /// that have arrived.
    fn advance(&mut self, now: u64, cx: &mut DeviceCx) {
        if let Some(p) = &mut self.peer
            && p.next_event().is_some_and(|t| t <= now)
        {
            let tpb = self.config.ticks_per_byte;
            p.advance(&mut Self::peer_cx(&mut self.rx_pending, tpb, now, cx));
        }
        self.catch_up(cx);
        let due = self.rx_pending.partition_point(|&(t, _)| t <= now);
        let later = self.rx_pending.split_off(due);
        for (_, b) in std::mem::replace(&mut self.rx_pending, later) {
            self.arrive(b);
        }
        self.drive(cx);
    }

    fn poll_counted(&self) -> bool {
        false
    }

    /// Its own source, when UIMR enables the request a pending character or the character in the holding register
    /// will raise. A character the peer could still send counts as pending: the peer has an event of its own, or
    /// a character in the holding register will reach it.
    /// The receive DMA's interrupts are listed by the eDMA.
    fn wake_sources(&self, out: &mut Vec<(usize, u8)>) -> bool {
        let from_peer = self
            .peer
            .as_ref()
            .is_some_and(|p| p.next_event().is_some() || self.holding.is_some());
        let rx = self.uimr & 2 != 0 && (!self.rx_pending.is_empty() || from_peer);
        let tx = self.uimr & 1 != 0 && self.holding.is_some();
        if rx || tx {
            out.push((self.config.controller, self.config.source));
        }
        true
    }
}
