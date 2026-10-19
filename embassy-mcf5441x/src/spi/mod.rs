//! DSPI (RM ch. 40) as a blocking SPI master.
//!
//! - [`Spi`] is a bus with no chip select of its own ([`SpiBus`]): select the device some other way, e.g. a GPIO
//!   output through embedded-hal-bus.
//! - [`PcsDevice`] is one device on one of the module's own chip selects ([`SpiDevice`]). The select follows the
//!   frames (PUSHR CONT), so a transaction is one queue: the select is held from its first frame to its last.
//!
//! Frames are 8 bits through CTAR0, with up to `IN_FLIGHT` queued ahead of the one being received. With
//! [`Config::wide_reads`] (off by default), reads move two bytes a frame through CTAR1 (the same timing at 16 bits).
//! Every chip select the instance has idles high, so a bare [`Spi`] never selects a device on a muxed PCS pad. Nothing here
//! uses interrupts, eDMA or statics, so a binary with no runtime (a BDM call) can drive a device with it.
use core::marker::PhantomData;

use embassy_hal_internal::{Peri, PeripheralType};
use embedded_hal::spi::{ErrorKind, ErrorType, Operation, SpiBus, SpiDevice};

use crate::clocks::{self, Gate};
use crate::pac;
use crate::pac::dspi0::regs;
use crate::pinmux;

mod ctar;
#[rustfmt::skip]
mod pins;

pub use ctar::{BaudPrescaler, ConfigError, Ctar, DelayPrescaler, Mode};

/// Frames queued ahead of the one being received: the RX FIFO holds 16 and SR[RXCTR] is 4 bits wide.
const IN_FLIGHT: usize = 12;

const MCR_MSTR: u32 = 1 << 31;
const MCR_CLR_FIFOS: u32 = (1 << 11) | (1 << 10);
const MCR_HALT: u32 = 1 << 0;
const SR_TCF: u32 = 1 << 31;
const SR_EOQF: u32 = 1 << 28;
const PUSHR_CONT: u32 = 1 << 31;
const PUSHR_CTAR1: u32 = 1 << 28;
const PUSHR_EOQ: u32 = 1 << 27;
const PUSHR_CTCNT: u32 = 1 << 26;

/// Bus settings.
#[non_exhaustive]
#[derive(Copy, Clone, Debug)]
pub struct Config {
    /// Mode, clock and delays.
    pub ctar: Ctar,
    /// Reads move two bytes a frame through CTAR1. Off by default: switching CTARs between frames under a held
    /// select is something the RM advises against, so a board opts in once it has seen it work. Off, every frame is
    /// 8 bits and CTAR1 is left alone.
    pub wide_reads: bool,
}

impl Config {
    pub const fn new(ctar: Ctar) -> Self {
        Self { ctar, wide_reads: false }
    }
}

/// A transfer error.
#[non_exhaustive]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Error {
    /// The transaction had an `Operation::DelayNs`: the driver has no timer to hold the select with. Nothing was sent.
    DelayUnsupported,
}

impl embedded_hal::spi::Error for Error {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

pub(crate) mod sealed {
    use crate::pinmux::{Function, ParField};

    pub trait Instance {
        const REGS: crate::pac::dspi0::Dspi0;
        const GATE: crate::clocks::Gate;
        /// The instance's chip-select lines, as MCR PCSIS bits.
        const PCS_LINES: u8;
    }
    pub trait SckPin<T> {
        const PAR: ParField;
        const FUNCTION: Function;
    }
    pub trait SoutPin<T> {
        const PAR: ParField;
        const FUNCTION: Function;
    }
    pub trait SinPin<T> {
        const PAR: ParField;
        const FUNCTION: Function;
    }
    pub trait PcsPin<T> {
        const PAR: ParField;
        const FUNCTION: Function;
        /// The chip-select line (PCSn).
        const PCS: u8;
    }
}

/// A DSPI token.
#[allow(private_bounds)]
pub trait Instance: sealed::Instance + PeripheralType + 'static {}
/// A pad that can carry `T`'s SCK.
#[allow(private_bounds)]
pub trait SckPin<T: Instance>: sealed::SckPin<T> + PeripheralType {}
/// A pad that can carry `T`'s SOUT.
#[allow(private_bounds)]
pub trait SoutPin<T: Instance>: sealed::SoutPin<T> + PeripheralType {}
/// A pad that can carry `T`'s SIN.
#[allow(private_bounds)]
pub trait SinPin<T: Instance>: sealed::SinPin<T> + PeripheralType {}
/// A pad that can carry one of `T`'s chip selects.
#[allow(private_bounds)]
pub trait PcsPin<T: Instance>: sealed::PcsPin<T> + PeripheralType {}

macro_rules! instance {
    ($name:ident, $gate:ident, $pcs:expr) => {
        impl sealed::Instance for crate::peripherals::$name {
            const REGS: pac::dspi0::Dspi0 = pac::$name;
            const GATE: Gate = Gate::$gate;
            const PCS_LINES: u8 = $pcs;
        }
        impl Instance for crate::peripherals::$name {}
    };
}

// DSPI0 has PCS0-3, DSPI1 PCS0-2, DSPI2 and DSPI3 PCS0 only
instance!(DSPI0, Dspi0, 0x0F);
instance!(DSPI1, Dspi1, 0x07);
instance!(DSPI2, Dspi2, 0x01);
instance!(DSPI3, Dspi3, 0x01);

/// Select SCK, SOUT and SIN on their pads (each its own PAR field; the register's other pads keep theirs).
fn mux_bus<T: Instance, C: SckPin<T>, O: SoutPin<T>, I: SinPin<T>>(_: &Peri<'_, C>, _: &Peri<'_, O>, _: &Peri<'_, I>) {
    pinmux::set_function(<C as sealed::SckPin<T>>::PAR, <C as sealed::SckPin<T>>::FUNCTION);
    pinmux::set_function(<O as sealed::SoutPin<T>>::PAR, <O as sealed::SoutPin<T>>::FUNCTION);
    pinmux::set_function(<I as sealed::SinPin<T>>::PAR, <I as sealed::SinPin<T>>::FUNCTION);
}

/// Frames an operation sends (nonzero exactly when it has bytes).
fn frames_of(op: &Operation<'_, u8>) -> usize {
    match op {
        Operation::Read(b) => b.len(),
        Operation::Write(b) => b.len(),
        Operation::Transfer(r, w) => r.len().max(w.len()),
        Operation::TransferInPlace(b) => b.len(),
        Operation::DelayNs(_) => 0,
    }
}

/// The last operation that sends a frame: its last frame ends the queue (EOQ) and releases the select.
fn last_sending(ops: &[Operation<'_, u8>]) -> Option<usize> {
    ops.iter().rposition(|o| frames_of(o) > 0)
}

/// The command half of one queue's PUSHR words.
struct Queue {
    /// Chip select and CONT (a `PcsDevice`), or 0 (the bare bus).
    cmd: u32,
    first: bool,
}

impl Queue {
    /// A frame word: `bits` (data, or CTAS) under the queue's command; CTCNT on the queue's first frame. The last
    /// frame ends the queue (EOQ) and releases the select (no CONT).
    #[inline(always)]
    fn word(&mut self, bits: u32, last: bool) -> u32 {
        let mut w = self.cmd | bits;
        if self.first {
            w |= PUSHR_CTCNT;
            self.first = false;
        }
        if last { (w & !PUSHR_CONT) | PUSHR_EOQ } else { w }
    }
}

/// A DSPI master with no chip select of its own.
pub struct Spi<'d> {
    r: pac::dspi0::Dspi0,
    wide: bool,
    _p: PhantomData<&'d mut ()>,
}

impl<'d> Spi<'d> {
    /// Ungate, mux SCK, SOUT and SIN, then master mode with both FIFOs on and `config`'s CTARs.
    pub fn new_blocking<T: Instance>(
        _spi: Peri<'d, T>,
        sck: Peri<'d, impl SckPin<T>>,
        sout: Peri<'d, impl SoutPin<T>>,
        sin: Peri<'d, impl SinPin<T>>,
        config: Config,
    ) -> Self {
        clocks::ungate(T::GATE);
        mux_bus::<T, _, _, _>(&sck, &sout, &sin);
        Self::setup::<T>(config)
    }

    /// Master mode with every chip select idle high (a PCS idling low would select its device between frames, and
    /// for a bare bus all the time) and both FIFOs cleared. The CTARs are written while the module is halted (they
    /// must not change while it runs).
    fn setup<T: Instance>(config: Config) -> Self {
        let r = T::REGS;
        let mcr = MCR_MSTR | (T::PCS_LINES as u32) << 16 | MCR_CLR_FIFOS;
        r.mcr().write_value(regs::Mcr(mcr | MCR_HALT));
        r.ctar0().write_value(regs::Ctar0(config.ctar.frames(8)));
        if config.wide_reads {
            r.ctar1().write_value(regs::Ctar1(config.ctar.frames(16)));
        }
        r.mcr().write_value(regs::Mcr(mcr));
        Self { r, wide: config.wide_reads, _p: PhantomData }
    }

    /// Run `ops` as one queue whose frames all carry `cmd` (chip select and CONT, or 0 on the bare bus).
    fn run(&mut self, cmd: u32, ops: &mut [Operation<'_, u8>]) -> Result<(), Error> {
        if ops.iter().any(|o| matches!(o, Operation::DelayNs(_))) {
            return Err(Error::DelayUnsupported);
        }
        let Some(last) = last_sending(ops) else { return Ok(()) };
        self.start();
        let mut q = Queue { cmd, first: true };
        for (i, op) in ops[..=last].iter_mut().enumerate() {
            let end = i == last;
            match op {
                Operation::Write(w) => {
                    let w: &[u8] = w;
                    self.frames(&mut q, w.len(), end, |k| w[k], |_, _| {})
                }
                Operation::Read(r) if self.wide => self.read_wide(&mut q, r, end),
                Operation::Read(r) => {
                    let n = r.len();
                    self.frames(&mut q, n, end, |_| 0, |k, v| r[k] = v)
                }
                Operation::Transfer(r, w) => {
                    let w: &[u8] = w;
                    let n = r.len().max(w.len());
                    // past the end of `write` the frames send 0; past the end of `read` the replies are dropped
                    self.frames(&mut q, n, end, |k| if k < w.len() { w[k] } else { 0 }, |k, v| {
                        if k < r.len() {
                            r[k] = v
                        }
                    })
                }
                Operation::TransferInPlace(b) => {
                    let (n, p) = (b.len(), b.as_mut_ptr());
                    // SAFETY: k < n; frame k's byte is read when it is pushed, before its reply overwrites it
                    self.frames(&mut q, n, end, |k| unsafe { *p.add(k) }, |k, v| unsafe { *p.add(k) = v })
                }
                Operation::DelayNs(_) => {}
            }
        }
        self.finish();
        Ok(())
    }

    /// A new queue: both FIFOs flushed, EOQF and TCF clear.
    fn start(&mut self) {
        let m = self.r.mcr().read().0;
        self.r.mcr().write_value(regs::Mcr(m | MCR_CLR_FIFOS));
        self.r.sr().write_value(regs::Sr(SR_EOQF | SR_TCF));
    }

    #[inline(always)]
    fn push(&mut self, w: u32) {
        self.r.pushr().write_value(regs::Pushr(w));
    }

    /// The next received frame (waits for it).
    #[inline(always)]
    fn pop(&mut self) -> u16 {
        while self.r.sr().read().rxctr() == 0 {}
        self.r.popr().read().rxdata()
    }

    /// Wait for the queue's last frame, then clear EOQF so the next queue can run.
    fn finish(&mut self) {
        while !self.r.sr().read().eoqf() {}
        self.r.sr().write_value(regs::Sr(SR_EOQF));
    }

    /// `n` 8-bit frames: `tx(k)` is frame k's byte and `rx(k, byte)` takes its reply, with up to `IN_FLIGHT`
    /// frames queued ahead.
    #[inline(always)]
    fn frames(&mut self, q: &mut Queue, n: usize, end: bool, tx: impl Fn(usize) -> u8, mut rx: impl FnMut(usize, u8)) {
        let mut sent = 0;
        for k in 0..n {
            while sent < n && sent < k + IN_FLIGHT {
                let w = q.word(tx(sent) as u32, end && sent + 1 == n);
                self.push(w);
                sent += 1;
            }
            rx(k, self.pop() as u8);
        }
    }

    /// `buf` in 16-bit frames through CTAR1, high byte first, and an 8-bit frame for an odd last byte. The steady
    /// state is one pop, two stores and one constant push per two bytes (it matters with code fetched uncached).
    fn read_wide(&mut self, q: &mut Queue, buf: &mut [u8], end: bool) {
        let words = buf.len() / 2;
        let n = words + buf.len() % 2;
        let kind = |k: usize| if k < words { PUSHR_CTAR1 } else { 0 };
        let p = buf.as_mut_ptr();
        let mut sent = 0;
        while sent < n && sent < IN_FLIGHT {
            let w = q.word(kind(sent), end && sent + 1 == n);
            self.push(w);
            sent += 1;
        }
        // every frame still to push in this loop is a plain 16-bit one: not the last, not the odd byte
        let steady = q.cmd | PUSHR_CTAR1;
        let mut k = 0;
        while sent + 1 < n && sent < words {
            let v = self.pop();
            // SAFETY: k < sent < words, so 2k + 1 < 2 × words <= buf.len()
            unsafe {
                *p.add(2 * k) = (v >> 8) as u8;
                *p.add(2 * k + 1) = v as u8;
            }
            k += 1;
            self.push(steady);
            sent += 1;
        }
        while sent < n {
            let w = q.word(kind(sent), end && sent + 1 == n);
            self.push(w);
            sent += 1;
        }
        while k < n {
            let v = self.pop();
            // SAFETY: a 16-bit frame (k < words) fills 2k and 2k + 1 < 2 × words; the odd frame fills 2k = len - 1
            unsafe {
                if k < words {
                    *p.add(2 * k) = (v >> 8) as u8;
                    *p.add(2 * k + 1) = v as u8;
                } else {
                    *p.add(2 * k) = v as u8;
                }
            }
            k += 1;
        }
    }
}

impl ErrorType for Spi<'_> {
    type Error = Error;
}

/// Each call is its own queue and returns once its last frame is in, so `flush` has nothing to wait for.
impl SpiBus<u8> for Spi<'_> {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Error> {
        self.run(0, &mut [Operation::Read(words)])
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Error> {
        self.run(0, &mut [Operation::Write(words)])
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Error> {
        self.run(0, &mut [Operation::Transfer(read, write)])
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Error> {
        self.run(0, &mut [Operation::TransferInPlace(words)])
    }

    fn flush(&mut self) -> Result<(), Error> {
        Ok(())
    }
}

/// One device on one of the DSPI's own chip selects.
pub struct PcsDevice<'d> {
    spi: Spi<'d>,
    /// The PUSHR chip-select bit, with CONT.
    cmd: u32,
}

impl<'d> PcsDevice<'d> {
    /// [`Spi::new_blocking`] plus the chip select `cs`, its pad muxed.
    pub fn new_blocking<T: Instance, C: PcsPin<T>>(
        _spi: Peri<'d, T>,
        sck: Peri<'d, impl SckPin<T>>,
        sout: Peri<'d, impl SoutPin<T>>,
        sin: Peri<'d, impl SinPin<T>>,
        _cs: Peri<'d, C>,
        config: Config,
    ) -> Self {
        clocks::ungate(T::GATE);
        mux_bus::<T, _, _, _>(&sck, &sout, &sin);
        pinmux::set_function(<C as sealed::PcsPin<T>>::PAR, <C as sealed::PcsPin<T>>::FUNCTION);
        let pcs = <C as sealed::PcsPin<T>>::PCS;
        Self { spi: Spi::setup::<T>(config), cmd: PUSHR_CONT | 1 << (16 + pcs as u32) }
    }
}

impl ErrorType for PcsDevice<'_> {
    type Error = Error;
}

/// A transaction is one queue: the select is asserted from its first frame to its last.
impl SpiDevice<u8> for PcsDevice<'_> {
    fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Error> {
        self.spi.run(self.cmd, operations)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_queue_ends_on_the_last_operation_that_sends() {
        let (mut three, mut none) = ([0u8; 3], [0u8; 0]);
        assert_eq!(last_sending(&[Operation::Write(&[0x9F]), Operation::Read(&mut three), Operation::Read(&mut none)]), Some(1));
        assert_eq!(last_sending(&[Operation::Read(&mut []), Operation::DelayNs(5)]), None);
        assert_eq!(last_sending(&[Operation::Transfer(&mut [], &[1]), Operation::Write(&[])]), Some(0));
        assert_eq!(last_sending(&[]), None);
    }

    #[test]
    fn config_starts_with_8_bit_frames_only() {
        // 16-bit reads switch CTARs under a held select, which the RM advises against: opt in, don't default
        assert!(!Config::new(Ctar::new(Mode::Mode0)).wide_reads);
    }

    #[test]
    fn every_chip_select_of_an_instance_idles_high() {
        assert_eq!(<crate::peripherals::DSPI0 as sealed::Instance>::PCS_LINES, 0x0F);
        assert_eq!(<crate::peripherals::DSPI1 as sealed::Instance>::PCS_LINES, 0x07);
        assert_eq!(<crate::peripherals::DSPI2 as sealed::Instance>::PCS_LINES, 0x01);
        assert_eq!(<crate::peripherals::DSPI3 as sealed::Instance>::PCS_LINES, 0x01);
    }

    #[test]
    fn a_frame_word_carries_the_queue_command_ctcnt_first_and_eoq_last() {
        // PCS1 with CONT: the bootloader's first READ frame is 0x84020003
        let mut q = Queue { cmd: PUSHR_CONT | 1 << 17, first: true };
        assert_eq!(q.word(0x03, false), 0x8402_0003);
        assert_eq!(q.word(0x12, false), 0x8002_0012);
        assert_eq!(q.word(PUSHR_CTAR1, true), 0x1802_0000); // last: no CONT, EOQ
        let mut bus = Queue { cmd: 0, first: true };
        assert_eq!(bus.word(0x55, true), 0x0C00_0055);
    }
}
