//! UARTs (RM ch. 36): 8N1 at a configured baud rate on the internal bus clock, blocking `Uart` and an
//! interrupt/eDMA-driven `BufferedUart` (`buffered`).
use core::marker::PhantomData;

use embassy_hal_internal::{Peri, PeripheralType};

use crate::clocks::{self, Gate};
use crate::interrupt::{typelevel, Priority};
use crate::pinmux;
use crate::{dma, pac};

pub mod buffered;
#[rustfmt::skip]
mod pins;

pub use buffered::{BufferedInterruptHandler, BufferedUart, BufferedUartRx, BufferedUartTx, RxDmaInterruptHandler};

/// Line settings. 8 data bits, no parity, 1 stop bit.
#[non_exhaustive]
#[derive(Copy, Clone, Debug)]
pub struct Config {
    pub baudrate: u32,
    /// Interrupt level of the port's interrupts (`BufferedUart`): level 5 by default.
    pub priority: Priority,
}

impl Config {
    pub const fn new(baudrate: u32) -> Self {
        Self { baudrate, priority: Priority::P5 }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ConfigError {
    /// The divider would be 0.
    BaudrateTooHigh,
    /// The divider would exceed 16 bits (or the rate is 0).
    BaudrateTooLow,
}

/// A receive error.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Error {
    /// USR error bits (OE 0x10, PE 0x20, FE 0x40), cleared after reporting.
    Status(u8),
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::Status(bits) => write!(f, "UART error status {bits:#04x}"),
        }
    }
}

impl core::error::Error for Error {}

impl embedded_io_async::Error for Error {
    fn kind(&self) -> embedded_io_async::ErrorKind {
        embedded_io_async::ErrorKind::Other
    }
}

/// UBG1:UBG2 for `baud`: bus / (32 × baud), rounded to the nearest (u32 arithmetic: no 64-bit division).
pub(crate) const fn divider(bus_hz: u32, baud: u32) -> Result<u16, ConfigError> {
    if baud == 0 {
        return Err(ConfigError::BaudrateTooLow);
    }
    let Some(den) = baud.checked_mul(32) else {
        return Err(ConfigError::BaudrateTooHigh);
    };
    // bus / den rounded: quotient, plus one if the remainder is at least half of den
    let q = bus_hz / den;
    let d = q + ((bus_hz % den) >= den - den / 2) as u32;
    if d == 0 {
        Err(ConfigError::BaudrateTooHigh)
    } else if d > 0xFFFF {
        Err(ConfigError::BaudrateTooLow)
    } else {
        Ok(d as u16)
    }
}

/// Check `baud` against the bus clock before anything is touched.
pub(crate) fn check_baud(baud: u32) -> Result<(), ConfigError> {
    divider(clocks::clocks().bus_hz, baud).map(|_| ())
}

/// How long line setup waits for a transmitter still sending (a console sharing the port) before resetting it.
pub(crate) const TX_IDLE_SPINS: u32 = 200_000;

/// How many USR polls line setup spends waiting for TXEMP: none unless an earlier line setup turned the
/// transmitter on (with it off, TXEMP reads 0 and the wait would always run to the bound).
pub(crate) const fn drain_spins(tx_was_on: bool) -> u32 {
    if tx_was_on { TX_IDLE_SPINS } else { 0 }
}

const USR_ERRORS: u8 = 0x70;
/// UMR1: RXRDY requests (RXIRQ 0), block error mode (ERR 1), no parity (PM 0b10), 8 bits (B/C 0b11).
const UMR1: u8 = 0x33;

pub(crate) mod sealed {
    pub trait Instance {
        const REGS: crate::pac::uart0::Uart0;
        const GATE: crate::clocks::Gate;
        fn buffered() -> &'static super::buffered::State;
        fn rx_ring() -> &'static super::buffered::RxRing;
        /// Set once `line_setup` has turned this port's transmitter on.
        fn tx_on() -> &'static core::sync::atomic::AtomicBool;
    }
    pub trait TxPin<T> {
        const PAR: crate::pinmux::ParField;
        const FUNCTION: crate::pinmux::Function;
    }
    pub trait RxPin<T> {
        const PAR: crate::pinmux::ParField;
        const FUNCTION: crate::pinmux::Function;
    }
}

/// A UART token.
#[allow(private_bounds)]
pub trait Instance: sealed::Instance + PeripheralType + 'static {
    type Interrupt: typelevel::Interrupt;
    /// The eDMA channel this UART's receiver requests on (fixed by the request map).
    type RxDma: dma::ChannelInstance;
}

/// A pad that can carry `T`'s TXD.
#[allow(private_bounds)]
pub trait TxPin<T: Instance>: sealed::TxPin<T> + PeripheralType {}
/// A pad that can carry `T`'s RXD.
#[allow(private_bounds)]
pub trait RxPin<T: Instance>: sealed::RxPin<T> + PeripheralType {}

macro_rules! instance {
    ($(#[$m:meta])* $name:ident, $gate:ident, $rx:ident) => {
        $(#[$m])*
        impl sealed::Instance for crate::peripherals::$name {
            const REGS: pac::uart0::Uart0 = pac::$name;
            const GATE: Gate = Gate::$gate;
            fn buffered() -> &'static buffered::State {
                static STATE: buffered::State = buffered::State::new();
                &STATE
            }
            fn rx_ring() -> &'static buffered::RxRing {
                static RING: buffered::RxRing = buffered::RxRing::new();
                &RING
            }
            fn tx_on() -> &'static core::sync::atomic::AtomicBool {
                static TX_ON: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);
                &TX_ON
            }
        }
        $(#[$m])*
        impl Instance for crate::peripherals::$name {
            type Interrupt = typelevel::$name;
            type RxDma = crate::peripherals::$rx;
        }
    };
}

// RX request channels from the eDMA request map (RM Table 19-6).
instance!(UART0, Uart0, DMA_CH2);
instance!(UART1, Uart1, DMA_CH4);
instance!(#[cfg(any(feature = "mcf54415", feature = "mcf54416", feature = "mcf54417", feature = "mcf54418"))] UART2, Uart2, DMA_CH6);
instance!(UART3, Uart3, DMA_CH16);
instance!(UART4, Uart4, DMA_CH18);
instance!(UART5, Uart5, DMA_CH20);
instance!(#[cfg(any(feature = "mcf54415", feature = "mcf54416", feature = "mcf54417", feature = "mcf54418"))] UART6, Uart6, DMA_CH22);
instance!(UART7, Uart7, DMA_CH32);
instance!(UART8, Uart8, DMA_CH34);
instance!(#[cfg(any(feature = "mcf54415", feature = "mcf54416", feature = "mcf54417", feature = "mcf54418"))] UART9, Uart9, DMA_CH36);

pub(crate) fn mux_tx<T: Instance, P: TxPin<T>>(_pin: &Peri<'_, P>) {
    pinmux::set_function(<P as sealed::TxPin<T>>::PAR, <P as sealed::TxPin<T>>::FUNCTION);
}

pub(crate) fn mux_rx<T: Instance, P: RxPin<T>>(_pin: &Peri<'_, P>) {
    pinmux::set_function(<P as sealed::RxPin<T>>::PAR, <P as sealed::RxPin<T>>::FUNCTION);
}

/// Ungate the UART and set the line up as proven on hardware: reset receiver, transmitter and mode pointer,
/// both clocks from the bus (UCSR 0xDD), 8N1 (UMR1 0x33, UMR2 0x07), the divider, transmitter on. The receiver
/// stays off. If an earlier line setup left the transmitter on (a console on this port), waits (boundedly) for it
/// to drain before resetting it. Hardware ran UMR1 0x13, the same line with character error mode; block error
/// mode (ERR) is from the RM.
pub(crate) fn line_setup<T: Instance>(baud: u32) -> Result<(), ConfigError> {
    let d = divider(clocks::clocks().bus_hz, baud)?;
    clocks::ungate(T::GATE);
    let r = T::REGS;
    for _ in 0..drain_spins(T::tx_on().load(core::sync::atomic::Ordering::Relaxed)) {
        if r.usr().read().txemp() {
            break;
        }
    }
    r.ucr().write_value(pac::uart0::regs::Ucr(0x20)); // reset receiver
    r.ucr().write_value(pac::uart0::regs::Ucr(0x30)); // reset transmitter
    r.ucr().write_value(pac::uart0::regs::Ucr(0x10)); // reset mode-register pointer
    r.ucsr().write_value(pac::uart0::regs::Ucsr(0xDD));
    // guess: UMR1[ERR] = 1, block mode (RM 41.4.2.3): FE, PE and RB accumulate in USR until RESET ERROR STATUS
    // (UCR 0x40, `take_error`). In character mode they belong to the character at the top of the FIFO, so receive
    // DMA takes the status away with the character and the driver never sees a framing error.
    r.umr1().write_value(pac::uart0::regs::Umr1(UMR1));
    r.umr2().write_value(pac::uart0::regs::Umr2(0x07));
    r.ubg1().write_value((d >> 8) as u8);
    r.ubg2().write_value(d as u8);
    r.ucr().write_value(pac::uart0::regs::Ucr(0x04)); // transmitter on
    T::tx_on().store(true, core::sync::atomic::Ordering::Relaxed);
    Ok(())
}

/// Turn the receiver on (UCR 0x01).
pub(crate) fn enable_rx(r: pac::uart0::Uart0) {
    r.ucr().write_value(pac::uart0::regs::Ucr(0x01));
}

/// The receiver's error bits, cleared (UCR 0x40) if any were set.
pub(crate) fn take_error(r: pac::uart0::Uart0) -> Option<Error> {
    let e = r.usr().read().0 & USR_ERRORS;
    if e == 0 {
        return None;
    }
    r.ucr().write_value(pac::uart0::regs::Ucr(0x40));
    Some(Error::Status(e))
}

/// Blocking transmitter.
pub struct UartTx<'d> {
    r: pac::uart0::Uart0,
    _p: PhantomData<&'d mut ()>,
}

/// Blocking receiver.
pub struct UartRx<'d> {
    r: pac::uart0::Uart0,
    _p: PhantomData<&'d mut ()>,
}

/// Blocking UART, both directions.
pub struct Uart<'d> {
    tx: UartTx<'d>,
    rx: UartRx<'d>,
}

impl<'d> UartTx<'d> {
    /// Transmit only: ungate, mux `tx`, set the line up.
    pub fn new<T: Instance>(_uart: Peri<'d, T>, tx: Peri<'d, impl TxPin<T>>, config: Config) -> Result<Self, ConfigError> {
        check_baud(config.baudrate)?;
        mux_tx::<T, _>(&tx);
        line_setup::<T>(config.baudrate)?;
        Ok(Self { r: T::REGS, _p: PhantomData })
    }

    /// Transmit only, leaving the pads alone: set the line up but select no pin. The bytes reach a pad only if
    /// something muxes one (a console that must not claim a pin another function shares, like MIDI OUT).
    pub fn new_unmuxed<T: Instance>(_uart: Peri<'d, T>, config: Config) -> Result<Self, ConfigError> {
        check_baud(config.baudrate)?;
        line_setup::<T>(config.baudrate)?;
        Ok(Self { r: T::REGS, _p: PhantomData })
    }

    /// A transmitter on a port that something else set up and may own (a console printing on a port a
    /// `BufferedUart` also sends on). Configures nothing.
    ///
    /// # Safety
    /// The port must be set up before anything is written. Writes race the owner's: if the owner's TXRDY
    /// interrupt fills UTB between this side's TXRDY poll and its write, one of the two bytes is lost. The
    /// caller accepts that (a debug console does).
    pub unsafe fn steal<T: Instance>() -> UartTx<'static> {
        UartTx { r: T::REGS, _p: PhantomData }
    }

    /// Send `buf`, waiting for TXRDY before each byte.
    pub fn blocking_write(&mut self, buf: &[u8]) {
        for &b in buf {
            while !self.r.usr().read().txrdy() {}
            self.r.utb().write(|w| w.set_tb(b));
        }
    }

    /// Wait until the last byte has left the shift register (TXEMP).
    pub fn blocking_flush(&mut self) {
        while !self.r.usr().read().txemp() {}
    }
}

impl core::fmt::Write for UartTx<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.blocking_write(s.as_bytes());
        Ok(())
    }
}

impl embedded_io::ErrorType for UartTx<'_> {
    type Error = core::convert::Infallible;
}

impl embedded_io::Write for UartTx<'_> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.blocking_write(buf);
        Ok(buf.len())
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        self.blocking_flush();
        Ok(())
    }
}

impl embedded_io::ErrorType for UartRx<'_> {
    type Error = Error;
}

impl embedded_io::Read for UartRx<'_> {
    /// Waits for the first byte, then returns what has already arrived. A receive error is reported by the call
    /// that meets it with nothing read yet: bytes before it are returned first.
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        let mut n = 0;
        while n < buf.len() {
            if self.r.usr().read().0 & USR_ERRORS != 0 {
                if n > 0 {
                    break;
                }
                return Err(take_error(self.r).unwrap_or(Error::Status(0)));
            }
            if self.r.usr().read().rxrdy() {
                buf[n] = self.r.urb().read().rb();
                n += 1;
            } else if n > 0 {
                break;
            }
        }
        Ok(n)
    }
}

impl<'d> UartRx<'d> {
    /// Fill `buf`, waiting for RXRDY before each byte; a receive error ends the read. The bytes before the error
    /// are in `buf` but not counted: use `embedded_io::Read::read` when the count matters.
    pub fn blocking_read(&mut self, buf: &mut [u8]) -> Result<(), Error> {
        for b in buf {
            loop {
                if let Some(e) = take_error(self.r) {
                    return Err(e);
                }
                if self.r.usr().read().rxrdy() {
                    break;
                }
            }
            *b = self.r.urb().read().rb();
        }
        Ok(())
    }
}

impl<'d> Uart<'d> {
    pub fn new<T: Instance>(
        _uart: Peri<'d, T>,
        tx: Peri<'d, impl TxPin<T>>,
        rx: Peri<'d, impl RxPin<T>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        check_baud(config.baudrate)?;
        mux_tx::<T, _>(&tx);
        mux_rx::<T, _>(&rx);
        line_setup::<T>(config.baudrate)?;
        enable_rx(T::REGS);
        Ok(Self { tx: UartTx { r: T::REGS, _p: PhantomData }, rx: UartRx { r: T::REGS, _p: PhantomData } })
    }

    pub fn split(self) -> (UartTx<'d>, UartRx<'d>) {
        (self.tx, self.rx)
    }

    pub fn blocking_write(&mut self, buf: &[u8]) {
        self.tx.blocking_write(buf)
    }

    pub fn blocking_flush(&mut self) {
        self.tx.blocking_flush()
    }

    pub fn blocking_read(&mut self, buf: &mut [u8]) -> Result<(), Error> {
        self.rx.blocking_read(buf)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn divider_midi() {
        assert_eq!(divider(125_000_000, 31_250), Ok(125)); // 125 MHz / (32 × 31250)
    }

    #[test]
    fn divider_115200_rounds_to_nearest() {
        assert_eq!(divider(125_000_000, 115_200), Ok(34)); // 33.9 rounds to 34
    }

    #[test]
    fn divider_out_of_range() {
        // 125 MHz / (32 × 9 MHz) = 0.43 → 0: too fast
        assert_eq!(divider(125_000_000, 9_000_000), Err(ConfigError::BaudrateTooHigh));
        // 125 MHz / (32 × 50) = 78125 > 0xFFFF: too slow
        assert_eq!(divider(125_000_000, 50), Err(ConfigError::BaudrateTooLow));
        assert_eq!(divider(125_000_000, 0), Err(ConfigError::BaudrateTooLow));
    }

    #[test]
    fn divider_huge_rates_are_too_high() {
        assert_eq!(divider(125_000_000, u32::MAX), Err(ConfigError::BaudrateTooHigh));
        assert_eq!(divider(u32::MAX, 1_000_000), Ok(134)); // 4294967295 / 32e6 = 134.2
    }

    #[test]
    fn the_blocking_receiver_is_an_embedded_io_reader() {
        fn r<T: embedded_io::Read>() {}
        r::<UartRx<'static>>();
    }

    #[test]
    fn line_setup_is_8n1_with_receive_errors_in_block_mode() {
        // guess: block mode is from the RM (41.4.2.3); hardware ran character mode (0x13)
        assert_eq!(UMR1 & 0x20, 0x20, "ERR: errors stay in USR until RESET ERROR STATUS");
        assert_eq!(UMR1 & 0x1F, 0x13, "no parity, 8 bits");
        assert_eq!(UMR1 & 0xC0, 0, "no RTS control, RXRDY interrupt");
    }

    #[test]
    fn line_setup_waits_only_for_a_transmitter_it_turned_on() {
        // with the transmitter off (reset, fresh ungate) TXEMP reads 0 (RM Table 41-5): never wait then
        assert_eq!(drain_spins(false), 0);
        assert_eq!(drain_spins(true), TX_IDLE_SPINS);
        assert!(TX_IDLE_SPINS > 0 && TX_IDLE_SPINS <= 200_000);
    }
}
