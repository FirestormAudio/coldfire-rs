//! DMA timers (RM ch. 39) as free-running timestamp counters ([`Timestamp`]): a 32-bit count of the internal bus
//! clock (or the bus clock / 16) through a prescaler of 1 to 256, for timing finer than embassy-time's 1 kHz.
//!
//! The counter runs free from 0 (DTRR 0xFFFF_FFFF, no restart at the reference) and wraps after 2^32 counts:
//! 71.6 s at a 60 MHz bus with prescaler 1. [`Timestamp::elapsed`] is right across at most one wrap, so a span
//! longer than the wrap period needs a larger prescaler or embassy-time. Interrupts, DMA requests, input capture,
//! the output modes and the DTnIN input clock are not offered.
use core::marker::PhantomData;

use embassy_hal_internal::{Peri, PeripheralType};

use crate::clocks::{self, Gate};

// Registers (RM Table 39-1): DTMR (16-bit), DTXMR and DTER (bytes), DTRR, DTCR and DTCN (longwords).
const DTMR: usize = 0x0;
const DTXMR: usize = 0x2;
const DTER: usize = 0x3;
const DTRR: usize = 0x4;
const DTCN: usize = 0xC;
/// DTMR RST: the timer runs (RM Table 39-2).
const DTMR_RST: u16 = 1;
/// DTER CAP and REF: write 1 to clear (RM Table 39-4).
const DTER_CLEAR: u8 = 0x03;

/// The input clock (DTMR CLK, RM Table 39-2). The DTnIN pin may join these, so a match on it needs a wildcard arm.
#[non_exhaustive]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Clock {
    /// The internal bus clock.
    Bus,
    /// The internal bus clock divided by 16.
    BusDiv16,
}

impl Clock {
    const fn bits(self) -> u16 {
        match self {
            Clock::Bus => 0b01 << 1,
            Clock::BusDiv16 => 0b10 << 1,
        }
    }

    const fn divider(self) -> u32 {
        match self {
            Clock::Bus => 1,
            Clock::BusDiv16 => 16,
        }
    }
}

/// Counter configuration.
#[non_exhaustive]
#[derive(Copy, Clone, Debug)]
pub struct Config {
    pub clock: Clock,
    /// Counts of the input clock per count of the timer, 1 to 256 (DTMR PS + 1).
    pub prescale: u16,
}

impl Default for Config {
    /// The bus clock, prescaler 1.
    fn default() -> Self {
        Self { clock: Clock::Bus, prescale: 1 }
    }
}

impl Config {
    pub const fn new(clock: Clock, prescale: u16) -> Self {
        Self { clock, prescale }
    }

    /// DTMR for this configuration, the timer running free (FRR, ORRI, OM and CE clear). Panics on a prescaler
    /// outside 1 to 256.
    const fn dtmr(self) -> u16 {
        assert!(self.prescale >= 1 && self.prescale <= 256, "DTIM prescaler: 1 to 256");
        (self.prescale - 1) << 8 | self.clock.bits() | DTMR_RST
    }

    /// The count rate from a `bus_hz` bus clock.
    const fn hz(self, bus_hz: u32) -> u32 {
        bus_hz / self.clock.divider() / self.prescale as u32
    }
}

pub(crate) mod sealed {
    pub trait Instance {
        fn base() -> usize;
        const GATE: crate::clocks::Gate;
    }
}

/// A DMA timer.
#[allow(private_bounds)]
pub trait Instance: sealed::Instance + PeripheralType {}

macro_rules! instance {
    ($name:ident, $gate:ident) => {
        impl sealed::Instance for crate::peripherals::$name {
            fn base() -> usize {
                crate::pac::$name.as_ptr() as usize
            }
            const GATE: Gate = Gate::$gate;
        }
        impl Instance for crate::peripherals::$name {}
    };
}

instance!(DTIM0, Dtim0);
instance!(DTIM1, Dtim1);
instance!(DTIM2, Dtim2);
instance!(DTIM3, Dtim3);

/// A DMA timer counting freely from 0.
pub struct Timestamp<'d> {
    base: usize,
    hz: u32,
    _p: PhantomData<&'d mut ()>,
}

impl<'d> Timestamp<'d> {
    /// Ungate the timer, reset it, and start it counting from 0 at `config`'s rate. Panics on a prescaler outside
    /// 1 to 256.
    pub fn new<T: Instance>(_dtim: Peri<'d, T>, config: Config) -> Self {
        let dtmr = config.dtmr();
        clocks::ungate(T::GATE);
        let b = T::base();
        // SAFETY: the token makes this timer ours
        unsafe {
            // RST low resets the timer (RM Table 39-2); then the reference and count, and start
            coldfire::mmio::w16(b + DTMR, 0);
            coldfire::mmio::w8(b + DTXMR, 0);
            coldfire::mmio::w32(b + DTRR, 0xFFFF_FFFF);
            coldfire::mmio::w8(b + DTER, DTER_CLEAR);
            coldfire::mmio::w32(b + DTCN, 0);
            coldfire::mmio::w16(b + DTMR, dtmr);
        }
        Self { base: b, hz: config.hz(clocks::clocks().bus_hz), _p: PhantomData }
    }

    /// The count (DTCN).
    pub fn now(&self) -> u32 {
        // SAFETY: a read of our timer's counter
        unsafe { coldfire::mmio::r32(self.base + DTCN) }
    }

    /// Counts per second.
    pub fn hz(&self) -> u32 {
        self.hz
    }

    /// Counts since `since` (an earlier [`now`](Self::now)), right across at most one wrap.
    pub fn elapsed(&self, since: u32) -> u32 {
        self.now().wrapping_sub(since)
    }

    /// `ticks` counts in microseconds, rounded down.
    pub fn ticks_to_micros(&self, ticks: u32) -> u64 {
        to_micros(ticks, self.hz)
    }
}

const fn to_micros(ticks: u32, hz: u32) -> u64 {
    ticks as u64 * 1_000_000 / hz as u64
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_rate_follows_the_clock_and_prescaler() {
        assert_eq!(Config::default().hz(60_000_000), 60_000_000);
        assert_eq!(Config::new(Clock::Bus, 60).hz(60_000_000), 1_000_000);
        assert_eq!(Config::new(Clock::BusDiv16, 1).hz(60_000_000), 3_750_000);
        assert_eq!(Config::new(Clock::BusDiv16, 256).hz(64_000_000), 15_625);
    }

    #[test]
    fn dtmr_holds_ps_clk_and_rst() {
        // RM Table 39-2: PS 15:8, CLK 2:1, RST 0; FRR, ORRI, OM and CE clear
        assert_eq!(Config::default().dtmr(), 0x0003);
        assert_eq!(Config::new(Clock::BusDiv16, 256).dtmr(), 0xFF05);
    }

    #[test]
    #[should_panic(expected = "1 to 256")]
    fn a_prescaler_of_0_is_refused() {
        Config::new(Clock::Bus, 0).dtmr();
    }

    #[test]
    #[should_panic(expected = "1 to 256")]
    fn a_prescaler_over_256_is_refused() {
        Config::new(Clock::Bus, 257).dtmr();
    }

    #[test]
    fn ticks_become_microseconds() {
        assert_eq!(to_micros(60_000, 60_000_000), 1000);
        assert_eq!(to_micros(u32::MAX, 60_000_000), 71_582_788);
    }
}
