//! Clocks: the PLL's outputs as the bootloader left them, PIT reload values, and the peripheral clock gates
//! (SCM PPMCR0: writing a module's slot number enables its clock).
use core::cell::Cell;

use critical_section::Mutex;

use crate::pac;
use crate::pac::pll::regs::{Cr, Dr};

/// What the PLL feeds the chip.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Clocks {
    pub vco_hz: u32,
    pub core_hz: u32,
    pub bus_hz: u32,
}

impl Clocks {
    /// fVCO = fREF × (FBKDIV + 1) / (2 if REFDIV = 1, else 1: 0 is ÷1 and the rest are reserved, used as ÷1);
    /// core = fVCO / (OUTDIV1 + 1); internal bus = fVCO / (OUTDIV2 + 1).
    pub const fn from_pll(ref_hz: u32, cr: Cr, dr: Dr) -> Self {
        let refdiv = if cr.refdiv() == 1 { 2 } else { 1 };
        let vco = (ref_hz as u64 * (cr.fbkdiv() as u64 + 1) / refdiv) as u32;
        Self { vco_hz: vco, core_hz: vco / (dr.outdiv1() as u32 + 1), bus_hz: vco / (dr.outdiv2() as u32 + 1) }
    }
}

static CLOCKS: Mutex<Cell<Option<Clocks>>> = Mutex::new(Cell::new(None));

pub(crate) fn set(c: Clocks) {
    critical_section::with(|cs| CLOCKS.borrow(cs).set(Some(c)));
}

/// The clocks `init` found. Panics before `init`.
pub fn clocks() -> Clocks {
    critical_section::with(|cs| CLOCKS.borrow(cs).get()).expect("embassy_mcf5441x::init not called")
}

/// PIT prescaler and modulus for `tick_hz` from `bus_hz`: the smallest prescaler 2^PRE (0..=15) whose count fits
/// PMR's 16 bits; one period is (PMR + 1) << PRE bus cycles.
pub const fn pit_reload(bus_hz: u32, tick_hz: u32) -> (u8, u16) {
    let mut pre = 0;
    while pre <= 15 {
        let div = (tick_hz as u64) << pre;
        if bus_hz as u64 % div == 0 {
            let n = bus_hz as u64 / div;
            if n >= 1 && n <= 65_536 {
                return (pre as u8, (n - 1) as u16);
            }
        }
        pre += 1;
    }
    panic!("PIT can't divide the bus clock into this tick rate exactly");
}

const BANK1: u8 = 0x80;

/// Module clock gates (slot numbers from RM ch. 9). Bank 0 is PPMCR0; bank 1 (`0x80 | slot`) is PPMCR1. More gates
/// are added as drivers need them, so a match on it needs a wildcard arm.
#[non_exhaustive]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Gate {
    Dspi1 = 15,
    Dspi0 = 23,
    Edma = 17,
    Intc0 = 18,
    Intc1 = 19,
    Intc2 = 20,
    Uart0 = 24,
    Uart1 = 25,
    Uart2 = 26,
    Uart3 = 27,
    Dtim0 = 28,
    Dtim1 = 29,
    Dtim2 = 30,
    Dtim3 = 31,
    Pit0 = 32,
    Pit1 = 33,
    Pit2 = 34,
    Pit3 = 35,
    Eport = 36,
    UsbOtg = 44,
    Ssi0 = 47,
    Ssi1 = 50,
    Esdhc = 51,
    Dspi2 = BANK1 | 14,
    Dspi3 = BANK1 | 15,
    Uart4 = BANK1 | 24,
    Uart5 = BANK1 | 25,
    Uart6 = BANK1 | 26,
    Uart7 = BANK1 | 27,
    Uart8 = BANK1 | 28,
    Uart9 = BANK1 | 29,
}

/// (bank, slot) of a gate.
pub(crate) const fn bank_slot(g: Gate) -> (u8, u8) {
    let v = g as u8;
    ((v & BANK1 != 0) as u8, v & !BANK1)
}

/// Enable a module's clock: write its slot to PPMCR0 or PPMCR1.
pub fn ungate(g: Gate) {
    match bank_slot(g) {
        (0, slot) => pac::SCM.ppmcr0().write(|w| w.set_cmcd(slot)),
        (_, slot) => pac::SCM.ppmcr().write(|w| w.set_cmcd(slot)),
    }
}

/// Learn the clocks from the PLL (as the bootloader left it) and remember them; returns them. `init` calls this;
/// call it earlier yourself when something needs the clocks before `init` (a console that prints init's panics).
pub fn init(ref_hz: u32) -> Clocks {
    let c = Clocks::from_pll(ref_hz, pac::PLL.cr().read(), pac::PLL.dr().read());
    set(c);
    c
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pac::pll::regs::{Cr, Dr};

    #[test]
    fn esdhc_gate() {
        assert_eq!(bank_slot(Gate::Esdhc), (0, 51));
    }

    #[test]
    fn usb_otg_gate() {
        assert_eq!(bank_slot(Gate::UsbOtg), (0, 44));
    }

    #[test]
    fn dspi_gates() {
        assert_eq!(bank_slot(Gate::Dspi0), (0, 23));
        assert_eq!(bank_slot(Gate::Dspi1), (0, 15));
        assert_eq!(bank_slot(Gate::Dspi2), (1, 14));
        assert_eq!(bank_slot(Gate::Dspi3), (1, 15));
    }

    #[test]
    fn uart_gates_split_across_the_two_banks() {
        assert_eq!(bank_slot(Gate::Edma), (0, 17));
        assert_eq!(bank_slot(Gate::Uart0), (0, 24));
        assert_eq!(bank_slot(Gate::Uart3), (0, 27));
        assert_eq!(bank_slot(Gate::Uart4), (1, 24));
        assert_eq!(bank_slot(Gate::Uart8), (1, 28));
        assert_eq!(bank_slot(Gate::Uart9), (1, 29));
        assert_eq!(bank_slot(Gate::Pit0), (0, 32));
    }

    #[test]
    fn from_pll_25mhz_crystal_250mhz_core() {
        // FBKDIV 19: fVCO = 25 MHz × 20; OUTDIV1 1, OUTDIV2 3: core fVCO/2, bus fVCO/4 (RM 8.3.2's limits exactly)
        let c = Clocks::from_pll(25_000_000, Cr(0x0013), Dr(0x0000_0061));
        assert_eq!(c, Clocks { vco_hz: 500_000_000, core_hz: 250_000_000, bus_hz: 125_000_000 });
    }

    #[test]
    fn from_pll_refdiv_one_halves_the_reference() {
        assert_eq!(Clocks::from_pll(25_000_000, Cr(0x0113), Dr(0x0000_0061)).vco_hz, 250_000_000);
    }

    #[test]
    fn from_pll_reserved_refdiv_divides_by_one() {
        assert_eq!(Clocks::from_pll(25_000_000, Cr(0x0213), Dr(0x0000_0061)).vco_hz, 500_000_000);
    }

    #[test]
    fn from_pll_other_output_dividers() {
        // OUTDIV1 3, OUTDIV2 7: core fVCO/4, bus fVCO/8
        let c = Clocks::from_pll(25_000_000, Cr(0x0013), Dr(0x0000_00E3));
        assert_eq!((c.vco_hz, c.core_hz, c.bus_hz), (500_000_000, 125_000_000, 62_500_000));
    }

    #[test]
    fn pit_reload_1khz_at_125mhz() {
        assert_eq!(pit_reload(125_000_000, 1000), (1, 62_499)); // (62499 + 1) << 1 = 125000
    }

    #[test]
    fn pit_reload_small_counts_need_no_prescaler() {
        assert_eq!(pit_reload(1_000_000, 1000), (0, 999));
    }

    #[test]
    #[should_panic(expected = "PIT can't divide")]
    fn pit_reload_refuses_an_inexact_tick() {
        pit_reload(132_000_001, 1000);
    }
}
