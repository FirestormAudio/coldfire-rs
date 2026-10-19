//! Embassy HAL for the NXP MCF5441x (ColdFire V4e). It assumes a bootloader has set up the PLL, the DDR
//! controller and the boot pin mux: `init` reads the PLL to learn its clocks and never programs it.
#![no_std]
#![cfg_attr(target_arch = "m68k", feature(asm_experimental_arch))]

#[cfg(not(any(feature = "mcf54410", feature = "mcf54415", feature = "mcf54416", feature = "mcf54417", feature = "mcf54418")))]
compile_error!("embassy-mcf5441x: choose a part feature (mcf54410, mcf54415, mcf54416, mcf54417 or mcf54418)");

pub use embassy_hal_internal::{Peri, PeripheralType};
pub use mcf5441x_pac as pac;
#[cfg(target_arch = "m68k")]
pub use coldfire_rt as rt;

pub mod clocks;
pub mod dma;
pub mod esdhc;
#[cfg(all(target_arch = "m68k", any(feature = "executor-thread", feature = "executor-interrupt")))]
pub mod executor;
pub mod gpio;
pub mod pinmux;
pub mod spi;
pub mod uart;
#[cfg(any(feature = "mcf54415", feature = "mcf54416", feature = "mcf54417", feature = "mcf54418"))]
pub mod usb;
pub mod interrupt;
#[cfg(all(target_arch = "m68k", feature = "_time-driver"))]
mod time_driver;

/// The PAC register block behind a peripheral token.
pub trait Regs: PeripheralType {
    type Block: Copy;
    const REGS: Self::Block;
}

macro_rules! impl_regs {
    ($name:ident, $m:ident::$t:ident) => {
        impl crate::Regs for peripherals::$name {
            type Block = pac::$m::$t;
            const REGS: Self::Block = pac::$name;
        }
    };
}

use dma::impl_dma_channel;
use gpio::impl_pin;

include!("peripherals.rs");

#[cfg(any(
    all(feature = "time-driver-pit0", any(feature = "time-driver-pit1", feature = "time-driver-pit2", feature = "time-driver-pit3")),
    all(feature = "time-driver-pit1", any(feature = "time-driver-pit2", feature = "time-driver-pit3")),
    all(feature = "time-driver-pit2", feature = "time-driver-pit3"),
))]
compile_error!("embassy-mcf5441x: choose at most one time-driver-pitN feature");

/// HAL configuration.
#[non_exhaustive]
#[derive(Copy, Clone, Debug)]
pub struct Config {
    /// PLL reference (crystal or external clock), Hz, e.g. a 24 MHz crystal.
    pub ref_hz: u32,
}

impl Config {
    pub const fn new(ref_hz: u32) -> Self {
        Self { ref_hz }
    }
}

/// Take the peripherals, ungate the interrupt controllers, learn the clocks from the PLL and start the time
/// driver. The status register's interrupt mask is left as it is: the app enables interrupts when it's ready
/// (the thread executor's `stop #0x2000` also unmasks while idle). Panics if called twice.
pub fn init(config: Config) -> Peripherals {
    let p = Peripherals::take();
    for g in [clocks::Gate::Intc0, clocks::Gate::Intc1, clocks::Gate::Intc2] {
        clocks::ungate(g);
    }
    let c = clocks::init(config.ref_hz);
    #[cfg(all(target_arch = "m68k", feature = "_time-driver"))]
    time_driver::init(c.bus_hz);
    #[cfg(not(all(target_arch = "m68k", feature = "_time-driver")))]
    let _ = c;
    p
}
