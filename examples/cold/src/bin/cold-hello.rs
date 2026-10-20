//! Boots from reset with no bootloader: the boot part sets up the PLL and DDR from `BOOT`, copies the app part into
//! DDR and runs `main`. Build it with one boot feature and that source's memory.x (memory/<source>/memory.x).
#![no_std]
#![no_main]

use coldfire_rt::entry;
use embassy_mcf5441x::boot::*;

/// 25 MHz × 20 = a 500 MHz VCO; core 250 MHz, bus 125 MHz, NFC 62.5 MHz: the reference manual's limits exactly.
const PLL: PllSettings = PllSettings { refdiv: 0, fbkdiv: 19, outdiv1: 1, outdiv2: 3, outdiv5: 7 };

/// Your DRAM's controller words go here (RM 21.4), worked out from its datasheet at your SD_CLK; this example leaves
/// the controller at its reset values. PADCR: bit 9 set, SPARE_DLY_CTRL 01.
const DDR: Ddr = Ddr { cr: &[], padcr: 1 << 9 | 0b01, full_strength: false };

/// Where the app part is. Serial: the SPI flash holds the 7-byte SBF header, then the image from flash byte 7; DSPI0
/// reads it in 8-bit frames at bus / 2 / 4 (CTAR FMSZ 7, BR 1).
#[cfg(feature = "boot-serial")]
const APP: AppSource = AppSource::SpiFlash { offset: 7, ctar: 7 << 27 | 1 };
/// NOR: the flash at address 0 holds the image.
#[cfg(feature = "boot-nor")]
const APP: AppSource = AppSource::Mapped(0);
/// NAND: the pages after the four boot pages.
#[cfg(feature = "boot-nand")]
const APP: AppSource = AppSource::Nand;

/// The NOR flash (boot-nor): 16 MB, 16-bit port, auto-acknowledge after 4 wait states (CSCR0 WS 4, AA, PS 10).
const CS0: FlexBusCs0 = FlexBusCs0 { cscr: 4 << 10 | 1 << 8 | 0b10 << 6, size: 16 << 20 };

/// Program the PLL, or (feature `pll-keep`) check the one a serial-boot header's SBF_RCON already set.
#[cfg(not(feature = "pll-keep"))]
const MODE: Pll = Pll::Set(PLL);
#[cfg(feature = "pll-keep")]
const MODE: Pll = Pll::Keep(PLL);

#[boot_config]
static BOOT: BootConfig = BootConfig::builder(RefClock::Crystal(25_000_000), MODE, DDR).app(APP).cs0(CS0).build();

#[entry]
fn main() -> ! {
    let _p = embassy_mcf5441x::init(embassy_mcf5441x::Config::new(25_000_000));
    loop {
        coldfire::asm::nop();
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        coldfire::asm::stop();
    }
}
