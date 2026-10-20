//! SSI0 as an I2S master, 16-bit stereo: each block received is sent straight back out (a line-level loopback through
//! your codec). Not run on hardware: no board here has SSI0 wired. Adapt the pins, the clocks and the codec setup.
#![no_std]
#![no_main]

use coldfire_rt::entry;
use embassy_mcf5441x::bind_interrupts;
use embassy_mcf5441x::interrupt::{Priority, SoftIrq};
use embassy_mcf5441x::ssi::{Block, ClockSource, Config, Rings, Ssi, TxDmaInterruptHandler, WordLength};

const REF_HZ: u32 = 24_000_000; // the PLL reference: change for your board's crystal

bind_interrupts!(struct Irqs { EDMA_INTR_INT50 => TxDmaInterruptHandler<embassy_mcf5441x::peripherals::SSI0>; });

/// 32 frames of 2 slots: the DMA rings for both directions.
static mut RINGS: Rings<32, 2> = Rings::new();

/// INTC0 source 44 is unused on the MCF5441x (RM Table 17-15): the render interrupt.
const RENDER: SoftIrq = SoftIrq::new(0, 44);

fn render(input: &Block<32, 2>, output: &mut Block<32, 2>) {
    output.copy_from_slice(input);
}

#[entry]
fn main() -> ! {
    let p = embassy_mcf5441x::init(embassy_mcf5441x::Config::new(REF_HZ));
    // the bit clock from the PLL: pick `div` and the CCR dividers for your sample rate
    let config = Config { clock: ClockSource::Pll { div: 8 }, ..Config::i2s_master(WordLength::W16, 2) };
    let ssi = Ssi::new(p.SSI0, p.PH3, p.PH5, p.PH6, p.PH7, config);
    // SAFETY: the rings are handed to the stream once, for good
    let rings = unsafe { &mut *(&raw mut RINGS) };
    let _stream = ssi.stream(p.DMA_CH50, p.DMA_CH48, Irqs, rings, Priority::P6, RENDER, Priority::P5, render);
    // SAFETY: start-up done; the stream's interrupts are ready
    unsafe { coldfire::interrupt::enable() };
    loop {
        coldfire::asm::stop();
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        coldfire::asm::stop();
    }
}
