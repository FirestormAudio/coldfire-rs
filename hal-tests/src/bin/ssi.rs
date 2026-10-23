//! SSI0 (BCLK PH3, FS PH5, TXD PH6, RXD PH7) as a 16-bit stereo I2S master streaming both ways: the render sends
//! each received sample back plus 1. After 20 blocks none was late or skipped; the stream stops and the program
//! passes. The host checks that what was sent is what it supplied plus 1, a fixed number of frames later.
#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};

use coldfire_rt::entry;
use embassy_mcf5441x::bind_interrupts;
use embassy_mcf5441x::interrupt::{Priority, SoftIrq};
use embassy_mcf5441x::ssi::{Block, ClockSource, Config, Rings, Ssi, TxDmaInterruptHandler, WordLength};
use mcf5441x_hal_tests::{check_eq, pass, say};

const REF_HZ: u32 = 24_000_000;

bind_interrupts!(struct Irqs { EDMA_INTR_INT50 => TxDmaInterruptHandler<embassy_mcf5441x::peripherals::SSI0>; });

static mut RINGS: Rings<64, 2> = Rings::new();

/// INTC0 source 44 is unused on the MCF5441x (RM Table 17-15): the render interrupt.
const RENDER: SoftIrq = SoftIrq::new(0, 44);
const BLOCKS: u32 = 20;

static RENDERED: AtomicU32 = AtomicU32::new(0);

fn render(input: &Block<64, 2>, output: &mut Block<64, 2>) {
    for (o, i) in output.iter_mut().zip(input) {
        for (o, &i) in o.iter_mut().zip(i) {
            *o = i.wrapping_add(1);
        }
    }
    RENDERED.fetch_add(1, Ordering::Relaxed);
}

#[entry]
fn main() -> ! {
    let p = embassy_mcf5441x::init(embassy_mcf5441x::Config::new(REF_HZ));
    let config = Config::i2s_master(WordLength::W16, ClockSource::Pll { div: 8 }, 1);
    let ssi = Ssi::new(p.SSI0, p.PH3, p.PH5, p.PH6, p.PH7, config);
    // SAFETY: the rings are handed to the stream once, for good
    let rings = unsafe { &mut *(&raw mut RINGS) };
    let stream = ssi.stream(p.DMA_CH50, p.DMA_CH48, Irqs, rings, Priority::P4, RENDER, Priority::P3, render);
    // SAFETY: start-up done; the stream's interrupts are ready
    unsafe { coldfire::interrupt::enable() };
    say!("streaming");
    while RENDERED.load(Ordering::Relaxed) < BLOCKS {
        coldfire::asm::stop();
    }
    let stats = stream.stats();
    check_eq!(stats.late, 0);
    check_eq!(stats.skipped, 0);
    let _ = stream.stop();
    pass()
}
