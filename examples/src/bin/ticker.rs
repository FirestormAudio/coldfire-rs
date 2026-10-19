//! The thread executor and embassy-time: a line on UART0 (TX PF3, 115,200 baud) every second.
#![no_std]
#![no_main]

use core::fmt::Write;

use coldfire_rt::entry;
use embassy_executor::Spawner;
use embassy_mcf5441x::executor::Executor;
use embassy_mcf5441x::uart::{Config, UartTx};
use embassy_time::{Instant, Timer};
use static_cell::StaticCell;

const REF_HZ: u32 = 24_000_000; // the PLL reference: change for your board's crystal

#[embassy_executor::task]
async fn tick(mut tx: UartTx<'static>) {
    for n in 1.. {
        Timer::after_secs(1).await;
        let _ = writeln!(tx, "tick {} at {} ms\r", n, Instant::now().as_millis());
    }
}

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[entry]
fn main() -> ! {
    let p = embassy_mcf5441x::init(embassy_mcf5441x::Config::new(REF_HZ));
    let tx = UartTx::new(p.UART0, p.PF3, Config::new(115_200)).expect("115,200 baud");
    // SAFETY: start-up, before anything shares state with an interrupt; the time driver's PIT interrupt is ready
    unsafe { coldfire::interrupt::enable() };
    EXECUTOR.init(Executor::new()).run(|spawner: Spawner| {
        spawner.spawn(tick(tx).unwrap());
    })
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        coldfire::asm::stop();
    }
}
