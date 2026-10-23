//! The edge port's async waits on IRQ2 (PC2), IRQ6 (PC5) and IRQ7 (PC6), all pulled up, against the host's edges
//! at fixed times of the emulated clock (GETTIMEOFDAY, µs since the start): IRQ2 is low from the start and rises at
//! 10 ms; IRQ6 falls at 20 ms, rises at 30 ms, falls at 60 ms and rises at 70 ms; IRQ2 falls at 40 ms; IRQ7 falls at
//! 80 ms. Each wait must wake within 1 ms of its edge. A wait dropped by `select` leaves its pin disarmed (EPIER),
//! and a later wait on that pin still works. Reading a level leaves the pin's EPPAR field and EPIER bit as they were.
#![no_std]
#![no_main]

use coldfire_rt::entry;
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_mcf5441x::bind_interrupts;
use embassy_mcf5441x::eport::{Config, Input, InterruptHandler};
use embassy_mcf5441x::executor::Executor;
use embassy_mcf5441x::gpio::Pull;
use embassy_mcf5441x::pac;
use embassy_time::{Instant, Timer};
use embedded_hal_async::digital::Wait;
use mcf5441x_hal_tests::{check, gettimeofday, pass, say};
use static_cell::StaticCell;

const REF_HZ: u32 = 24_000_000;

bind_interrupts!(struct Irqs {
    EPORT_EPF2 => InterruptHandler;
    EPORT_EPF6 => InterruptHandler;
    EPORT_EPF7 => InterruptHandler;
});

/// The wait just ended within 1 ms after the host's edge at `ms`.
fn woke(step: u32, ms: u64) {
    let us = gettimeofday();
    say!("step {} woke at {} us", step, us);
    check!(us >= ms * 1000 && us < ms * 1000 + 1000, "step {}: woke at {} us, the edge was at {} ms", step, us, ms);
}

#[embassy_executor::task]
async fn main_task(mut irq2: Input<'static>, mut irq6: Input<'static>, mut irq7: Input<'static>) {
    say!("step 1");
    let _ = irq2.wait_for_low().await;
    let us = gettimeofday();
    check!(us < 10_000, "step 1: the pin was low already, but the wait took until {} us", us);

    say!("step 2");
    let _ = irq2.wait_for_rising_edge().await;
    woke(2, 10);

    say!("step 3");
    let _ = irq6.wait_for_falling_edge().await;
    woke(3, 20);

    say!("step 4");
    let _ = irq6.wait_for_high().await;
    woke(4, 30);

    say!("step 5");
    let _ = irq2.wait_for_any_edge().await;
    woke(5, 40);

    say!("step 6");
    let t0 = gettimeofday();
    let r = select(irq6.wait_for_rising_edge(), Timer::after_millis(2)).await;
    check!(matches!(r, Either::Second(())), "step 6: the timer should win");
    let us = gettimeofday() - t0;
    check!((2_000..3_000).contains(&us), "step 6: the timer took {} us", us);
    check!(pac::EPORT.epier().read().0 & 1 << 6 == 0, "step 6: the dropped wait left IRQ6 armed");

    say!("step 7");
    let _ = irq6.wait_for_falling_edge().await;
    woke(7, 60);

    say!("step 8");
    check!(irq2.is_low() && irq6.is_low(), "step 8: both pins are low");
    Timer::at(Instant::from_millis(75)).await;
    check!(irq6.is_high(), "step 8: IRQ6 rose at 70 ms");
    check!(irq7.is_high(), "step 8: IRQ7 is high");
    // IRQ2's last wait was both edges, IRQ6's the falling edge
    let (eppar, epier) = (pac::EPORT.eppar().read(), pac::EPORT.epier().read().0);
    check!(eppar == 0b11 << 4 | 0b10 << 12 && epier == 0, "step 8: the reads left EPPAR {:#x} EPIER {:#x}", eppar, epier);

    say!("step 9");
    let _ = irq7.wait_for_low().await;
    woke(9, 80);
    pass()
}

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[entry]
fn main() -> ! {
    let p = embassy_mcf5441x::init(embassy_mcf5441x::Config::new(REF_HZ));
    let irq2 = Input::new(p.PC2, Pull::Up, Irqs, Config::default());
    let irq6 = Input::new(p.PC5, Pull::Up, Irqs, Config::default());
    let irq7 = Input::new(p.PC6, Pull::Up, Irqs, Config::default());
    // SAFETY: start-up done; the edge port's and the time driver's interrupts are ready
    unsafe { coldfire::interrupt::enable() };
    EXECUTOR.init(Executor::new()).run(|spawner: Spawner| {
        spawner.spawn(main_task(irq2, irq6, irq7).unwrap());
    })
}
