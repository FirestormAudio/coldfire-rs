//! The DMA timers as timestamp counters, against the host's clock (GETTIMEOFDAY: the emulated clock): DTIM0 at the
//! bus clock, DTIM1 with a prescaler of 60 and DTIM2 at the bus clock / 16 over a 1 ms busy wait, then DTIM0 across an
//! 80 s sleep, which wraps its 32-bit count.
#![no_std]
#![no_main]

use coldfire_rt::entry;
use embassy_executor::Spawner;
use embassy_mcf5441x::dtim::{Clock, Config, Timestamp};
use embassy_mcf5441x::executor::Executor;
use embassy_time::Timer;
use mcf5441x_hal_tests::{check, check_eq, gettimeofday, pass, say};
use static_cell::StaticCell;

const REF_HZ: u32 = 24_000_000;

/// The counts `t` should show over `us` µs.
fn expected(t: &Timestamp, us: u64) -> u64 {
    us * u64::from(t.hz()) / 1_000_000
}

/// Over a busy wait of 1 ms by the host's clock, each counter's `elapsed` matches it within 0.5 %.
fn busy_ms(timers: &[&Timestamp]) {
    let starts: [u32; 3] = core::array::from_fn(|i| timers.get(i).map_or(0, |t| t.now()));
    let g0 = gettimeofday();
    let mut us = 0;
    while us < 1000 {
        us = gettimeofday() - g0;
    }
    // every count read before any line is printed: printing takes time
    let ends: [u32; 3] = core::array::from_fn(|i| timers.get(i).map_or(0, |t| t.elapsed(starts[i])));
    for (t, &e) in timers.iter().zip(&ends) {
        let got = u64::from(e);
        let want = expected(t, us);
        say!("{} Hz: {} counts in {} us (want {})", t.hz(), got, us, want);
        check!(got.abs_diff(want) * 200 <= want, "{} counts, want {}", got, want);
    }
}

#[embassy_executor::task]
async fn main_task(t0: Timestamp<'static>) {
    say!("wrap");
    let (s, g0) = (t0.now(), gettimeofday());
    Timer::after_secs(80).await;
    let got = t0.elapsed(s);
    let us = gettimeofday() - g0;
    let want = (expected(&t0, us) % (1u64 << 32)) as u32;
    say!("{} counts across {} us (want {})", got, us, want);
    check!(us > 71_600_000, "the sleep should outlast one wrap");
    check!(got.abs_diff(want) <= t0.hz() / 1000, "{} counts, want {}", got, want);
    pass()
}

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[entry]
fn main() -> ! {
    let p = embassy_mcf5441x::init(embassy_mcf5441x::Config::new(REF_HZ));
    let bus = embassy_mcf5441x::clocks::clocks().bus_hz;
    let t0 = Timestamp::new(p.DTIM0, Config::default());
    let t1 = Timestamp::new(p.DTIM1, Config::new(Clock::Bus, 60));
    let t2 = Timestamp::new(p.DTIM2, Config::new(Clock::BusDiv16, 1));
    check_eq!(t0.hz(), bus);
    check_eq!(t1.hz(), bus / 60);
    check_eq!(t2.hz(), bus / 16);
    say!("busy wait");
    busy_ms(&[&t0, &t1, &t2]);
    // SAFETY: start-up done; the time driver's interrupt is ready
    unsafe { coldfire::interrupt::enable() };
    EXECUTOR.init(Executor::new()).run(|spawner: Spawner| {
        spawner.spawn(main_task(t0).unwrap());
    })
}
