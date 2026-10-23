//! embassy-time on PIT0 (the HAL's time driver): `Timer::after` for 1, 10 and 250 ms and a 5 ms `Ticker`, each
//! measured with `Instant` and against the host's clock (GETTIMEOFDAY: the emulated clock), on the thread executor.
#![no_std]
#![no_main]

use coldfire_rt::entry;
use embassy_executor::Spawner;
use embassy_mcf5441x::executor::Executor;
use embassy_time::{Duration, Instant, Ticker, Timer};
use mcf5441x_hal_tests::{check, gettimeofday, pass, say};
use static_cell::StaticCell;

const REF_HZ: u32 = 24_000_000;

/// The wait between `t0` (an `Instant`) and `g0` (the host's clock, µs) and now, checked to be `ms` to `ms + 1`
/// milliseconds by `Instant` and within 1 ms of that by the host's clock.
fn check_wait(what: &str, ms: u64, t0: Instant, g0: u64) {
    let took = Instant::now().duration_since(t0).as_micros();
    let host = gettimeofday() - g0;
    say!("{}: {} us, host {} us", what, took, host);
    check!(took >= ms * 1000 && took <= (ms + 1) * 1000, "{} took {} us", what, took);
    check!(host.abs_diff(took) <= 1000, "{}: the host's clock says {} us", what, host);
}

#[embassy_executor::task]
async fn main_task() {
    for ms in [1, 10, 250] {
        let (t0, g0) = (Instant::now(), gettimeofday());
        Timer::after_millis(ms).await;
        check_wait("timer", ms, t0, g0);
    }
    let mut ticker = Ticker::every(Duration::from_millis(5));
    let (t0, g0) = (Instant::now(), gettimeofday());
    for _ in 0..10 {
        ticker.next().await;
    }
    check_wait("ticker", 50, t0, g0);
    pass()
}

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[entry]
fn main() -> ! {
    let _p = embassy_mcf5441x::init(embassy_mcf5441x::Config::new(REF_HZ));
    // SAFETY: start-up done; the time driver's interrupt is ready
    unsafe { coldfire::interrupt::enable() };
    EXECUTOR.init(Executor::new()).run(|spawner: Spawner| {
        spawner.spawn(main_task().unwrap());
    })
}
