//! Interrupt executors at levels 2 and 5 (INTC0 sources 60 and 61, which RM Table 17-15 leaves unused) and the
//! thread executor: a level-2 task logs `a`, wakes the level-5 task and spins until it has run; the level-5 task
//! preempts it, logs `B` and lets it go; the level-2 task logs `c`. The thread-mode task then finds `aBc`.
#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, AtomicU8, AtomicUsize, Ordering};

use coldfire_rt::{entry, interrupt};
use embassy_executor::Spawner;
use embassy_mcf5441x::executor::{Executor, InterruptExecutor};
use embassy_mcf5441x::interrupt::{InterruptExt, Priority, SoftIrq};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use mcf5441x_hal_tests::{check_eq, pass, say};
use static_cell::StaticCell;

const REF_HZ: u32 = 24_000_000;

const LOW: SoftIrq = SoftIrq::new(0, 60);
const HIGH: SoftIrq = SoftIrq::new(0, 61);

/// The vectors of the two sources, for `#[interrupt]`.
mod vectors {
    pub const SWI_LOW: u16 = 64 + 60;
    pub const SWI_HIGH: u16 = 64 + 61;
}

static EXEC_LOW: InterruptExecutor = InterruptExecutor::new();
static EXEC_HIGH: InterruptExecutor = InterruptExecutor::new();

#[interrupt(vectors = vectors)]
fn SWI_LOW() {
    // SAFETY: this is the source's handler, installed before `start`
    unsafe { EXEC_LOW.on_interrupt() }
}

#[interrupt(vectors = vectors)]
fn SWI_HIGH() {
    // SAFETY: as above
    unsafe { EXEC_HIGH.on_interrupt() }
}

static LOG: [AtomicU8; 8] = [const { AtomicU8::new(0) }; 8];
static LEN: AtomicUsize = AtomicUsize::new(0);

fn log(b: u8) {
    LOG[LEN.fetch_add(1, Ordering::AcqRel)].store(b, Ordering::Release);
}

static WAKE_HIGH: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static HIGH_RAN: AtomicBool = AtomicBool::new(false);
static DONE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

#[embassy_executor::task]
async fn high() {
    WAKE_HIGH.wait().await;
    log(b'B');
    HIGH_RAN.store(true, Ordering::Release);
}

#[embassy_executor::task]
async fn low() {
    log(b'a');
    WAKE_HIGH.signal(());
    // the level-5 executor's interrupt preempts this level-2 one
    while !HIGH_RAN.load(Ordering::Acquire) {}
    log(b'c');
    DONE.signal(());
}

#[embassy_executor::task]
async fn main_task() {
    DONE.wait().await;
    let n = LEN.load(Ordering::Acquire);
    let mut got = [0u8; 8];
    for (g, l) in got.iter_mut().zip(&LOG) {
        *g = l.load(Ordering::Acquire);
    }
    say!("log {:?}", core::str::from_utf8(&got[..n]));
    check_eq!(&got[..n], b"aBc");
    pass()
}

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[entry]
fn main() -> ! {
    let _p = embassy_mcf5441x::init(embassy_mcf5441x::Config::new(REF_HZ));
    LOW.set_priority(Priority::P2);
    HIGH.set_priority(Priority::P5);
    let high_spawner = EXEC_HIGH.start(HIGH);
    let low_spawner = EXEC_LOW.start(LOW);
    high_spawner.spawn(high().unwrap());
    low_spawner.spawn(low().unwrap());
    // SAFETY: start-up done; both executors' handlers are installed
    unsafe { coldfire::interrupt::enable() };
    EXECUTOR.init(Executor::new()).run(|spawner: Spawner| {
        spawner.spawn(main_task().unwrap());
    })
}
