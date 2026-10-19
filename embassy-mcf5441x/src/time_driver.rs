//! embassy-time driver on one PIT (feature `time-driver-pitN`): a periodic interrupt at embassy-time-driver's
//! tick rate (1 kHz) counts ticks and wakes due timers.
use core::cell::{Cell, RefCell};
use core::task::Waker;

use critical_section::Mutex;
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;

use crate::clocks::{self, Gate};
use crate::interrupt::{Interrupt, InterruptExt, Priority};
use crate::pac;
use crate::pac::pit0::vals::{En, Pie};

#[cfg(feature = "time-driver-pit0")]
const PIT: (pac::pit0::Pit0, Interrupt, Gate) = (pac::PIT0, Interrupt::PIT0, Gate::Pit0);
#[cfg(feature = "time-driver-pit1")]
const PIT: (pac::pit0::Pit0, Interrupt, Gate) = (pac::PIT1, Interrupt::PIT1, Gate::Pit1);
#[cfg(feature = "time-driver-pit2")]
const PIT: (pac::pit0::Pit0, Interrupt, Gate) = (pac::PIT2, Interrupt::PIT2, Gate::Pit2);
#[cfg(feature = "time-driver-pit3")]
const PIT: (pac::pit0::Pit0, Interrupt, Gate) = (pac::PIT3, Interrupt::PIT3, Gate::Pit3);

struct PitDriver {
    ticks: Mutex<Cell<u64>>,
    queue: Mutex<RefCell<Queue>>,
}

embassy_time_driver::time_driver_impl!(static DRIVER: PitDriver = PitDriver {
    ticks: Mutex::new(Cell::new(0)),
    queue: Mutex::new(RefCell::new(Queue::new())),
});

impl Driver for PitDriver {
    fn now(&self) -> u64 {
        critical_section::with(|cs| self.ticks.borrow(cs).get())
    }

    fn schedule_wake(&self, at: u64, waker: &Waker) {
        critical_section::with(|cs| {
            let mut q = self.queue.borrow(cs).borrow_mut();
            if q.schedule_wake(at, waker) {
                // A timer already due is woken now, not a tick later.
                q.next_expiration(self.ticks.borrow(cs).get());
            }
        });
    }
}

impl PitDriver {
    fn tick(&self) {
        critical_section::with(|cs| {
            let t = self.ticks.borrow(cs).get() + 1;
            self.ticks.borrow(cs).set(t);
            self.queue.borrow(cs).borrow_mut().next_expiration(t);
        });
    }
}

fn on_interrupt() {
    PIT.0.pcsr().write_value(PIT.0.pcsr().read()); // PIF is write-1-to-clear
    DRIVER.tick();
}

#[cfg(feature = "time-driver-pit0")]
#[coldfire_rt::interrupt(vectors = crate::pac::Interrupt)]
fn PIT0() {
    on_interrupt()
}
#[cfg(feature = "time-driver-pit1")]
#[coldfire_rt::interrupt(vectors = crate::pac::Interrupt)]
fn PIT1() {
    on_interrupt()
}
#[cfg(feature = "time-driver-pit2")]
#[coldfire_rt::interrupt(vectors = crate::pac::Interrupt)]
fn PIT2() {
    on_interrupt()
}
#[cfg(feature = "time-driver-pit3")]
#[coldfire_rt::interrupt(vectors = crate::pac::Interrupt)]
fn PIT3() {
    on_interrupt()
}

/// Start the tick: (PMR + 1) << PRE bus cycles per tick (132 MHz, 1 kHz: PRE 2, PMR 32999), level 6.
pub(crate) fn init(bus_hz: u32) {
    let (pit, irq, gate) = PIT;
    let (pre, pm) = clocks::pit_reload(bus_hz, embassy_time_driver::TICK_HZ as u32);
    clocks::ungate(gate);
    irq.set_priority(Priority::P6);
    // SAFETY: the handler only touches this PIT and the driver, both ready before the PIT is enabled below.
    unsafe { irq.enable() };
    let cfg = |en| {
        let mut w = pac::pit0::regs::Pcsr(0);
        w.set_pre(pre);
        w.set_ovw(true);
        w.set_pie(Pie::Enabled);
        w.set_pif(true);
        w.set_rld(true);
        w.set_en(en);
        w
    };
    pit.pcsr().write_value(cfg(En::Disabled));
    pit.pmr().write(|w| w.set_pm(pm));
    pit.pcsr().write_value(cfg(En::Enabled));
}
