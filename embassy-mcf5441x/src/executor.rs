//! Executors: thread mode (sleeps in `stop #0x2000` when idle) and interrupt mode (a [`SoftIrq`]), on
//! `embassy_executor::raw` with this crate's pender.
#[cfg(feature = "executor-interrupt")]
use crate::interrupt::{InterruptExt, SoftIrq};

/// The pender `embassy_executor::raw` calls when a task is woken.
#[unsafe(export_name = "__pender")]
fn pender(context: *mut ()) {
    // `context` is either null (thread mode executor, created by `Executor::new`) or a pointer to the
    // `SoftIrq` of an `InterruptExecutor` (created by `InterruptExecutor::start`).
    #[cfg(feature = "executor-thread")]
    if !cfg!(feature = "executor-interrupt") || context.is_null() {
        // Relaxed is enough: interrupts are masked when the executor reads this variable.
        thread::SIGNAL_WORK_THREAD_MODE.store(true, core::sync::atomic::Ordering::Relaxed);
        return;
    }
    #[cfg(feature = "executor-interrupt")]
    {
        // Safety: `context` points to the `SoftIrq` inside a `&'static InterruptExecutor`, written in
        // `start()` before the `raw::Executor` is created.
        let irq = unsafe { &*(context as *const SoftIrq) };
        irq.pend();
    }
    #[cfg(not(feature = "executor-interrupt"))]
    let _ = context;
}

#[cfg(feature = "executor-thread")]
pub use thread::*;
#[cfg(feature = "executor-thread")]
mod thread {
    use core::marker::PhantomData;
    use core::sync::atomic::{AtomicBool, Ordering};

    use embassy_executor::{Spawner, raw};

    /// Set by the pender when a task is woken, cleared by the executor before it sleeps.
    pub(super) static SIGNAL_WORK_THREAD_MODE: AtomicBool = AtomicBool::new(false);

    /// Thread mode executor, using `stop`.
    ///
    /// It runs in supervisor mode at the lowest interrupt level and sleeps with `stop #0x2000` when it has no
    /// more work. Any interrupt wakes it; when a task was woken, the next idle check sees it and polls instead
    /// of sleeping. While asleep the interrupt level is 0, whatever it was before.
    pub struct Executor {
        inner: raw::Executor,
        not_send: PhantomData<*mut ()>,
    }

    impl Executor {
        pub fn new() -> Self {
            Self { inner: raw::Executor::new(core::ptr::null_mut()), not_send: PhantomData }
        }

        /// Run the executor: `init` spawns the first tasks, then this polls forever. Needs `&'static mut self`
        /// (a `StaticCell`, for example) and supervisor mode (`stop` is privileged).
        pub fn run(&'static mut self, init: impl FnOnce(Spawner)) -> ! {
            init(self.inner.spawner());
            loop {
                unsafe {
                    self.inner.poll();
                    critical_section::with(|_| {
                        if SIGNAL_WORK_THREAD_MODE.load(Ordering::Relaxed) {
                            SIGNAL_WORK_THREAD_MODE.store(false, Ordering::Relaxed);
                        } else {
                            // `stop` loads SR (IPL 0) and halts in one instruction, so a wakeup that arrives
                            // after the flag check is still taken. The interrupt's `rte` resumes here, and the
                            // critical section's release restores the original SR.
                            core::arch::asm!("stop #0x2000", options(nostack));
                        }
                    });
                }
            }
        }
    }
}

#[cfg(feature = "executor-interrupt")]
pub use interrupt::*;
#[cfg(feature = "executor-interrupt")]
mod interrupt {
    use core::cell::{Cell, UnsafeCell};
    use core::mem::MaybeUninit;

    use critical_section::Mutex;
    use embassy_executor::{SendSpawner, raw};

    use crate::interrupt::{InterruptExt, SoftIrq};

    struct Inner {
        irq: SoftIrq,
        executor: raw::Executor,
    }

    /// Interrupt mode executor: polls its tasks in the handler of a [`SoftIrq`], which a woken task pends.
    ///
    /// Tasks at a higher interrupt level preempt tasks at a lower one, including the thread-mode
    /// [`Executor`](super::Executor). All levels share the supervisor stack. Critical sections mask levels 1–6
    /// (SR = 0x2700), so long ones delay these tasks too.
    pub struct InterruptExecutor {
        started: Mutex<Cell<bool>>,
        inner: UnsafeCell<MaybeUninit<Inner>>,
    }

    unsafe impl Send for InterruptExecutor {}
    unsafe impl Sync for InterruptExecutor {}

    impl InterruptExecutor {
        #[inline]
        pub const fn new() -> Self {
            Self { started: Mutex::new(Cell::new(false)), inner: UnsafeCell::new(MaybeUninit::uninit()) }
        }

        /// The interrupt's handler: clears the software pend, then polls. A task woken during the poll pends it
        /// again, so the interrupt is taken once more after this returns.
        ///
        /// # Safety
        /// Call only from the `SoftIrq`'s handler, and only after `start()`.
        pub unsafe fn on_interrupt(&'static self) {
            let inner = unsafe { (&*self.inner.get()).assume_init_ref() };
            inner.irq.unpend();
            unsafe { inner.executor.poll() };
        }

        /// Start the executor and unmask its source; it then runs in the background through the interrupt.
        ///
        /// Before calling: install the handler at `irq.vector()` (calling [`on_interrupt`](Self::on_interrupt)
        /// with d0-d1/a0-a1 saved, as `exception_entry!` does) and set the source's level to 1..=6 with
        /// [`InterruptExt::set_priority`]. Don't change it afterwards, and don't unmask it yourself.
        ///
        /// # Panics
        /// If the source's level isn't 1..=6, or the executor was already started.
        pub fn start(&'static self, irq: SoftIrq) -> SendSpawner {
            assert!(matches!(irq.priority(), 1..=6), "interrupt source level must be in the range 1..=6");
            if critical_section::with(|cs| self.started.borrow(cs).replace(true)) {
                panic!("InterruptExecutor::start() called multiple times on the same executor.");
            }
            let inner = unsafe {
                let inner = (*self.inner.get()).as_mut_ptr();
                // The source first: the executor's pender context points to it.
                (&raw mut (*inner).irq).write(irq);
                let irq = &raw const (*inner).irq;
                (&raw mut (*inner).executor).write(raw::Executor::new(irq as *mut ()));
                &*inner
            };
            unsafe { inner.irq.enable() };
            inner.executor.spawner().make_send()
        }

        /// A spawner for this executor. Panics if it hasn't been started.
        pub fn spawner(&'static self) -> SendSpawner {
            if !critical_section::with(|cs| self.started.borrow(cs).get()) {
                panic!("InterruptExecutor::spawner() called on uninitialized executor.");
            }
            let inner = unsafe { (&*self.inner.get()).assume_init_ref() };
            inner.executor.spawner().make_send()
        }
    }
}
