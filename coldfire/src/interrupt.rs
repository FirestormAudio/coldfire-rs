//! Interrupt masking through SR's IPL.
use core::arch::asm;

/// Allow all interrupt levels (SR = 0x2000: supervisor, IPL 0).
/// # Safety
/// Not inside a critical section: interrupts would run while its token is live.
#[inline(always)]
pub unsafe fn enable() {
    unsafe { asm!("move.w #0x2000,%sr", options(nostack)) };
}

/// Mask all levels (IPL 7); returns the previous SR for `restore`.
#[inline(always)]
pub fn disable() -> u16 {
    let sr: u32;
    unsafe { asm!("move.w %sr,{0}", "move.w #0x2700,%sr", out(reg_data) sr, options(nostack)) };
    sr as u16
}

/// # Safety
/// `sr` must come from `disable` (or be no less masked than the caller's critical section needs).
#[inline(always)]
pub unsafe fn restore(sr: u16) {
    unsafe { asm!("move.w {0},%sr", in(reg_data) sr as u32, options(nostack)) };
}

/// Run `f` with interrupts masked.
#[inline(always)]
pub fn free<R>(f: impl FnOnce() -> R) -> R {
    let sr = disable();
    let r = f();
    unsafe { restore(sr) }; // the SR `disable` returned
    r
}

/// Sleep in `stop` until `cond()` holds, with no lost-wakeup window: `cond` runs with interrupts masked,
/// and `stop #0x2000` unmasks and halts in one instruction, so an interrupt that makes `cond` true
/// can't slip in between the check and the stop. The previous SR is restored on return. While it
/// sleeps the IPL is 0 whatever the caller's was: a caller inside a critical section is unmasked
/// while it waits.
/// # Safety
/// Not inside a critical section: it unmasks (IPL 0) while it sleeps.
pub unsafe fn wait_until(mut cond: impl FnMut() -> bool) {
    let sr: u32;
    unsafe { asm!("move.w %sr,{0}", out(reg_data) sr, options(nostack)) };
    loop {
        unsafe { asm!("move.w #0x2700,%sr", options(nostack)) };
        if cond() {
            break;
        }
        unsafe { asm!("stop #0x2000", options(nostack)) }; // the interrupt's rte resumes here
    }
    unsafe { asm!("move.w {0},%sr", in(reg_data) sr, options(nostack)) };
}
