//! Single instructions.
use core::arch::asm;

/// Sleep until an interrupt: `stop #0x2000` (supervisor, IPL 0 while stopped).
#[inline(always)]
pub fn stop() {
    unsafe { asm!("stop #0x2000", options(nostack)) };
}

/// Halt the core (visible to a BDM debugger); resumes after `halt` when the debugger lets it go.
#[inline(always)]
pub fn halt() {
    unsafe { asm!("halt", options(nostack)) };
}

#[inline(always)]
pub fn nop() {
    unsafe { asm!("nop", options(nostack)) };
}
