//! The `critical-section` implementation for a single core: SR to IPL 7, restored on release.
use core::arch::asm;

struct Cs;
critical_section::set_impl!(Cs);

unsafe impl critical_section::Impl for Cs {
    unsafe fn acquire() -> critical_section::RawRestoreState {
        let sr: u32;
        unsafe { asm!("move.w %sr,{0}", "move.w #0x2700,%sr", out(reg_data) sr, options(nostack)) };
        sr as u16
    }
    unsafe fn release(sr: critical_section::RawRestoreState) {
        unsafe { asm!("move.w {0},%sr", in(reg_data) sr as u32, options(nostack)) };
    }
}
