//! Status and control registers. Control registers are write-only through `movec`.
use core::arch::asm;

pub mod sr {
    use super::asm;
    #[inline(always)]
    pub fn read() -> u16 {
        let v: u32;
        unsafe { asm!("move.w %sr,{0}", out(reg_data) v, options(nostack)) };
        v as u16
    }
    /// # Safety
    /// Lowering the IPL inside a critical section lets interrupts in while its token is live; clearing
    /// S leaves supervisor mode (the next privileged instruction traps).
    #[inline(always)]
    pub unsafe fn write(v: u16) {
        unsafe { asm!("move.w {0},%sr", in(reg_data) v as u32, options(nostack)) };
    }
}

pub mod vbr {
    use super::asm;
    /// # Safety
    /// `v` must point at a valid vector table on a 1 MB boundary (VBR bits 19..0 are not implemented,
    /// RM 3.2.8).
    #[inline(always)]
    pub unsafe fn write(v: u32) {
        unsafe { asm!("movec {0},%vbr", in(reg_data) v, options(nostack)) };
    }
}

pub mod cacr {
    use super::asm;
    /// # Safety
    /// Changes cache behaviour for all code and data.
    #[inline(always)]
    pub unsafe fn write(v: u32) {
        unsafe { asm!("movec {0},%cacr", in(reg_data) v, options(nostack)) };
    }
}

macro_rules! acr {
    ($m:ident, $reg:literal) => {
        pub mod $m {
            use super::asm;
            /// # Safety
            /// Changes the cacheability of an address range.
            #[inline(always)]
            pub unsafe fn write(v: u32) {
                unsafe { asm!(concat!("movec {0},", $reg), in(reg_data) v, options(nostack)) };
            }
        }
    };
}
acr!(acr0, "%acr0");
acr!(acr1, "%acr1");
acr!(acr2, "%acr2");
acr!(acr3, "%acr3");
