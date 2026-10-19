//! Width-fixed memory-mapped register access.

#[inline(always)] pub unsafe fn r8(a: usize) -> u8 { unsafe { (a as *const u8).read_volatile() } }
#[inline(always)] pub unsafe fn w8(a: usize, v: u8) { unsafe { (a as *mut u8).write_volatile(v) } }
/// 16- and 32-bit reads are done in assembly so the access width is fixed: through rustc_codegen_gcc,
/// GCC narrowed a volatile `r32(a) & bit` into a byte `btst` on the register (a different access).
#[cfg(target_arch = "m68k")]
#[inline(always)] pub unsafe fn r16(a: usize) -> u16 {
    let v: u32;
    unsafe { core::arch::asm!("move.w ({0}),{1}", in(reg_addr) a, out(reg_data) v, options(nostack, readonly)) };
    v as u16
}
#[inline(always)] pub unsafe fn w16(a: usize, v: u16) { unsafe { (a as *mut u16).write_volatile(v) } }
#[cfg(target_arch = "m68k")]
#[inline(always)] pub unsafe fn r32(a: usize) -> u32 {
    let v: u32;
    unsafe { core::arch::asm!("move.l ({0}),{1}", in(reg_addr) a, out(reg_data) v, options(nostack, readonly)) };
    v
}
/// Host builds (unit tests of crates that use these helpers) have no ColdFire asm: plain volatile reads.
#[cfg(not(target_arch = "m68k"))]
#[inline(always)] pub unsafe fn r16(a: usize) -> u16 { unsafe { (a as *const u16).read_volatile() } }
#[cfg(not(target_arch = "m68k"))]
#[inline(always)] pub unsafe fn r32(a: usize) -> u32 { unsafe { (a as *const u32).read_volatile() } }
#[inline(always)] pub unsafe fn w32(a: usize, v: u32) { unsafe { (a as *mut u32).write_volatile(v) } }

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn host_mmio_reads_and_writes_each_width() {
        let mut buf = [0u32; 2];
        let a = buf.as_mut_ptr() as usize;
        unsafe {
            w32(a, 0x1122_3344);
            assert_eq!(r32(a), 0x1122_3344);
            w16(a + 4, 0xABCD);
            assert_eq!(r16(a + 4), 0xABCD);
            w8(a + 6, 0x5A);
            assert_eq!(r8(a + 6), 0x5A);
        }
    }
}
