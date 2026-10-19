//! Minimal runtime for ColdFire V4e: reset, a RAM vector table, exception trampolines. The board's memory.x
//! places everything (see link.x); interrupt masking, the critical section and the GCC libcalls are in
//! `coldfire`.
#![no_std]
#![cfg_attr(any(target_arch = "m68k", docsrs), feature(asm_experimental_arch, allow_internal_unstable))]
#![cfg_attr(any(target_arch = "m68k", docsrs), allow(internal_features))] // allow_internal_unstable, for exception_entry!
// The startup and trampolines are ColdFire assembly: a host build has the re-exports and `Handler` only, and docs.rs
// (`--cfg docsrs`) also `set_vector` and `exception_entry!`.

#[cfg(target_arch = "m68k")]
use core::arch::global_asm;

pub use coldfire; // re-exported, and linked: its libcalls and critical section are needed
pub use coldfire_rt_macros::{boot_config, entry, exception, interrupt};

/// One `#[exception]`/`#[interrupt]` handler: the linker gathers these from `.coldfire_rt.handlers` (link.x
/// brackets them with `__shandlers`/`__ehandlers`) and startup installs each into the RAM vector table.
#[repr(C)]
pub struct Handler {
    pub vector: usize,
    pub entry: unsafe extern "C" fn(),
}

#[cfg(all(target_arch = "m68k", not(feature = "cold")))]
global_asm!(
    ".section .text._start,\"ax\"",
    ".global _start",
    "_start:",
    "move.w #0x2700,%sr",
    "lea __stack_top,%sp",
    "jmp coldfire_rt_start",
);

// Cold: the reset vector (offset 0) and a _start in the boot part. The HAL's __coldfire_rt_boot sets up the chip and
// copies the app part into DDR; then the usual startup runs there.
#[cfg(all(target_arch = "m68k", feature = "cold"))]
global_asm!(
    ".section .boot.vectors,\"a\"",
    ".long __boot_stack_top, __boot_entry",
    ".section .boot.text._start,\"ax\"",
    ".global _start",
    "_start:",
    "move.w #0x2700,%sr",
    "lea __boot_stack_top,%sp",
    "jsr __coldfire_rt_boot",
    "lea __stack_top,%sp",
    "jmp coldfire_rt_start",
);

/// A cold-boot description for `#[boot_config]`. The chip's HAL implements it for its config type, which also has
/// the inherent `const fn`s `ops_len(&self) -> usize`, `ops::<N>(&self) -> [Self::Op; N]` and
/// `fetch(&self) -> Self::Fetch`; `#[boot_config]` links their results into the boot part.
///
/// # Safety
/// The boot part reads `__coldfire_rt_boot_ops` as `[Op; __coldfire_rt_boot_ops_len]` and `__coldfire_rt_boot_fetch`
/// as `Fetch`: the implementing HAL must read them as exactly these types.
pub unsafe trait BootConfig {
    type Op: Copy + 'static;
    type Fetch: Copy + 'static;
}

#[cfg(any(target_arch = "m68k", docsrs))]
#[unsafe(link_section = ".vectors")]
static mut VECTORS: [usize; 256] = [0; 256];

#[cfg(target_arch = "m68k")]
unsafe extern "C" {
    static mut __sbss: u32;
    static mut __ebss: u32;
    static __shandlers: Handler;
    static __ehandlers: Handler;
    fn main() -> !;
}

#[cfg(target_arch = "m68k")]
#[unsafe(no_mangle)]
unsafe extern "C" fn coldfire_rt_start() -> ! {
    unsafe {
        let mut p = &raw mut __sbss as *mut u32;
        let end = &raw mut __ebss as *mut u32;
        while p < end {
            p.write_volatile(0);
            p = p.add(1);
        }
        let v = &raw mut VECTORS;
        for i in 0..256 {
            (*v)[i] = default_entry as *const () as usize;
        }
        let mut h = &raw const __shandlers;
        let end = &raw const __ehandlers;
        while h < end {
            (*v)[(*h).vector] = (*h).entry as *const () as usize;
            h = h.add(1);
        }
        let base = v as u32;
        coldfire::register::vbr::write(base);
        main()
    }
}

#[cfg(any(target_arch = "m68k", docsrs))]
/// Install `entry` at vector `n` at run time; it replaces whatever startup put there (the default or a table handler).
pub unsafe fn set_vector(n: usize, entry: unsafe extern "C" fn()) {
    unsafe { (*(&raw mut VECTORS))[n] = entry as *const () as usize };
}

/// Exception trampoline: saves the C caller-saved registers, calls `$handler(frame_word)`, restores, `rte`.
/// It doesn't save the EMAC (MACSR, accumulators): a handler that uses it must save them itself.
/// Its `global_asm!` is allowed in crates that don't enable `asm_experimental_arch` themselves (e.g. an
/// `#[interrupt]` expansion).
#[cfg(any(target_arch = "m68k", docsrs))]
#[macro_export]
#[allow_internal_unstable(asm_experimental_arch)]
macro_rules! exception_entry {
    ($entry:ident, $handler:ident) => {
        core::arch::global_asm!(
            concat!(".pushsection .text.", stringify!($entry), ",\"ax\""),
            ".balign 2",
            concat!(".global ", stringify!($entry)),
            concat!(stringify!($entry), ":"),
            "lea -16(%sp),%sp",
            "movem.l %d0-%d1/%a0-%a1,(%sp)",
            "move.l 16(%sp),-(%sp)",
            concat!("jsr ", stringify!($handler)),
            "addq.l #4,%sp",
            "movem.l (%sp),%d0-%d1/%a0-%a1",
            "lea 16(%sp),%sp",
            "rte",
            ".popsection",
        );
        unsafe extern "C" {
            pub fn $entry();
        }
    };
}

#[cfg(target_arch = "m68k")]
exception_entry!(default_entry, coldfire_rt_default_handler);

#[cfg(target_arch = "m68k")]
#[unsafe(no_mangle)]
extern "C" fn coldfire_rt_default_handler(frame_word: u32) {
    panic!("unhandled exception, vector {}", (frame_word >> 18) & 0xFF);
}
