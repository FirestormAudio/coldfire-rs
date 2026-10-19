//! Cold boot: what the chip needs between reset and `main` when no bootloader ran. The board describes its clocks,
//! DDR and boot medium in a [`BootConfig`]; the boot part checks it at compile time against the reference manual's
//! limits and runs it from reset.
mod config;
pub mod ops;
pub mod fetch;
pub mod spi;
#[cfg(target_arch = "m68k")]
mod exec;
pub use config::*;

#[cfg(target_arch = "m68k")]
pub use coldfire_rt::boot_config;

// SAFETY: the boot part reads the linked operations as `ops::Op` and the fetch description as `AppSource`.
#[cfg(target_arch = "m68k")]
unsafe impl coldfire_rt::BootConfig for BootConfig {
    type Op = ops::Op;
    type Fetch = AppSource;
}

impl BootConfig {
    /// Where the boot part fetches the app part from (what `#[boot_config]` links into the boot part).
    pub const fn fetch(&self) -> AppSource {
        self.app
    }
}

#[cfg(target_arch = "m68k")]
unsafe extern "C" {
    static __coldfire_rt_boot_ops: ops::Op;
    static __coldfire_rt_boot_ops_len: usize;
    static __coldfire_rt_boot_fetch: AppSource;
    static __app_vma: u8;
    static __app_len: u8;
    static __app_offset: u8;
}

/// The boot part's entry, which coldfire-rt's cold `_start` calls on the boot stack: run the board's sequence (clocks,
/// DDR, ...) and copy the app part into DDR, then return to start it.
#[cfg(target_arch = "m68k")]
#[unsafe(no_mangle)]
#[unsafe(link_section = ".boot.text.coldfire_rt_boot")]
extern "C" fn __coldfire_rt_boot() {
    unsafe {
        exec::run(&raw const __coldfire_rt_boot_ops, (&raw const __coldfire_rt_boot_ops_len).read_volatile());
        let source = (&raw const __coldfire_rt_boot_fetch).read_volatile();
        let p = fetch::plan(&source, &raw const __app_offset as u32);
        fetch::fetch(p, &raw const __app_vma as u32, &raw const __app_len as u32);
    }
}

#[cfg(all(target_arch = "m68k", feature = "boot-nand"))]
core::arch::global_asm!(
    ".section .boot.reloc,\"ax\"",
    ".global __coldfire_rt_reloc",
    "__coldfire_rt_reloc:",
    "jmp _start",
);
