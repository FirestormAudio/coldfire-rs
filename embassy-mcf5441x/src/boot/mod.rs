//! Cold boot: what the chip needs between reset and `main` when no bootloader ran. The board describes its clocks,
//! DDR and boot medium in a [`BootConfig`]; the boot part checks it at compile time against the reference manual's
//! limits and runs it from reset.
mod config;
pub mod ops;
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

/// The boot part's entry (coldfire-rt's cold `_start` calls it): nothing yet.
#[cfg(target_arch = "m68k")]
#[unsafe(no_mangle)]
#[unsafe(link_section = ".boot.text.coldfire_rt_boot")]
extern "C" fn __coldfire_rt_boot() {}

#[cfg(all(target_arch = "m68k", feature = "boot-nand"))]
core::arch::global_asm!(
    ".section .boot.reloc,\"ax\"",
    ".global __coldfire_rt_reloc",
    "__coldfire_rt_reloc:",
    "jmp _start",
);
