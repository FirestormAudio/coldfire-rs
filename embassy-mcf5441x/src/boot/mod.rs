//! Cold boot: what the chip needs between reset and `main` when no bootloader ran. The board describes its clocks,
//! DDR and boot medium in a [`BootConfig`]; the boot part checks it at compile time against the reference manual's
//! limits and runs it from reset.
mod config;
pub use config::*;
