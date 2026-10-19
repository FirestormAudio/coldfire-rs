//! Core support for ColdFire V4e (MCF5441x and kin): width-fixed MMIO, status register and interrupt
//! masking, `stop`/`halt`, control registers, the critical-section implementation, the libcalls GCC emits
//! for atomics and byte swaps, and (feature `emac`) the Enhanced Multiply-Accumulate unit.
#![no_std]
#![cfg_attr(target_arch = "m68k", feature(asm_experimental_arch))]

pub mod mmio;
#[cfg(target_arch = "m68k")]
pub mod asm;
#[cfg(target_arch = "m68k")]
pub mod interrupt;
#[cfg(target_arch = "m68k")]
pub mod register;
#[cfg(target_arch = "m68k")]
mod libcalls;
#[cfg(all(target_arch = "m68k", feature = "critical-section-single-core"))]
mod cs;
#[cfg(feature = "emac")]
pub mod emac;
