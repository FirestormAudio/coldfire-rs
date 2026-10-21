//! An exact MCF5441x (ColdFire V4e) emulator for host-side testing.

pub mod isa;
pub mod mem;
pub mod stop;

pub use mem::{Fault, Memory, Ram, Size};
pub use stop::Stop;
