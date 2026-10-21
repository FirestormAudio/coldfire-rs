//! An exact MCF5441x (ColdFire V4e) emulator for host-side testing.

pub mod cpu;
pub mod isa;
pub mod machine;
pub mod mem;
pub mod stop;

pub use cpu::{Cpu, ExceptionPolicy};
pub use machine::Machine;
pub use mem::{Fault, Memory, Ram, Size};
pub use stop::{Limit, Stop, StopReason};
