//! An exact MCF5441x (ColdFire V4e) emulator for host-side testing.

pub mod cpu;
pub mod device;
pub mod irq;
pub mod isa;
pub mod machine;
pub mod mem;
mod sched;
pub mod stop;

pub use cpu::{Cpu, ExceptionPolicy};
pub use device::{Access, Device, DeviceCx, DeviceId};
pub use irq::{InterruptController, Irq, IrqLines};
pub use machine::Machine;
pub use mem::{Fault, Memory, Ram, Size};
pub use stop::{Limit, Stop, StopReason};
