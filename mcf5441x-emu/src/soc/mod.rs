//! Models of the MCF5441x's on-chip modules.

pub mod intc;
pub mod pit;
pub mod stub;

pub use intc::{GlobalIack, Intc};
pub use pit::Pit;
pub use stub::{StubBlock, StubMode, Unmodelled};
