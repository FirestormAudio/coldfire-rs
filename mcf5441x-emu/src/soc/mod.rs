//! Models of the MCF5441x's on-chip modules.

pub mod intc;
pub mod stub;

pub use intc::{GlobalIack, Intc};
pub use stub::{StubBlock, StubMode, Unmodelled};
