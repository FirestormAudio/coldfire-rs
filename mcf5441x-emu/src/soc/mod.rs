//! Models of the MCF5441x's on-chip modules.

pub mod edma;
pub mod intc;
pub mod pit;
pub mod stub;

pub use edma::Edma;
pub use intc::{GlobalIack, Intc};
pub use pit::Pit;
pub use stub::{StubBlock, StubMode, Unmodelled};
