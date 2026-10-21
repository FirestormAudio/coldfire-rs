//! Models of the MCF5441x's on-chip modules.

pub mod dspi;
pub mod edma;
pub mod esdhc;
pub mod intc;
pub mod pit;
pub mod ssi;
pub mod stub;
pub mod uart;

pub use dspi::{Dspi, SpiDevice};
pub use edma::Edma;
pub use esdhc::{Esdhc, EsdhcConfig};
pub use intc::{GlobalIack, Intc};
pub use pit::Pit;
pub use ssi::{Ssi, SsiConfig, SsiCounters};
pub use stub::{StubBlock, StubMode, Unmodelled};
pub use uart::{Uart, UartConfig};
