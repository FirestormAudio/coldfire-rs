//! Models of parts a board puts next to the MCF5441x: each is configured by the board (its size, its ID) and
//! plugged into the on-chip module it hangs off.

pub mod spinor;

pub use spinor::{NorFaults, SpiNor};
