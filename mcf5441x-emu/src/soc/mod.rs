//! Models of the MCF5441x's on-chip modules.

pub mod ddrmc;
pub mod dspi;
pub mod dtim;
pub mod edma;
pub mod eport;
pub mod esdhc;
pub mod gpio;
pub mod intc;
pub mod map;
pub mod nfc;
pub mod pit;
pub mod pll;
pub mod scm;
pub mod ssi;
pub mod stub;
pub mod uart;
pub mod usb;

pub use ddrmc::Ddrmc;
pub use dspi::{Dspi, Frame, SpiDevice};
pub use dtim::Dtim;
pub use edma::Edma;
pub use eport::Eport;
pub use esdhc::{Esdhc, EsdhcConfig};
pub use gpio::{GpioPorts, PinChange};
pub use intc::{GlobalIack, Intc};
pub use map::{Chip, ChipOptions, Gates, Mcf5441x};
pub use nfc::{NandArray, Nfc};
pub use pit::Pit;
pub use pll::Pll;
pub use scm::PowerManagement;
pub use ssi::{Ssi, SsiConfig, SsiCounters};
pub use stub::{StubBlock, StubMode, Unmodelled};
pub use uart::{PeerCx, SerialPeer, Uart, UartConfig};
pub use usb::{
    HostError, HostScript, Step, Token, UsbConfig, UsbHost, UsbKnobs, UsbOtg, UsbStats, endpoints,
};

/// Whether a `size` access at `offset` runs past the end of the 32-bit register at `offset & !3`. The core splits
/// misaligned accesses (RM Table 3-12), so its accesses never do; a bus master's access is made whole
/// ([`Bus`](crate::Bus)), and a block refuses one that crosses a register.
fn crosses_register(offset: u32, size: crate::mem::Size) -> bool {
    (offset & 3) + size.bytes() > 4
}

/// The detail of a refused access that crosses a register of `block`.
fn crossing(block: &str, offset: u32, size: crate::mem::Size, what: &str) -> String {
    format!(
        "{block}: {}-byte {what} at +{offset:#x} crosses a 32-bit register",
        size.bytes()
    )
}

/// The bits a `size` access at `offset` covers in the big-endian 32-bit register at `offset & !3`, as (shift,
/// mask). The access must not cross the register ([`crosses_register`]).
fn lanes(offset: u32, size: crate::mem::Size) -> (u32, u32) {
    let bytes = size.bytes();
    let shift = (4 - (offset & 3) - bytes) * 8;
    let mask = if bytes == 4 {
        u32::MAX
    } else {
        (1 << (bytes * 8)) - 1
    };
    (shift, mask)
}

/// The part of `reg` a `size` access at `offset` reads.
fn lane_read(reg: u32, offset: u32, size: crate::mem::Size) -> u32 {
    let (shift, mask) = lanes(offset, size);
    reg >> shift & mask
}

/// `reg` with the part a `size` write of `value` at `offset` covers replaced.
fn lane_merge(reg: u32, offset: u32, size: crate::mem::Size, value: u32) -> u32 {
    let (shift, mask) = lanes(offset, size);
    reg & !(mask << shift) | (value & mask) << shift
}
