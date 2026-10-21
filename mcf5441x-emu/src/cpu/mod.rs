//! The V4e integer core: programmer-visible state, decoding into [`Decoded`] and the instruction handlers.

pub mod ccr;
pub mod decode;
pub mod ea;
pub mod exec;
pub mod ops;

pub use ccr::Flags;
pub use decode::{DecodeError, Decoded, decode};
pub use ea::Ea;
pub use exec::{Flow, Handler, handler};

use crate::stop::Stop;

/// SR bits that exist (RM §3.2.9, Figure 3-8): T, S, M, I2-I0 and the CCR; the others read as zero.
const SR_MASK: u16 = 0xB71F;
/// The supervisor bit.
pub const SR_S: u16 = 0x2000;

/// The programmer-visible state of the core.
#[derive(Debug, Clone)]
pub struct Cpu {
    pub d: [u32; 8],
    /// A0-A6 and the active stack pointer in A7.
    pub a: [u32; 8],
    pub pc: u32,
    /// The inactive stack pointer: the user SP while in supervisor mode, the supervisor SP while in user mode.
    pub other_sp: u32,
    pub vbr: u32,
    /// SR without the CCR (T, S, M, IPL).
    sys: u16,
    pub(crate) flags: Flags,
}

impl Default for Cpu {
    fn default() -> Self {
        Cpu::new()
    }
}

impl Cpu {
    /// The state after reset (RM ch. 3, reset exception: S set, T and M clear, interrupt mask 7, VBR zero) before
    /// the reset vector is fetched. The manual leaves the CCR undefined at reset; this core clears it.
    pub fn new() -> Cpu {
        Cpu {
            d: [0; 8],
            a: [0; 8],
            pc: 0,
            other_sp: 0,
            vbr: 0,
            sys: 0x2700,
            flags: Flags::default(),
        }
    }

    /// The status register.
    pub fn sr(&self) -> u16 {
        self.sys | self.ccr() as u16
    }

    /// Write the status register; a change of S swaps A7 and [`Cpu::other_sp`].
    pub fn set_sr(&mut self, sr: u16) {
        let sr = sr & SR_MASK;
        if (sr ^ self.sys) & SR_S != 0 {
            std::mem::swap(&mut self.a[7], &mut self.other_sp);
        }
        self.sys = sr & 0xFF00;
        self.flags.set_ccr(sr as u8);
    }

    /// The condition-code register: X N Z V C in bits 4-0.
    pub fn ccr(&self) -> u8 {
        self.flags.ccr()
    }

    /// Write X N Z V C from bits 4-0.
    pub fn set_ccr(&mut self, ccr: u8) {
        self.flags.set_ccr(ccr & 0x1F);
    }

    /// Whether the core is in supervisor mode.
    pub fn supervisor(&self) -> bool {
        self.sys & SR_S != 0
    }

    /// Take exception `vector` from the instruction being executed. The one place instruction handlers raise an
    /// exception; for now the run stops there with the registers as they were.
    pub fn raise(&mut self, vector: u8) -> Flow {
        Flow::Stop(Stop::Exception(vector))
    }

    /// Register `n` of D0-D7 (0-7) and A0-A7 (8-15).
    #[inline(always)]
    pub fn reg(&self, n: u8) -> u32 {
        if n < 8 {
            self.d[n as usize & 7]
        } else {
            self.a[n as usize & 7]
        }
    }
}
