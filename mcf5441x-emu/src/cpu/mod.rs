//! The V4e integer core: programmer-visible state, decoding into [`Decoded`] and the instruction handlers.

pub mod ccr;
pub mod decode;
pub mod ea;
pub mod exception;
pub mod exec;
pub mod ops;

pub use ccr::Flags;
pub use decode::{DecodeError, Decoded, decode};
pub use ea::Ea;
pub use exception::ExceptionPolicy;
pub use exec::{Flow, Handler, handler};

/// SR bits that exist (RM §3.2.9, Figure 3-8): T, S, M, I2-I0 and the CCR; the others read as zero.
const SR_MASK: u16 = 0xB71F;
/// The trace bit.
pub const SR_T: u16 = 0x8000;
/// The supervisor bit.
pub const SR_S: u16 = 0x2000;
/// The master/interrupt-state bit.
pub const SR_M: u16 = 0x1000;
/// CACR\[EUSP\]: separate user and supervisor stack pointers (RM 3.2.3, RM ch. 6 CACR bit 5).
pub const CACR_EUSP: u32 = 0x20;

/// The programmer-visible state of the core.
#[derive(Debug, Clone)]
pub struct Cpu {
    pub d: [u32; 8],
    /// A0-A6 and the active stack pointer in A7.
    pub a: [u32; 8],
    pub pc: u32,
    /// The inactive stack pointer: the user SP while in supervisor mode, the supervisor SP while in user mode.
    pub other_sp: u32,
    /// The vector base: bits 31-20 are used (RM 3.2.8).
    pub vbr: u32,
    /// The control registers MOVEC writes (RM Table 3-1). They are stored as written; of their effects only
    /// CACR\[EUSP\] is modelled here.
    pub cacr: u32,
    pub asid: u32,
    /// ACR0-ACR7.
    pub acr: [u32; 8],
    pub mmubar: u32,
    pub rgpiobar: u32,
    pub rambar: u32,
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
    /// The state after reset (RM ch. 3, reset exception: S set, T and M clear, interrupt mask 7, VBR zero; CACR
    /// zero, so one stack pointer) before the reset vector is fetched. The manual leaves the CCR undefined at
    /// reset; this core clears it.
    pub fn new() -> Cpu {
        Cpu {
            d: [0; 8],
            a: [0; 8],
            pc: 0,
            other_sp: 0,
            vbr: 0,
            cacr: 0,
            asid: 0,
            acr: [0; 8],
            mmubar: 0,
            rgpiobar: 0,
            rambar: 0,
            sys: 0x2700,
            flags: Flags::default(),
        }
    }

    /// The status register.
    pub fn sr(&self) -> u16 {
        self.sys | self.ccr() as u16
    }

    /// Write the status register. With CACR\[EUSP\] set, a change of S swaps A7 and [`Cpu::other_sp`]; with it
    /// clear the core has one stack pointer (RM 3.2.3).
    pub fn set_sr(&mut self, sr: u16) {
        let sr = sr & SR_MASK;
        if (sr ^ self.sys) & SR_S != 0 && self.dual_stacks() {
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

    /// Whether the trace bit is set.
    #[inline(always)]
    pub fn tracing(&self) -> bool {
        self.sys & SR_T != 0
    }

    /// Whether CACR\[EUSP\] gives the core separate user and supervisor stack pointers.
    #[inline(always)]
    pub fn dual_stacks(&self) -> bool {
        self.cacr & CACR_EUSP != 0
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
