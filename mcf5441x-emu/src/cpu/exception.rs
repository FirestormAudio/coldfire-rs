//! Exception processing (CFPRM ch. 11, RM 3.3.3): the stack frame, the vector fetch, and which exceptions a run
//! stops on instead of taking them.

use crate::cpu::{Cpu, SR_M, SR_S, SR_T};
use crate::mem::{Fault, Memory, Size};
use crate::stop::Stop;

/// Vector numbers (CFPRM Table 11-1, RM Table 3-5).
pub mod vector {
    pub const ADDRESS_ERROR: u8 = 3;
    pub const ILLEGAL: u8 = 4;
    pub const DIVIDE_BY_ZERO: u8 = 5;
    pub const PRIVILEGE_VIOLATION: u8 = 8;
    pub const TRACE: u8 = 9;
    pub const LINE_F: u8 = 11;
    pub const FORMAT_ERROR: u8 = 14;
    /// TRAP #0; TRAP #n is `TRAP_0 + n`.
    pub const TRAP_0: u8 = 32;
}

// Measured on an MCF54415: a jump to an odd address writes FS 0100 "error on instruction fetch" (RM Table 3-7) with
// the jump as the stacked PC; a word-sized index, a scale factor of 8 or a full-format extension word (RM 3.3.4.2)
// writes FS 1100 "error on operand read" with the faulting instruction as the stacked PC. An odd JSR or RTS target
// writes FS 0100 too, measured. guess: an odd BSR target (by analogy with JSR) and an odd PC loaded by RTE (by
// analogy with RTS) were not measured; they take 0100.
/// The fault status of an address error from an odd instruction address: 0100.
pub const ADDRESS_ERROR_FETCH_FS: u32 = 0b0100;
/// The fault status of an address error from an index extension word RM 3.3.4.2 rejects: 1100.
pub const ADDRESS_ERROR_OPERAND_FS: u32 = 0b1100;

/// Bits 31-20 of VBR; bits 19-0 are not implemented and read as zero (RM 3.2.8).
pub const VBR_MASK: u32 = 0xFFF0_0000;

/// Which exceptions stop a [`Machine::run`](crate::Machine::run) instead of being taken.
///
/// `stop_on[v]` makes an exception with vector `v` end the run with [`Stop::Exception`] (`v`), or
/// [`Stop::Illegal`] for vector 4, with the registers as they were before the instruction and PC on it. Otherwise
/// the core builds the frame and continues in the handler from the vector table.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ExceptionPolicy {
    pub stop_on: [bool; 256],
}

impl Default for ExceptionPolicy {
    /// Stop on every vector except TRAP #0-15 (32-47) and the device interrupts (64-255).
    fn default() -> Self {
        let mut stop_on = [true; 256];
        for (v, stop) in stop_on.iter_mut().enumerate() {
            *stop = !((32..48).contains(&v) || v >= 64);
        }
        ExceptionPolicy { stop_on }
    }
}

/// Whether the frame of exception `vector` holds the address of the next instruction ("Next" in CFPRM
/// Table 11-1) rather than the one that caused it ("Fault").
pub fn stacks_next_pc(vector: u8) -> bool {
    matches!(vector, 9 | 12 | 13 | 15 | 24..=47 | 64..=255)
}

impl Cpu {
    /// Take exception `vector` with `return_pc` in the frame (CFPRM ch. 11 steps 1-4): enter supervisor mode with
    /// T clear, write the two-longword frame (format `4 + SSP[1:0]`, fault status ([`ADDRESS_ERROR_FETCH_FS`] for
    /// an address error, else 0; [`Cpu::raise_with_fs`] gives another, such as [`ADDRESS_ERROR_OPERAND_FS`]),
    /// vector, the SR before the exception, then
    /// `return_pc`) at the SSP aligned down to 4, and continue at the handler read from `VBR + 4 * vector`.
    ///
    /// On an error nothing in the core changes (memory may hold part of the frame). A handler address with bit 0
    /// set is a fault during exception processing, which halts the core (RM 3.3.4.14): it is reported as
    /// `Fault::Device(Stop::Halt("fault-on-fault"))`.
    pub fn raise(&mut self, mem: &mut Memory, vector: u8, return_pc: u32) -> Result<(), Fault> {
        let fs = if vector == vector::ADDRESS_ERROR {
            ADDRESS_ERROR_FETCH_FS
        } else {
            0
        };
        self.raise_with_fs(mem, vector, fs, return_pc)
    }

    /// As [`Cpu::raise`], with the fault status `fs` (RM Table 3-7) in the frame.
    pub fn raise_with_fs(
        &mut self,
        mem: &mut Memory,
        vector: u8,
        fs: u32,
        return_pc: u32,
    ) -> Result<(), Fault> {
        let sys = (self.sr() & !SR_T) | SR_S;
        self.enter(mem, vector, fs, return_pc, sys)
    }

    /// Take interrupt `vector` at priority `level` (1-7): as [`Cpu::raise`], and also clear M and set the
    /// interrupt mask to `level` (CFPRM ch. 11 step 1, RM Table 3-3).
    pub fn interrupt(
        &mut self,
        mem: &mut Memory,
        vector: u8,
        level: u8,
        return_pc: u32,
    ) -> Result<(), Fault> {
        let sys = (self.sr() & !(SR_T | SR_M | 0x0700)) | SR_S | ((level as u16 & 7) << 8);
        self.enter(mem, vector, 0, return_pc, sys)
    }

    fn enter(
        &mut self,
        mem: &mut Memory,
        vector: u8,
        fs: u32,
        return_pc: u32,
        sr: u16,
    ) -> Result<(), Fault> {
        let old = self.sr();
        let ssp = if self.supervisor() || !self.dual_stacks() {
            self.a[7]
        } else {
            self.other_sp
        };
        let format = 4 | (ssp & 3);
        let frame = (ssp & !3).wrapping_sub(8);
        mem.write(
            frame,
            Size::Long,
            (format << 28)
                | ((fs >> 2) << 26)
                | ((vector as u32) << 18)
                | ((fs & 3) << 16)
                | old as u32,
        )?;
        mem.write(frame.wrapping_add(4), Size::Long, return_pc)?;
        let handler = mem.read(
            (self.vbr & VBR_MASK).wrapping_add(4 * vector as u32),
            Size::Long,
        )?;
        if handler & 1 != 0 {
            return Err(Fault::Device(Stop::Halt("fault-on-fault")));
        }
        self.set_sr(sr);
        self.a[7] = frame;
        self.pc = handler;
        Ok(())
    }
}
