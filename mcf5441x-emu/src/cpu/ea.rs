//! Effective addresses (CFPRM ch. 2), decoded once and evaluated per execution.

use crate::cpu::Cpu;
use crate::isa::Mode;
use crate::mem::{Fault, Memory, Size};

/// A decoded effective address: the mode, its register and its extension words reduced to one value.
///
/// | mode                  | `reg`       | `value`                                          |
/// |-----------------------|-------------|--------------------------------------------------|
/// | Dn, An, (An), (An)+, -(An) | n      | 0                                                |
/// | (d16,An)              | n           | sign-extended d16                                |
/// | (d8,An,Xi)            | n           | sign-extended d8                                 |
/// | (xxx).W, (xxx).L      | 0           | the address (sign-extended for .W)               |
/// | (d16,PC)              | 0           | the address: PC + 2 + d16 (CFPRM 2.2.8)          |
/// | (d8,PC,Xi)            | 0           | PC + 2 + d8, to which the scaled index is added  |
/// | #imm                  | 0           | the operand, zero-extended                       |
///
/// For the indexed modes `index` is the index register (0-7 D0-D7, 8-15 A0-A7, always a longword: RM 3.3.4.2)
/// and `scale` its left shift.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Ea {
    pub mode: Mode,
    pub reg: u8,
    pub index: u8,
    pub scale: u8,
    pub value: u32,
}

impl Ea {
    /// The placeholder for a form without this EA field.
    pub const NONE: Ea = Ea {
        mode: Mode::Dn,
        reg: 0,
        index: 0,
        scale: 0,
        value: 0,
    };
}

/// Where an operand lives once its effective address has been evaluated.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Loc {
    D(u8),
    A(u8),
    Mem(u32),
    Imm(u32),
}

/// Evaluate `ea` for an access of `size`, applying the (An)+ and -(An) register updates. A7 steps by the operand
/// size like any other address register (CFPRM 2.2.4, 2.2.5), so a byte access moves it by one.
#[inline(always)]
pub fn resolve(cpu: &mut Cpu, ea: &Ea, size: Size) -> Loc {
    let r = ea.reg as usize & 7;
    match ea.mode {
        Mode::Dn => Loc::D(ea.reg),
        Mode::An => Loc::A(ea.reg),
        Mode::Ind => Loc::Mem(cpu.a[r]),
        Mode::PostInc => {
            let at = cpu.a[r];
            cpu.a[r] = at.wrapping_add(size.bytes());
            Loc::Mem(at)
        }
        Mode::PreDec => {
            cpu.a[r] = cpu.a[r].wrapping_sub(size.bytes());
            Loc::Mem(cpu.a[r])
        }
        Mode::Disp => Loc::Mem(cpu.a[r].wrapping_add(ea.value)),
        Mode::Index => Loc::Mem(
            cpu.a[r]
                .wrapping_add(ea.value)
                .wrapping_add(cpu.reg(ea.index) << ea.scale),
        ),
        Mode::AbsW | Mode::AbsL | Mode::PcDisp => Loc::Mem(ea.value),
        Mode::PcIndex => Loc::Mem(ea.value.wrapping_add(cpu.reg(ea.index) << ea.scale)),
        Mode::Imm => Loc::Imm(ea.value),
    }
}

/// The address `ea` designates, for the control modes (LEA, PEA, JMP, JSR); `None` for the register and
/// immediate modes.
#[inline(always)]
pub fn address(cpu: &mut Cpu, ea: &Ea) -> Option<u32> {
    match resolve(cpu, ea, Size::Long) {
        Loc::Mem(a) => Some(a),
        _ => None,
    }
}

/// Read a `size` operand, zero-extended. An address register read as a word gives its low word.
#[inline(always)]
pub fn read(cpu: &Cpu, mem: &mut Memory, loc: Loc, size: Size) -> Result<u32, Fault> {
    let mask = size_mask(size);
    match loc {
        Loc::D(r) => Ok(cpu.d[r as usize & 7] & mask),
        Loc::A(r) => Ok(cpu.a[r as usize & 7] & mask),
        Loc::Mem(a) => mem.read(a, size),
        Loc::Imm(v) => Ok(v & mask),
    }
}

/// Write a `size` operand. A data register keeps its bits above `size`; an address register is always written
/// whole (the caller sign-extends, as MOVEA does). Immediates are not writable and the decode table never makes
/// them a destination; a write to one is ignored.
#[inline(always)]
pub fn write(
    cpu: &mut Cpu,
    mem: &mut Memory,
    loc: Loc,
    size: Size,
    value: u32,
) -> Result<(), Fault> {
    match loc {
        Loc::D(r) => {
            let mask = size_mask(size);
            let d = &mut cpu.d[r as usize & 7];
            *d = (*d & !mask) | (value & mask);
            Ok(())
        }
        Loc::A(r) => {
            cpu.a[r as usize & 7] = value;
            Ok(())
        }
        Loc::Mem(a) => mem.write(a, size, value),
        Loc::Imm(_) => Ok(()),
    }
}

/// Evaluate and read in one step.
#[inline(always)]
pub fn load(cpu: &mut Cpu, mem: &mut Memory, ea: &Ea, size: Size) -> Result<u32, Fault> {
    let loc = resolve(cpu, ea, size);
    read(cpu, mem, loc, size)
}

/// Evaluate and write in one step.
#[inline(always)]
pub fn store(
    cpu: &mut Cpu,
    mem: &mut Memory,
    ea: &Ea,
    size: Size,
    value: u32,
) -> Result<(), Fault> {
    let loc = resolve(cpu, ea, size);
    write(cpu, mem, loc, size, value)
}

/// The low `size` bits.
#[inline(always)]
pub const fn size_mask(size: Size) -> u32 {
    match size {
        Size::Byte => 0xFF,
        Size::Word => 0xFFFF,
        Size::Long => 0xFFFF_FFFF,
    }
}

/// Sign-extend the low `size` bits.
#[inline(always)]
pub const fn sign_extend(size: Size, v: u32) -> u32 {
    match size {
        Size::Byte => v as u8 as i8 as u32,
        Size::Word => v as u16 as i16 as u32,
        Size::Long => v,
    }
}
