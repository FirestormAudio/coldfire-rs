//! BTST, BSET, BCLR, BCHG and Scc (CFPRM ch. 4). The bit instructions set Z when the tested bit was zero and
//! leave the other flags alone. A data register takes the bit number modulo 32; a memory operand is a byte and
//! takes it modulo 8.

use crate::cpu::ea;
use crate::cpu::{Cpu, Decoded, Flow};
use crate::isa::Mode;
use crate::mem::{Fault, Memory, Size};

use crate::cpu::ccr::Z;

/// What a bit instruction does to the bit after testing it.
#[derive(Clone, Copy)]
enum Act {
    Test,
    Set,
    Clear,
    Change,
}

#[inline(always)]
fn bit_op(
    cpu: &mut Cpu,
    mem: &mut Memory,
    d: &Decoded,
    number: u32,
    act: Act,
) -> Result<Flow, Fault> {
    let (size, mask) = if d.ea.mode == Mode::Dn {
        (Size::Long, 31)
    } else {
        (Size::Byte, 7)
    };
    let bit = 1u32 << (number & mask);
    let loc = ea::resolve(cpu, &d.ea, size);
    let v = ea::read(cpu, mem, loc, size)?;
    let new = match act {
        Act::Test => v,
        Act::Set => v | bit,
        Act::Clear => v & !bit,
        Act::Change => v ^ bit,
    };
    if !matches!(act, Act::Test) {
        ea::write(cpu, mem, loc, size, new)?;
    }
    let ccr = cpu.flags.ccr() & 0x0F;
    cpu.flags
        .set_nzvc(if v & bit == 0 { ccr | Z } else { ccr & !Z });
    Ok(Flow::Next)
}

#[inline(always)]
fn imm(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded, act: Act) -> Result<Flow, Fault> {
    bit_op(cpu, mem, d, d.operands[3] & 0xFF, act)
}

#[inline(always)]
fn dn(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded, act: Act) -> Result<Flow, Fault> {
    let n = cpu.d[d.operands[0] as usize & 7];
    bit_op(cpu, mem, d, n, act)
}

pub fn btst_imm(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    imm(cpu, mem, d, Act::Test)
}
pub fn btst_dn(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    dn(cpu, mem, d, Act::Test)
}
pub fn bset_imm(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    imm(cpu, mem, d, Act::Set)
}
pub fn bset_dn(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    dn(cpu, mem, d, Act::Set)
}
pub fn bclr_imm(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    imm(cpu, mem, d, Act::Clear)
}
pub fn bclr_dn(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    dn(cpu, mem, d, Act::Clear)
}
pub fn bchg_imm(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    imm(cpu, mem, d, Act::Change)
}
pub fn bchg_dn(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    dn(cpu, mem, d, Act::Change)
}

/// Scc Dx: the low byte becomes all ones if the condition holds, else all zeros; the flags are not affected.
pub fn scc(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let byte = if cpu.flags.condition(d.operands[2] as u8) {
        0xFF
    } else {
        0
    };
    let r = d.operands[1] as usize & 7;
    cpu.d[r] = (cpu.d[r] & !0xFF) | byte;
    Ok(Flow::Next)
}
