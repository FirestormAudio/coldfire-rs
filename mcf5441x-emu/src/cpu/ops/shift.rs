//! ASL, ASR, LSL and LSR (CFPRM ch. 4): longword shifts by an immediate count of 1 to 8 or by the low six bits
//! of a data register (modulo 64). C and X receive the last bit shifted out, V is always clear, and a zero count
//! clears C and leaves X.

use crate::cpu::{Cpu, Decoded, Flow};
use crate::mem::{Fault, Memory};

/// Shift `v` by `n` (0 to 63); returns the result and the last bit shifted out. The operand is widened to 64
/// bits so that counts past 32 shift everything out and the carry follows.
type Op = fn(u32, u32) -> (u32, bool);

fn asl(v: u32, n: u32) -> (u32, bool) {
    let w = (v as u64) << n;
    (w as u32, (w >> 32) & 1 != 0)
}

fn lsr(v: u32, n: u32) -> (u32, bool) {
    let w = ((v as u64) << 32) >> n;
    ((w >> 32) as u32, (w >> 31) & 1 != 0)
}

fn asr(v: u32, n: u32) -> (u32, bool) {
    let w = (((v as i32 as i64) << 32) >> n) as u64;
    ((w >> 32) as u32, (w >> 31) & 1 != 0)
}

#[inline(always)]
fn apply(cpu: &mut Cpu, reg: u32, count: u32, op: Op) -> Result<Flow, Fault> {
    let r = reg as usize & 7;
    let (res, carry) = if count == 0 {
        (cpu.d[r], false)
    } else {
        op(cpu.d[r], count)
    };
    cpu.d[r] = res;
    cpu.flags.shift(res, carry, count != 0);
    Ok(Flow::Next)
}

/// The immediate form: the count field holds 1 to 7, and 0 means 8.
#[inline(always)]
fn imm(cpu: &mut Cpu, d: &Decoded, op: Op) -> Result<Flow, Fault> {
    let q = d.operands[2];
    apply(cpu, d.operands[1], if q == 0 { 8 } else { q }, op)
}

/// The register form: the count is in Dy, modulo 64.
#[inline(always)]
fn reg(cpu: &mut Cpu, d: &Decoded, op: Op) -> Result<Flow, Fault> {
    let count = cpu.d[d.operands[0] as usize & 7] & 63;
    apply(cpu, d.operands[1], count, op)
}

pub fn asl_imm(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    imm(cpu, d, asl)
}
pub fn asl_reg(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    reg(cpu, d, asl)
}
pub fn asr_imm(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    imm(cpu, d, asr)
}
pub fn asr_reg(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    reg(cpu, d, asr)
}
// LSL is the same operation as ASL here: V is always clear, and zeros enter at the bottom.
pub fn lsl_imm(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    imm(cpu, d, asl)
}
pub fn lsl_reg(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    reg(cpu, d, asl)
}
pub fn lsr_imm(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    imm(cpu, d, lsr)
}
pub fn lsr_reg(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    reg(cpu, d, lsr)
}
