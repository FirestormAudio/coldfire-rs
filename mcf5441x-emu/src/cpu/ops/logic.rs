//! AND, OR, EOR, NOT, CLR, TST, TAS, EXT, EXTB and SWAP (CFPRM ch. 4). N and Z come from the result, V and C are
//! cleared and X is unchanged.

use crate::cpu::ea::{self, sign_extend, size_mask};
use crate::cpu::{Cpu, Decoded, Flow};
use crate::mem::{Fault, Memory, Size};

/// `<ea>y op Dx -> Dx`.
#[inline(always)]
fn ea_dn(
    cpu: &mut Cpu,
    mem: &mut Memory,
    d: &Decoded,
    f: fn(u32, u32) -> u32,
) -> Result<Flow, Fault> {
    let src = ea::load(cpu, mem, &d.ea, Size::Long)?;
    let r = d.operands[0] as usize & 7;
    let res = f(cpu.d[r], src);
    cpu.d[r] = res;
    cpu.flags.logic(Size::Long, res);
    Ok(Flow::Next)
}

/// `<ea>x op Dy -> <ea>x`.
#[inline(always)]
fn dn_ea(
    cpu: &mut Cpu,
    mem: &mut Memory,
    d: &Decoded,
    f: fn(u32, u32) -> u32,
) -> Result<Flow, Fault> {
    let src = cpu.d[d.operands[0] as usize & 7];
    let loc = ea::resolve(cpu, &d.ea, Size::Long);
    let dst = ea::read(cpu, mem, loc, Size::Long)?;
    let res = f(dst, src);
    ea::write(cpu, mem, loc, Size::Long, res)?;
    cpu.flags.logic(Size::Long, res);
    Ok(Flow::Next)
}

/// `Dx op #<data> -> Dx`.
#[inline(always)]
fn imm_dn(cpu: &mut Cpu, d: &Decoded, f: fn(u32, u32) -> u32) -> Result<Flow, Fault> {
    let r = d.operands[1] as usize & 7;
    let res = f(cpu.d[r], d.operands[3]);
    cpu.d[r] = res;
    cpu.flags.logic(Size::Long, res);
    Ok(Flow::Next)
}

fn and(a: u32, b: u32) -> u32 {
    a & b
}
fn or(a: u32, b: u32) -> u32 {
    a | b
}
fn xor(a: u32, b: u32) -> u32 {
    a ^ b
}

pub fn and_ea_dn(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    ea_dn(cpu, mem, d, and)
}
pub fn or_ea_dn(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    ea_dn(cpu, mem, d, or)
}
pub fn and_dn_ea(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    dn_ea(cpu, mem, d, and)
}
pub fn or_dn_ea(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    dn_ea(cpu, mem, d, or)
}
/// EOR Dy,`<ea>`x.
pub fn eor(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    dn_ea(cpu, mem, d, xor)
}
pub fn andi(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    imm_dn(cpu, d, and)
}
pub fn ori(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    imm_dn(cpu, d, or)
}
pub fn eori(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    imm_dn(cpu, d, xor)
}

/// NOT Dx.
pub fn not(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let r = d.operands[1] as usize & 7;
    let res = !cpu.d[r];
    cpu.d[r] = res;
    cpu.flags.logic(Size::Long, res);
    Ok(Flow::Next)
}

/// CLR.sz `<ea>`: writes zero; N=0 Z=1 V=0 C=0, X unchanged.
pub fn clr(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    ea::store(cpu, mem, &d.ea, d.size, 0)?;
    cpu.flags.logic(d.size, 0);
    Ok(Flow::Next)
}

/// `TAS.B <ea>x`: N and Z from the byte as read, then bit 7 is set, in one read-modify-write.
pub fn tas(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let loc = ea::resolve(cpu, &d.ea, Size::Byte);
    let v = ea::read(cpu, mem, loc, Size::Byte)?;
    ea::write(cpu, mem, loc, Size::Byte, v | 0x80)?;
    cpu.flags.logic(Size::Byte, v);
    Ok(Flow::Next)
}

/// TST.sz `<ea>`.
pub fn tst(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let v = ea::load(cpu, mem, &d.ea, d.size)?;
    cpu.flags.logic(d.size, v);
    Ok(Flow::Next)
}

// guess: for EXT.W the manual says only "the result"; N and Z are taken from the 16-bit result (the operation's
// size), not from the whole register, whose upper word is untouched.
/// EXT.W (byte to word, upper word kept) and EXT.L (word to longword).
pub fn ext(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let r = d.operands[1] as usize & 7;
    let v = cpu.d[r];
    let res = match d.size {
        Size::Word => (v & 0xFFFF_0000) | (sign_extend(Size::Byte, v) & 0xFFFF),
        _ => sign_extend(Size::Word, v),
    };
    cpu.d[r] = res;
    cpu.flags.logic(d.size, res & size_mask(d.size));
    Ok(Flow::Next)
}

/// EXTB.L: byte to longword.
pub fn extb(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let r = d.operands[1] as usize & 7;
    let res = sign_extend(Size::Byte, cpu.d[r]);
    cpu.d[r] = res;
    cpu.flags.logic(Size::Long, res);
    Ok(Flow::Next)
}

/// SWAP Dx: exchange the halves; flags from the 32-bit result.
pub fn swap(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let r = d.operands[1] as usize & 7;
    let res = cpu.d[r].rotate_left(16);
    cpu.d[r] = res;
    cpu.flags.logic(Size::Long, res);
    Ok(Flow::Next)
}
