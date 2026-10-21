//! Integer arithmetic (CFPRM ch. 4).

use crate::cpu::ea::{self, Loc};
use crate::cpu::{Cpu, Decoded, Flow, ccr};
use crate::mem::{Fault, Memory, Size};

/// The 3-bit quick-data field: 1 to 8, with 0 meaning 8.
#[inline(always)]
fn quick(d: &Decoded) -> u32 {
    let q = d.operands[2] & 7;
    if q == 0 { 8 } else { q }
}

/// ADDQ/SUBQ: `dst op q`. An address register destination leaves the condition codes alone.
#[inline(always)]
fn quick_op(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded, sub: bool) -> Result<Flow, Fault> {
    let q = quick(d);
    let loc = ea::resolve(cpu, &d.ea, Size::Long);
    let dst = ea::read(cpu, mem, loc, Size::Long)?;
    let res = if sub {
        dst.wrapping_sub(q)
    } else {
        dst.wrapping_add(q)
    };
    ea::write(cpu, mem, loc, Size::Long, res)?;
    if !matches!(loc, Loc::A(_)) {
        if sub {
            cpu.flags.sub(Size::Long, q, dst, res);
        } else {
            cpu.flags.add(Size::Long, q, dst, res);
        }
    }
    Ok(Flow::Next)
}

/// ADDQ.L #`<data>`,`<ea>`x: X N Z V C as ADD.
pub fn addq(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    quick_op(cpu, mem, d, false)
}

/// SUBQ.L #`<data>`,`<ea>`x: X N Z V C as SUB.
pub fn subq(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    quick_op(cpu, mem, d, true)
}

/// Record the flags of `dst op src = res` (op is SUB when `sub`), X = C.
#[inline(always)]
fn flags(cpu: &mut Cpu, size: Size, sub: bool, src: u32, dst: u32, res: u32) {
    if sub {
        cpu.flags.sub(size, src, dst, res);
    } else {
        cpu.flags.add(size, src, dst, res);
    }
}

#[inline(always)]
fn calc(sub: bool, dst: u32, src: u32) -> u32 {
    if sub {
        dst.wrapping_sub(src)
    } else {
        dst.wrapping_add(src)
    }
}

/// ADD/SUB `<ea>`y,Dx.
#[inline(always)]
fn ea_dn(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded, sub: bool) -> Result<Flow, Fault> {
    let src = ea::load(cpu, mem, &d.ea, Size::Long)?;
    let r = d.operands[0] as usize & 7;
    let dst = cpu.d[r];
    let res = calc(sub, dst, src);
    cpu.d[r] = res;
    flags(cpu, Size::Long, sub, src, dst, res);
    Ok(Flow::Next)
}

/// ADD/SUB Dy,`<ea>`x.
#[inline(always)]
fn dn_ea(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded, sub: bool) -> Result<Flow, Fault> {
    let src = cpu.d[d.operands[0] as usize & 7];
    let loc = ea::resolve(cpu, &d.ea, Size::Long);
    let dst = ea::read(cpu, mem, loc, Size::Long)?;
    let res = calc(sub, dst, src);
    ea::write(cpu, mem, loc, Size::Long, res)?;
    flags(cpu, Size::Long, sub, src, dst, res);
    Ok(Flow::Next)
}

/// ADDI/SUBI #`<data>`,Dx.
#[inline(always)]
fn imm_dn(cpu: &mut Cpu, d: &Decoded, sub: bool) -> Result<Flow, Fault> {
    let r = d.operands[1] as usize & 7;
    let (src, dst) = (d.operands[3], cpu.d[r]);
    let res = calc(sub, dst, src);
    cpu.d[r] = res;
    flags(cpu, Size::Long, sub, src, dst, res);
    Ok(Flow::Next)
}

/// ADDA/SUBA `<ea>`y,Ax: the whole register, no condition codes.
#[inline(always)]
fn to_an(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded, sub: bool) -> Result<Flow, Fault> {
    let src = ea::load(cpu, mem, &d.ea, Size::Long)?;
    let r = d.operands[0] as usize & 7;
    cpu.a[r] = calc(sub, cpu.a[r], src);
    Ok(Flow::Next)
}

/// ADDX/SUBX Dy,Dx: the extend bit joins the operation, and Z is only ever cleared (CFPRM ch. 4: "cleared if the
/// result is nonzero; unchanged otherwise"), so a chain of them ends with Z set only if the whole result is zero.
#[inline(always)]
fn extended(cpu: &mut Cpu, d: &Decoded, sub: bool) -> Result<Flow, Fault> {
    let (rx, ry) = (d.operands[0] as usize & 7, d.operands[1] as usize & 7);
    let (src, dst) = (cpu.d[ry], cpu.d[rx]);
    let old = cpu.ccr();
    let x = (old & ccr::X != 0) as u32;
    let res = if sub {
        dst.wrapping_sub(src).wrapping_sub(x)
    } else {
        dst.wrapping_add(src).wrapping_add(x)
    };
    cpu.d[rx] = res;
    extended_ccr(cpu, old, sub, src, dst, res);
    Ok(Flow::Next)
}

/// Set X N V C from the operation and Z from `res` and the old Z.
#[inline(always)]
fn extended_ccr(cpu: &mut Cpu, old: u8, sub: bool, src: u32, dst: u32, res: u32) {
    let mut f = ccr::Flags::default();
    flags_of(&mut f, sub, src, dst, res);
    let nvc = f.nzvc() & !ccr::Z;
    let c = nvc & ccr::C;
    let z = if res != 0 { 0 } else { old & ccr::Z };
    cpu.set_ccr(nvc | z | if c != 0 { ccr::X } else { 0 });
}

#[inline(always)]
fn flags_of(f: &mut ccr::Flags, sub: bool, src: u32, dst: u32, res: u32) {
    if sub {
        f.sub(Size::Long, src, dst, res);
    } else {
        f.add(Size::Long, src, dst, res);
    }
}

/// CMP/CMPI: the flags of `dst - src`, X unchanged.
#[inline(always)]
fn compare(cpu: &mut Cpu, size: Size, src: u32, dst: u32) {
    cpu.flags.cmp(size, src, dst, dst.wrapping_sub(src));
}

pub fn add_ea_dn(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    ea_dn(cpu, mem, d, false)
}
pub fn sub_ea_dn(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    ea_dn(cpu, mem, d, true)
}
pub fn add_dn_ea(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    dn_ea(cpu, mem, d, false)
}
pub fn sub_dn_ea(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    dn_ea(cpu, mem, d, true)
}
pub fn addi(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    imm_dn(cpu, d, false)
}
pub fn subi(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    imm_dn(cpu, d, true)
}
pub fn adda(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    to_an(cpu, mem, d, false)
}
pub fn suba(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    to_an(cpu, mem, d, true)
}
pub fn addx(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    extended(cpu, d, false)
}
pub fn subx(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    extended(cpu, d, true)
}

/// CMP.sz `<ea>`y,Dx.
pub fn cmp(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let src = ea::load(cpu, mem, &d.ea, d.size)?;
    let dst = cpu.d[d.operands[0] as usize & 7] & ea::size_mask(d.size);
    compare(cpu, d.size, src, dst);
    Ok(Flow::Next)
}

/// CMPI.sz #`<data>`,Dx.
pub fn cmpi(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let dst = cpu.d[d.operands[1] as usize & 7] & ea::size_mask(d.size);
    compare(cpu, d.size, d.operands[3], dst);
    Ok(Flow::Next)
}

/// CMPA.sz `<ea>`y,Ax: a word source is sign-extended; the comparison is always long.
pub fn cmpa(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let src = ea::sign_extend(d.size, ea::load(cpu, mem, &d.ea, d.size)?);
    let dst = cpu.a[d.operands[0] as usize & 7];
    compare(cpu, Size::Long, src, dst);
    Ok(Flow::Next)
}

/// NEG Dx: `0 - Dx`, X = C.
pub fn neg(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let r = d.operands[1] as usize & 7;
    let src = cpu.d[r];
    let res = 0u32.wrapping_sub(src);
    cpu.d[r] = res;
    cpu.flags.sub(Size::Long, src, 0, res);
    Ok(Flow::Next)
}

/// NEGX Dx: `0 - Dx - X`, Z only cleared.
pub fn negx(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let r = d.operands[1] as usize & 7;
    let src = cpu.d[r];
    let old = cpu.ccr();
    let res = 0u32
        .wrapping_sub(src)
        .wrapping_sub((old & ccr::X != 0) as u32);
    cpu.d[r] = res;
    extended_ccr(cpu, old, true, src, 0, res);
    Ok(Flow::Next)
}
