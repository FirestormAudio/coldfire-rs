//! MOVE, MOVEA, MOVEQ, MVS and MVZ (CFPRM ch. 4).

use crate::cpu::ea::{self, sign_extend};
use crate::cpu::{Cpu, Decoded, Flow};
use crate::mem::{Fault, Memory, Size};

/// MOVE.sz `<ea>`y,`<ea>`x: N and Z from the moved value, V and C cleared, X unchanged. The source is evaluated (and
/// its (An)+/-(An) applied) before the destination.
pub fn move_(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let v = ea::load(cpu, mem, &d.ea, d.size)?;
    ea::store(cpu, mem, &d.ea2, d.size, v)?;
    cpu.flags.logic(d.size, v);
    Ok(Flow::Next)
}

/// MOVEA.sz `<ea>`y,Ax: a word is sign-extended; the condition codes are not affected.
pub fn movea(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let v = ea::load(cpu, mem, &d.ea, d.size)?;
    cpu.a[d.operands[0] as usize & 7] = sign_extend(d.size, v);
    Ok(Flow::Next)
}

/// MOVEQ.L #`<data>`,Dx: the sign-extended byte; N and Z set, V and C cleared, X unchanged.
pub fn moveq(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let v = d.operands[2];
    cpu.d[d.operands[0] as usize & 7] = v;
    cpu.flags.logic(Size::Long, v);
    Ok(Flow::Next)
}

/// MVS.sz `<ea>`y,Dx: sign-extend to a longword; N and Z from the result, V and C cleared, X unchanged.
pub fn mvs(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let v = sign_extend(d.size, ea::load(cpu, mem, &d.ea, d.size)?);
    cpu.d[d.operands[0] as usize & 7] = v;
    cpu.flags.logic(Size::Long, v);
    Ok(Flow::Next)
}

/// MVZ.sz `<ea>`y,Dx: zero-extend to a longword; N cleared, Z from the result, V and C cleared, X unchanged.
pub fn mvz(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let v = ea::load(cpu, mem, &d.ea, d.size)?;
    cpu.d[d.operands[0] as usize & 7] = v;
    cpu.flags.logic(Size::Long, v);
    Ok(Flow::Next)
}
