//! Integer arithmetic (CFPRM ch. 4).

use crate::cpu::ea::{self, Loc};
use crate::cpu::{Cpu, Decoded, Flow};
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

/// ADDQ.L #<data>,<ea>x: X N Z V C as ADD.
pub fn addq(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    quick_op(cpu, mem, d, false)
}

/// SUBQ.L #<data>,<ea>x: X N Z V C as SUB.
pub fn subq(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    quick_op(cpu, mem, d, true)
}
