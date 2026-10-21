//! Program flow (CFPRM ch. 4): BRA, BSR, Bcc, JMP, JSR, RTS, LEA, PEA, LINK, UNLK, MOVEM and NOP.
//!
//! A jump, branch or return only sets PC; checking it for an odd address belongs to the instruction fetch.

use crate::cpu::ea;
use crate::cpu::{Cpu, Decoded, Flow};
use crate::mem::{Fault, Memory, Size};

/// Push a longword on the stack: A7 - 4 -> A7, then the value at (A7).
#[inline(always)]
fn push(cpu: &mut Cpu, mem: &mut Memory, value: u32) -> Result<(), Fault> {
    let sp = cpu.a[7].wrapping_sub(4);
    mem.write(sp, Size::Long, value)?;
    cpu.a[7] = sp;
    Ok(())
}

/// The address of the instruction after `d`.
#[inline(always)]
fn next_pc(cpu: &Cpu, d: &Decoded) -> u32 {
    cpu.pc.wrapping_add(d.len as u32)
}

/// The branch target: the displacement is relative to the opword's address + 2 (CFPRM ch. 4 Bcc).
#[inline(always)]
fn branch_target(cpu: &Cpu, d: &Decoded) -> u32 {
    cpu.pc.wrapping_add(2).wrapping_add(d.operands[3])
}

/// NOP: nothing happens.
pub fn nop(_: &mut Cpu, _: &mut Memory, _: &Decoded) -> Result<Flow, Fault> {
    Ok(Flow::Next)
}

/// BRA: PC + 2 + displacement -> PC.
pub fn bra(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    cpu.pc = branch_target(cpu, d);
    Ok(Flow::Jump)
}

/// BSR: push the address of the next instruction, then branch.
pub fn bsr(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    push(cpu, mem, next_pc(cpu, d))?;
    cpu.pc = branch_target(cpu, d);
    Ok(Flow::Jump)
}

/// Bcc: branch when condition `cc` holds on the current flags, otherwise continue.
pub fn bcc(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    if cpu.flags.condition(d.operands[2] as u8) {
        cpu.pc = branch_target(cpu, d);
        Ok(Flow::Jump)
    } else {
        Ok(Flow::Next)
    }
}

/// The address a control-mode `<ea>` designates. The decode table only lets control modes through.
#[inline(always)]
fn control_address(cpu: &mut Cpu, d: &Decoded) -> u32 {
    ea::address(cpu, &d.ea).unwrap_or(0)
}

/// JMP <ea>: the effective address -> PC.
pub fn jmp(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    cpu.pc = control_address(cpu, d);
    Ok(Flow::Jump)
}

/// JSR <ea>: push the address of the next instruction, then jump.
pub fn jsr(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let target = control_address(cpu, d);
    push(cpu, mem, next_pc(cpu, d))?;
    cpu.pc = target;
    Ok(Flow::Jump)
}

/// RTS: pop PC.
pub fn rts(cpu: &mut Cpu, mem: &mut Memory, _: &Decoded) -> Result<Flow, Fault> {
    let pc = mem.read(cpu.a[7], Size::Long)?;
    cpu.a[7] = cpu.a[7].wrapping_add(4);
    cpu.pc = pc;
    Ok(Flow::Jump)
}

/// LEA <ea>y,Ax: the effective address -> Ax; the condition codes are not affected.
pub fn lea(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    cpu.a[d.operands[0] as usize & 7] = control_address(cpu, d);
    Ok(Flow::Next)
}

/// PEA <ea>y: push the effective address; the condition codes are not affected.
pub fn pea(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let a = control_address(cpu, d);
    push(cpu, mem, a)?;
    Ok(Flow::Next)
}

/// LINK.W Ay,#<displacement>: push Ay, load the stack pointer into Ay, then add the sign-extended displacement
/// to the stack pointer.
pub fn link(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let r = d.operands[1] as usize & 7;
    let sp = cpu.a[7].wrapping_sub(4);
    mem.write(sp, Size::Long, cpu.a[r])?;
    cpu.a[7] = sp;
    cpu.a[r] = sp;
    cpu.a[7] = sp.wrapping_add(ea::sign_extend(Size::Word, d.operands[3]));
    Ok(Flow::Next)
}

/// UNLK Ax: Ax -> SP, pop Ax.
pub fn unlk(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let r = d.operands[1] as usize & 7;
    let sp = cpu.a[r];
    let v = mem.read(sp, Size::Long)?;
    cpu.a[7] = sp.wrapping_add(4);
    cpu.a[r] = v;
    Ok(Flow::Next)
}

/// The register `i` of the MOVEM mask: bit 0 is D0, bit 7 D7, bit 8 A0, bit 15 A7.
#[inline(always)]
fn mask_reg(cpu: &mut Cpu, i: u32) -> &mut u32 {
    if i < 8 {
        &mut cpu.d[i as usize]
    } else {
        &mut cpu.a[i as usize - 8]
    }
}

/// MOVEM.L #list,<ea>x: the selected registers, D0 first, to consecutive longwords from the effective address.
pub fn movem_regs_ea(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let mask = d.operands[3] & 0xFFFF;
    let mut at = control_address(cpu, d);
    for i in (0..16).filter(|i| mask >> i & 1 != 0) {
        let v = *mask_reg(cpu, i);
        mem.write(at, Size::Long, v)?;
        at = at.wrapping_add(4);
    }
    Ok(Flow::Next)
}

/// MOVEM.L <ea>y,#list: consecutive longwords from the effective address into the selected registers.
pub fn movem_ea_regs(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let mask = d.operands[3] & 0xFFFF;
    let mut at = control_address(cpu, d);
    for i in (0..16).filter(|i| mask >> i & 1 != 0) {
        *mask_reg(cpu, i) = mem.read(at, Size::Long)?;
        at = at.wrapping_add(4);
    }
    Ok(Flow::Next)
}
