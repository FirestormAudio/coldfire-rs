//! The status register, the stack pointers and the control registers (CFPRM ch. 8 and the CCR moves of ch. 4),
//! TRAP, ILLEGAL, RTE, STOP, HALT, the cache and debug hints, and the FPU opwords this chip lacks.
//!
//! A supervisor instruction in user mode takes a privilege violation (vector 8) before doing anything.

use crate::cpu::exception::{VBR_MASK, vector};
use crate::cpu::{Cpu, Decoded, Flow, SR_S, ea};
use crate::mem::{Fault, Memory, Size};
use crate::stop::Stop;

/// The privilege violation a supervisor instruction takes in user mode.
macro_rules! supervisor {
    ($cpu:expr) => {
        if !$cpu.supervisor() {
            return Ok(Flow::Exception(vector::PRIVILEGE_VIOLATION));
        }
    };
}

/// Write the low word of Dx, keeping its upper word.
#[inline(always)]
fn set_low_word(cpu: &mut Cpu, r: u32, v: u16) {
    let d = &mut cpu.d[r as usize & 7];
    *d = (*d & 0xFFFF_0000) | v as u32;
}

/// MOVE.W SR,Dx: the unimplemented bits read as zero.
pub fn move_from_sr(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    supervisor!(cpu);
    set_low_word(cpu, d.operands[1], cpu.sr());
    Ok(Flow::Next)
}

/// MOVE.W <ea>y,SR (Dy or #imm): every implemented bit; clearing S enters user mode.
pub fn move_to_sr(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    supervisor!(cpu);
    let v = ea::load(cpu, mem, &d.ea, Size::Word)?;
    cpu.set_sr(v as u16);
    // A lower mask may let a held interrupt request in.
    mem.resample();
    Ok(Flow::Next)
}

/// MOVE.W CCR,Dx: the CCR zero-extended to a word.
pub fn move_from_ccr(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    set_low_word(cpu, d.operands[1], cpu.ccr() as u16);
    Ok(Flow::Next)
}

/// MOVE.B <ea>y,CCR (Dy or #imm): the low byte; the upper byte of SR is not altered.
pub fn move_to_ccr(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let v = ea::load(cpu, mem, &d.ea, Size::Byte)?;
    cpu.set_ccr(v as u8);
    Ok(Flow::Next)
}

// guess (MOVE to/from USP): CFPRM ch. 8 says they execute correctly when CACR[EUSP] is set and does not say what
// they do when it is clear; here they reach the OTHER_A7 register either way.
/// MOVE.L Ay,USP.
pub fn move_to_usp(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    supervisor!(cpu);
    cpu.other_sp = cpu.a[d.operands[1] as usize & 7];
    Ok(Flow::Next)
}

/// MOVE.L USP,Ax.
pub fn move_from_usp(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    supervisor!(cpu);
    cpu.a[d.operands[1] as usize & 7] = cpu.other_sp;
    Ok(Flow::Next)
}

/// MOVEC.L Ry,Rc: extension bit 15 and bits 14-12 give Ry (D0-D7, A0-A7), bits 11-0 the control register
/// (RM Table 3-1, the registers marked "Written with MOVEC"). The manual leaves a write to any other Rc undefined
/// (CFPRM ch. 8), so the run stops on it as unmodelled.
pub fn movec(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    supervisor!(cpu);
    let ext = d.operands[3];
    let v = cpu.reg((ext >> 12) as u8 & 15);
    match ext & 0xFFF {
        0x002 => cpu.cacr = v,
        0x003 => cpu.asid = v & 0xFF,
        rc @ 0x004..=0x007 => cpu.acr[rc as usize - 4] = v,
        0x008 => cpu.mmubar = v,
        0x009 => cpu.rgpiobar = v,
        rc @ 0x00C..=0x00F => cpu.acr[rc as usize - 8] = v,
        0x801 => cpu.vbr = v & VBR_MASK,
        0xC05 => cpu.rambar = v,
        _ => return Ok(Flow::Stop(Stop::Unmodelled)),
    }
    Ok(Flow::Next)
}

// guess (STRLDSR in user mode): CFPRM ch. 8 gives the operation as "Else TRAP" with no vector; this takes the
// privilege violation of the other supervisor instructions.
/// STRLDSR #imm: push the zero-filled SR, then load the immediate. An immediate with S clear is a privilege
/// violation, taken before the push.
pub fn strldsr(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    supervisor!(cpu);
    let imm = d.operands[3] as u16;
    if imm & SR_S == 0 {
        return Ok(Flow::Exception(vector::PRIVILEGE_VIOLATION));
    }
    let sp = cpu.a[7].wrapping_sub(4);
    mem.write(sp, Size::Long, cpu.sr() as u32)?;
    cpu.a[7] = sp;
    cpu.set_sr(imm);
    mem.resample();
    Ok(Flow::Next)
}

/// TRAP #n: vector 32 + n, with the next instruction's address in the frame.
pub fn trap(_: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    Ok(Flow::Exception(vector::TRAP_0 + (d.operands[2] as u8 & 15)))
}

/// ILLEGAL: the illegal-instruction exception.
pub fn illegal(_: &mut Cpu, _: &mut Memory, _: &Decoded) -> Result<Flow, Fault> {
    Ok(Flow::Exception(vector::ILLEGAL))
}

/// RTE: check the frame's format (4-7, else a format error with the frame left alone, RM 3.3.4.10), load SR
/// and PC from it, and pop 8 bytes plus the format's alignment adjustment.
pub fn rte(cpu: &mut Cpu, mem: &mut Memory, _: &Decoded) -> Result<Flow, Fault> {
    supervisor!(cpu);
    let sp = cpu.a[7];
    let fv = mem.read(sp, Size::Long)?;
    let format = fv >> 28;
    if !(4..=7).contains(&format) {
        return Ok(Flow::Exception(vector::FORMAT_ERROR));
    }
    let pc = mem.read(sp.wrapping_add(4), Size::Long)?;
    cpu.a[7] = sp.wrapping_add(8 + (format - 4));
    cpu.set_sr(fv as u16);
    cpu.pc = pc;
    mem.resample();
    Ok(Flow::Jump)
}

/// STOP #imm: load SR and wait for an interrupt with PC past the STOP. An immediate with S clear is a privilege
/// violation.
pub fn stop(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    supervisor!(cpu);
    let imm = d.operands[3] as u16;
    if imm & SR_S == 0 {
        return Ok(Flow::Exception(vector::PRIVILEGE_VIOLATION));
    }
    cpu.set_sr(imm);
    Ok(Flow::Wait)
}

// guess (HALT): a GO command from the debug module resumes at the next instruction (CFPRM ch. 8), so the run
// stops with HALT complete and PC past it. CSR[UHE], which lets user mode halt, belongs to the debug module and
// is not modelled.
/// HALT: the run stops with `Halt("halt")`.
pub fn halt(cpu: &mut Cpu, _: &mut Memory, _: &Decoded) -> Result<Flow, Fault> {
    supervisor!(cpu);
    Ok(Flow::StopAfter(Stop::Halt("halt")))
}

/// PULSE: no operation (it signals on the PST pins).
pub fn pulse(_: &mut Cpu, _: &mut Memory, _: &Decoded) -> Result<Flow, Fault> {
    Ok(Flow::Next)
}

/// WDDATA.sz <ea>y: the operand is read (for the DDATA pins) and dropped.
pub fn wddata(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    ea::load(cpu, mem, &d.ea, d.size)?;
    Ok(Flow::Next)
}

/// WDEBUG.L <ea>y: writes a debug-module register, which this core does not model.
pub fn wdebug(cpu: &mut Cpu, _: &mut Memory, _: &Decoded) -> Result<Flow, Fault> {
    supervisor!(cpu);
    Ok(Flow::Stop(Stop::Unmodelled))
}

/// CPUSHL and INTOUCH: the caches are not modelled, so pushing a line or prefetching one changes nothing a
/// program can see.
pub fn cache_hint(cpu: &mut Cpu, _: &mut Memory, _: &Decoded) -> Result<Flow, Fault> {
    supervisor!(cpu);
    Ok(Flow::Next)
}

// guess (FSAVE, FRESTORE in user mode): with no FPU the opword is unimplemented, so they take line F before any
// privilege check.
/// An FPU instruction: the chip has no FPU (RM Table 3-9, D0\[FPU\] = 0), so it is an unimplemented line-F opcode.
pub fn line_f(_: &mut Cpu, _: &mut Memory, _: &Decoded) -> Result<Flow, Fault> {
    Ok(Flow::Exception(vector::LINE_F))
}
