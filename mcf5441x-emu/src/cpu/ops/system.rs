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

/// MOVE.W `<ea>`y,SR (Dy or #imm): every implemented bit; clearing S enters user mode.
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

/// MOVE.B `<ea>`y,CCR (Dy or #imm): the low byte; the upper byte of SR is not altered.
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

// Measured on an MCF54415: MOVEC to 0xC04 (RAMBAR0 in CFPRM ch. 8; RM Table 3-1 lists no register there) takes no
// exception and has no visible effect: 0xC04 reads 0 over the debug port before and after, and RAMBAR (0xC05) is
// unchanged.
/// MOVEC.L Ry,Rc: extension bit 15 and bits 14-12 give Ry (D0-D7, A0-A7), bits 11-0 the control register
/// (RM Table 3-1, the registers marked "Written with MOVEC"); a write to 0xC04 is accepted and changes nothing. The
/// manual leaves a write to any other Rc undefined (CFPRM ch. 8), so the run stops on it as unmodelled.
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
        0xC04 => {}
        0xC05 => cpu.rambar = v,
        _ => return Ok(Flow::Stop(Stop::Unmodelled)),
    }
    Ok(Flow::Next)
}

/// STRLDSR #imm: push the zero-filled SR, then load the immediate. In user mode it is an illegal instruction with
/// the STRLDSR as the stacked PC, not the privilege violation of the other supervisor instructions (measured on an
/// MCF54415; CFPRM ch. 8 gives the operation as "Else TRAP" with no vector). An immediate with S clear is a
/// privilege violation, taken before the push.
pub fn strldsr(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    if !cpu.supervisor() {
        return Ok(Flow::Exception(vector::ILLEGAL));
    }
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

// Measured on an MCF54415: an odd PC in the frame makes the RTE itself take the address error (FS 0100, the RTE
// as the stacked PC); its frame is not popped and the new frame is written below it, as for an odd RTS.
// Measured on an MCF54415: with a bad format and an odd PC together the format error wins; the format is checked
// first (RM 3.3.4.10).
/// RTE: check the frame's format (4-7, else a format error with the frame left alone, RM 3.3.4.10) and its PC (odd:
/// an address error with the frame left alone), load SR and PC from it, and pop 8 bytes plus the format's
/// alignment adjustment.
pub fn rte(cpu: &mut Cpu, mem: &mut Memory, _: &Decoded) -> Result<Flow, Fault> {
    supervisor!(cpu);
    let sp = cpu.a[7];
    let fv = mem.read(sp, Size::Long)?;
    let format = fv >> 28;
    if !(4..=7).contains(&format) {
        return Ok(Flow::Exception(vector::FORMAT_ERROR));
    }
    let pc = mem.read(sp.wrapping_add(4), Size::Long)?;
    if pc & 1 != 0 {
        return Ok(Flow::Exception(vector::ADDRESS_ERROR));
    }
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

/// WDDATA.sz `<ea>`y: the operand is read (for the DDATA pins) and dropped.
pub fn wddata(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    ea::load(cpu, mem, &d.ea, d.size)?;
    Ok(Flow::Next)
}

/// The first word of a WDEBUG operand that writes debug-module register DRc: `0x2C80 | DRc` (the WDMREG command,
/// RM 43.4.1.5.16 and Figure 43-52; CFPRM ch. 8 WDEBUG).
const WDMREG: u16 = 0x2C80;

// Measured on an MCF54415: a write of 0x00020000 (PCD) to CSR takes no exception and the program runs on.
// guess: the other bits here are taken from RM Table 43-7 (read-only from the debug port, or not writable by the
// processor), not measured. Every other bit (BKD, MAP, TRC, EMU, DDC, UHE, BTB, NPL, IPI, SSM, OTE, FDBG, DBGH and
// the reserved 19, 7 and 2) changes what the core does, or is undefined when set, so a write that sets one stops.
/// The CSR bits (RM Table 43-7) a program cannot observe without a debug port: BSTAT, FOF, TRG, HALT, BKPT and HRL
/// (31-20) are status the debug port reads, PCD (17) only quiets the PSTCLK and PSTDDATA pins, and IPW (16) can be
/// changed only from the debug port.
const CSR_UNOBSERVABLE: u32 = 0xFFF0_0000 | 1 << 17 | 1 << 16;

/// The name of debug-module register `drc` (RM Table 43-5).
fn debug_register(drc: u16) -> &'static str {
    match drc {
        0x00 => "CSR",
        0x01 => "XCSR",
        0x02 => "CSR2",
        0x05 => "BAAR",
        0x06 => "AATR",
        0x07 => "TDR",
        0x08 => "PBR0",
        0x09 => "PBMR",
        0x0A => "PBAC",
        0x0C => "ABHR",
        0x0D => "ABLR",
        0x0E => "DBR",
        0x0F => "DBMR",
        0x14 => "PBASID",
        0x16 => "AATR1",
        0x17 => "XTDR",
        0x18 => "PBR1",
        0x1A => "PBR2",
        0x1B => "PBR3",
        0x1C => "ABHR1",
        0x1D => "ABLR1",
        0x1E => "DBR1",
        0x1F => "DBMR1",
        _ => "reserved",
    }
}

// guess (a misaligned operand): CFPRM ch. 8 calls the operation undefined; the run stops on it, before any access.
/// WDEBUG.L `<ea>`y: reads the debug command from the two longwords at the effective address (CFPRM ch. 8: the
/// command word `0x2C80 | DRc`, then the 32-bit data) and writes debug-module register DRc. With no debug port the
/// debug module is not modelled: a write to CSR that sets only bits a program cannot observe
/// (`CSR_UNOBSERVABLE`; the observable ones are all clear out of reset and stay so) changes nothing; any other
/// write stops the run with `Unmodelled` and a detail naming the register and the value.
pub fn wdebug(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    supervisor!(cpu);
    let addr = ea::address(cpu, &d.ea).unwrap_or(0);
    if addr & 3 != 0 {
        mem.refused = Some(format!(
            "WDEBUG: operand at {addr:#010x} is not longword aligned"
        ));
        return Err(Fault::Device(Stop::Unmodelled));
    }
    let first = mem.read(addr, Size::Long)?;
    let second = mem.read(addr.wrapping_add(4), Size::Long)?;
    let command = (first >> 16) as u16;
    let data = first << 16 | second >> 16;
    let detail = if command & !0x1F != WDMREG {
        format!("WDEBUG: command word {command:#06x} is not a register write")
    } else {
        let drc = command & 0x1F;
        if drc == 0 && data & !CSR_UNOBSERVABLE == 0 {
            return Ok(Flow::Next);
        }
        format!(
            "WDEBUG: write of {data:#010x} to debug register {drc:#04x} ({})",
            debug_register(drc)
        )
    };
    mem.refused = Some(detail);
    Err(Fault::Device(Stop::Unmodelled))
}

/// CPUSHL and INTOUCH: the caches are not modelled, so pushing a line or prefetching one changes nothing a
/// program can see.
pub fn cache_hint(cpu: &mut Cpu, _: &mut Memory, _: &Decoded) -> Result<Flow, Fault> {
    supervisor!(cpu);
    Ok(Flow::Next)
}

// FSAVE takes line F in user mode too, not a privilege violation: with no FPU the opword is unimplemented
// (measured on an MCF54415). guess: FRESTORE, not measured, does the same.
/// An FPU instruction: the chip has no FPU (RM Table 3-9, D0\[FPU\] = 0), so it is an unimplemented line-F opcode.
pub fn line_f(_: &mut Cpu, _: &mut Memory, _: &Decoded) -> Result<Flow, Fault> {
    Ok(Flow::Exception(vector::LINE_F))
}
