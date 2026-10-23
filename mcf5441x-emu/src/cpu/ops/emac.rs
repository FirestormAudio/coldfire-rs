//! The EMAC instructions (CFPRM ch. 6). The arithmetic lives in [`crate::cpu::emac`]; these handlers fetch the
//! operands, do the one memory access of MAC with load, and write registers only once nothing can fault.

use crate::cpu::emac::MacOp;
use crate::cpu::exception::vector::ILLEGAL;
use crate::cpu::{Cpu, Decoded, Flow, ea};
use crate::isa::Mode;
use crate::mem::{Fault, Memory, Size};

/// The multiply-accumulate fields of the extension word (CFPRM ch. 6 MAC): sz in bit 11, the scale factor in
/// 10-9, MSAC in 8, U/Lx in 7 and U/Ly in 6. The accumulator comes from operand slot 2.
fn mac_op(d: &Decoded) -> MacOp {
    let ext = d.operands[3];
    MacOp {
        acc: d.operands[2] as usize & 3,
        long: ext & 0x0800 != 0,
        sf: (ext >> 9) & 3,
        subtract: ext & 0x0100 != 0,
        upper_x: ext & 0x0080 != 0,
        upper_y: ext & 0x0040 != 0,
    }
}

/// Write register `n` (0-7 D0-D7, 8-15 A0-A7) whole.
fn set_reg(cpu: &mut Cpu, n: u32, v: u32) {
    if n < 8 {
        cpu.d[n as usize & 7] = v;
    } else {
        cpu.a[n as usize & 7] = v;
    }
}

/// `MAC.sz / MSAC.sz Ry,Rx{<<|>>},ACCx`. Extension bits 1-0, which select the EMAC_B dual-accumulate forms (CFPRM
/// ch. 6 MAAAC, MASAC, MSAAC, MSSAC) that RM ch. 5 does not list for this core, are ignored by MAC.L: 01, 10 and 11
/// take no exception and accumulate into ACCx alone, the other accumulators and MACSR untouched (measured on an
/// MCF54415).
pub fn mac(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    // guess: MAC.W and MSAC (not measured) with bits 1-0 nonzero take the illegal-instruction exception.
    let ext = d.operands[3];
    if ext & 3 != 0 && ext & 0x0900 != 0x0800 {
        return Ok(Flow::Exception(ILLEGAL));
    }
    let (rx, ry) = (cpu.reg(d.operands[0] as u8), cpu.reg(d.operands[1] as u8));
    cpu.emac.mac(mac_op(d), ry, rx);
    Ok(Flow::Next)
}

/// `MAC.sz / MSAC.sz Ry,Rx{<<|>>},<ea>y{&},Rw,ACCx`: the multiply-accumulate and, in parallel, `(<ea>y)` -> Rw as
/// a longword. With the mask bit (extension bit 5) the address is ANDed with MASK as RM 5.2.2 gives it: (An) and
/// (d16,An) mask the operand address; (An)+ uses An and leaves (An + 4) & MASK; -(An) uses and leaves
/// (An - 4) & MASK.
pub fn mac_load(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let ext = d.operands[3];
    let (rx, ry) = (cpu.reg(d.operands[0] as u8), cpu.reg(d.operands[1] as u8));
    let mask = if ext & 0x0020 != 0 {
        cpu.emac.mask
    } else {
        0xFFFF_FFFF
    };
    let r = d.ea.reg as usize & 7;
    let an = cpu.a[r];
    let (addr, new_an) = match d.ea.mode {
        Mode::PostInc => (an, Some(an.wrapping_add(4) & mask)),
        Mode::PreDec => {
            let a = an.wrapping_sub(4) & mask;
            (a, Some(a))
        }
        Mode::Disp => (an.wrapping_add(d.ea.value) & mask, None),
        // (An), the only other mode the decode table accepts.
        _ => (an & mask, None),
    };
    let v = mem.read(addr, Size::Long)?;
    if let Some(a) = new_an {
        cpu.a[r] = a;
    }
    cpu.emac.mac(mac_op(d), ry, rx);
    // Measured on an MCF54415 for (Ay)+: when Rw is the address register stepped, the loaded value wins, as for a
    // 68K MOVE.L (A0)+,A0. guess: -(Ay) does the same (not measured).
    set_reg(cpu, ext >> 12, v);
    Ok(Flow::Next)
}

/// `MOVE.L {Ry,#<data>},ACCx`.
pub fn move_to_acc(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let v = ea::load(cpu, mem, &d.ea, Size::Long)?;
    cpu.emac.load(d.operands[2] as usize & 3, v);
    Ok(Flow::Next)
}

/// MOVE.L ACCy,ACCx.
pub fn move_acc_acc(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    cpu.emac
        .copy(d.operands[1] as usize & 3, d.operands[2] as usize & 3);
    Ok(Flow::Next)
}

/// MOVE.L ACCy,Rx. The MACSR is not affected; the CFPRM page's closing "0 -> ACCx, ACCextx, MACSR\[PAVx\]" line
/// belongs to MOVCLR, as its condition-code box ("PAVx: not affected") shows.
pub fn move_from_acc(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let v = cpu.emac.store(d.operands[2] as usize & 3);
    set_reg(cpu, d.operands[1], v);
    Ok(Flow::Next)
}

/// MOVCLR.L ACCy,Rx.
pub fn movclr(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let v = cpu.emac.store_and_clear(d.operands[2] as usize & 3);
    set_reg(cpu, d.operands[1], v);
    Ok(Flow::Next)
}

/// `MOVE.L {Ry,#<data>},MACSR`.
pub fn move_to_macsr(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let v = ea::load(cpu, mem, &d.ea, Size::Long)?;
    cpu.emac.set_macsr(v);
    Ok(Flow::Next)
}

/// MOVE.L MACSR,Rx: bits 31-12 read as zero.
pub fn move_from_macsr(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let v = cpu.emac.macsr;
    set_reg(cpu, d.operands[1], v);
    Ok(Flow::Next)
}

/// MOVE.L MACSR,CCR: X cleared, N Z V from MACSR\[N Z V\] and C from MACSR\[EV\], which share their bit
/// positions.
pub fn move_macsr_ccr(cpu: &mut Cpu, _: &mut Memory, _: &Decoded) -> Result<Flow, Fault> {
    let v = cpu.emac.macsr & 0x0F;
    cpu.set_ccr(v as u8);
    Ok(Flow::Next)
}

/// `MOVE.L {Ry,#<data>},ACCext01`.
pub fn move_to_accext01(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let v = ea::load(cpu, mem, &d.ea, Size::Long)?;
    cpu.emac.set_accext(0, v);
    Ok(Flow::Next)
}

/// `MOVE.L {Ry,#<data>},ACCext23`.
pub fn move_to_accext23(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let v = ea::load(cpu, mem, &d.ea, Size::Long)?;
    cpu.emac.set_accext(1, v);
    Ok(Flow::Next)
}

/// MOVE.L ACCext01,Rx.
pub fn move_from_accext01(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let v = cpu.emac.accext(0);
    set_reg(cpu, d.operands[1], v);
    Ok(Flow::Next)
}

/// MOVE.L ACCext23,Rx.
pub fn move_from_accext23(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let v = cpu.emac.accext(1);
    set_reg(cpu, d.operands[1], v);
    Ok(Flow::Next)
}

/// `MOVE.L {Ry,#<data>},MASK`: the low word is loaded.
pub fn move_to_mask(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let v = ea::load(cpu, mem, &d.ea, Size::Long)?;
    cpu.emac.set_mask(v);
    Ok(Flow::Next)
}

/// MOVE.L MASK,Rx: bits 31-16 read as ones.
pub fn move_from_mask(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let v = cpu.emac.mask;
    set_reg(cpu, d.operands[1], v);
    Ok(Flow::Next)
}
