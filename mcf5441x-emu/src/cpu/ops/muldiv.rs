//! MULS, MULU, DIVS, DIVU, REMS and REMU (CFPRM ch. 4). V and C are cleared by the multiplies (unlike the 68K
//! family). A divide by zero raises vector 5 and changes nothing; an overflowing quotient leaves the destination
//! alone and sets V.

use crate::cpu::ccr::V;
use crate::cpu::ea;
use crate::cpu::{Cpu, Decoded, Flow};
use crate::mem::{Fault, Memory, Size};

/// The divide-by-zero exception vector (CFPRM Table 11-1).
const DIVIDE_BY_ZERO: u8 = 5;

/// MULS.W and MULU.W `<ea>y,Dx`: 16 x 16 to 32 from the low words.
#[inline(always)]
fn mul_w(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded, signed: bool) -> Result<Flow, Fault> {
    let src = ea::load(cpu, mem, &d.ea, Size::Word)?;
    let r = d.operands[0] as usize & 7;
    let dst = cpu.d[r] & 0xFFFF;
    let res = if signed {
        ((src as u16 as i16 as i32) * (dst as u16 as i16 as i32)) as u32
    } else {
        src * dst
    };
    cpu.d[r] = res;
    cpu.flags.logic(Size::Long, res);
    Ok(Flow::Next)
}

pub fn muls_w(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    mul_w(cpu, mem, d, true)
}
pub fn mulu_w(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    mul_w(cpu, mem, d, false)
}

/// MULS.L and MULU.L `<ea>y,Dx`: the low 32 bits of the product, which are the same for both signednesses.
pub fn mul_l(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let src = ea::load(cpu, mem, &d.ea, Size::Long)?;
    let r = (d.operands[3] >> 12) as usize & 7;
    let res = cpu.d[r].wrapping_mul(src);
    cpu.d[r] = res;
    cpu.flags.logic(Size::Long, res);
    Ok(Flow::Next)
}

/// Overflow: V set, N Z C clear, X unchanged.
fn overflow(cpu: &mut Cpu) -> Result<Flow, Fault> {
    cpu.flags.set_nzvc(V);
    Ok(Flow::Next)
}

/// DIVS.W and DIVU.W `<ea>y,Dx`: 32-bit Dx / 16-bit source; remainder in the upper word, quotient in the lower.
#[inline(always)]
fn div_w(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded, signed: bool) -> Result<Flow, Fault> {
    let before = cpu.a;
    let src = ea::load(cpu, mem, &d.ea, Size::Word)?;
    if src == 0 {
        cpu.a = before;
        return Ok(cpu.raise(DIVIDE_BY_ZERO));
    }
    let r = d.operands[0] as usize & 7;
    let dividend = cpu.d[r];
    let (q, rem, fits) = if signed {
        let (n, s) = (dividend as i32 as i64, src as u16 as i16 as i64);
        let q = n / s;
        (q as u32, (n % s) as u32, (-0x8000..0x8000).contains(&q))
    } else {
        let q = dividend / src;
        (q, dividend % src, q <= 0xFFFF)
    };
    if !fits {
        return overflow(cpu);
    }
    cpu.d[r] = (rem << 16) | (q & 0xFFFF);
    cpu.flags.logic(Size::Word, q & 0xFFFF);
    Ok(Flow::Next)
}

pub fn divs_w(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    div_w(cpu, mem, d, true)
}
pub fn divu_w(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    div_w(cpu, mem, d, false)
}

/// DIVS.L, DIVU.L, REMS.L and REMU.L `<ea>y,Dw:Dx`, told apart by extension bit 11 (signed) and by whether Dw
/// is Dx (quotient into Dx) or another register (remainder into Dw, Dx unchanged). N and Z follow the quotient
/// in both forms (CFPRM ch. 4 REMS, REMU condition codes).
pub fn div_l(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let before = cpu.a;
    let src = ea::load(cpu, mem, &d.ea, Size::Long)?;
    let ext = d.operands[3];
    let (x, w) = ((ext >> 12) as usize & 7, ext as usize & 7);
    if src == 0 {
        // The frame points at the divide, so the (Ay)+ / -(Ay) update is undone.
        cpu.a = before;
        return Ok(cpu.raise(DIVIDE_BY_ZERO));
    }
    let dividend = cpu.d[x];
    let (q, rem) = if ext & 0x0800 != 0 {
        let (n, s) = (dividend as i32 as i64, src as i32 as i64);
        // The one overflow, i32::MIN / -1, does not fit in 32 bits.
        if n / s > i32::MAX as i64 {
            return overflow(cpu);
        }
        ((n / s) as u32, (n % s) as u32)
    } else {
        (dividend / src, dividend % src)
    };
    if w == x {
        cpu.d[x] = q;
    } else {
        cpu.d[w] = rem;
    }
    cpu.flags.logic(Size::Long, q);
    Ok(Flow::Next)
}
