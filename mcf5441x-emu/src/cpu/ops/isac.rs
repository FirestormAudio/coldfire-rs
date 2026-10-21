//! The ISA_B and ISA_C additions (CFPRM ch. 4): MOV3Q, SATS, FF1, BITREV, BYTEREV and TPF. MVS and MVZ are with
//! the moves.

use crate::cpu::ccr::V;
use crate::cpu::ea;
use crate::cpu::{Cpu, Decoded, Flow};
use crate::mem::{Fault, Memory, Size};

/// MOV3Q.L #<data>,<ea>x: data is -1 or 1 to 7 (the field value 0 means -1), sign-extended; N and Z from the
/// result, V and C cleared, X unchanged.
pub fn mov3q(cpu: &mut Cpu, mem: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let v = match d.operands[2] {
        0 => 0xFFFF_FFFF,
        q => q,
    };
    ea::store(cpu, mem, &d.ea, Size::Long, v)?;
    cpu.flags.logic(Size::Long, v);
    Ok(Flow::Next)
}

// guess: the SATS page has no condition-code box. "The condition codes are set according to the result" is read
// as N and Z from the result, V and C cleared and X unchanged, as for MOVE.
/// SATS.L Dx: with V set, an operand that reads positive becomes 0x80000000 and one that reads negative becomes
/// 0x7FFFFFFF (the sign bit is the wrong way round after an overflow); with V clear it is left alone.
pub fn sats(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let r = d.operands[1] as usize & 7;
    if cpu.flags.ccr() & V != 0 {
        cpu.d[r] = if cpu.d[r] & 0x8000_0000 == 0 {
            0x8000_0000
        } else {
            0x7FFF_FFFF
        };
    }
    cpu.flags.logic(Size::Long, cpu.d[r]);
    Ok(Flow::Next)
}

/// FF1.L Dx: the offset of the first set bit counted from bit 31, 32 for zero. N is the msb of the source and Z
/// is set for a zero source, which is how [`Flags::logic`](crate::cpu::Flags) reads the source value; V and C
/// are cleared.
pub fn ff1(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let r = d.operands[1] as usize & 7;
    let src = cpu.d[r];
    cpu.d[r] = src.leading_zeros();
    cpu.flags.logic(Size::Long, src);
    Ok(Flow::Next)
}

/// BITREV.L Dx: the condition codes are not affected.
pub fn bitrev(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let r = d.operands[1] as usize & 7;
    cpu.d[r] = cpu.d[r].reverse_bits();
    Ok(Flow::Next)
}

/// BYTEREV.L Dx: the condition codes are not affected.
pub fn byterev(cpu: &mut Cpu, _: &mut Memory, d: &Decoded) -> Result<Flow, Fault> {
    let r = d.operands[1] as usize & 7;
    cpu.d[r] = cpu.d[r].swap_bytes();
    Ok(Flow::Next)
}

/// TPF, TPF.W #<data>, TPF.L #<data>: no operation; the decoder has already counted the extension words.
pub fn tpf(_: &mut Cpu, _: &mut Memory, _: &Decoded) -> Result<Flow, Fault> {
    Ok(Flow::Next)
}
