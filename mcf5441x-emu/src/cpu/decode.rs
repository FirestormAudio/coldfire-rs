//! Instruction decoding: an opword and its extension words into a [`Decoded`] that handlers run without looking
//! at the instruction stream again.

use crate::cpu::ea::{Ea, sign_extend};
use crate::isa::{self, Ext, Form, Mode, Op};
use crate::mem::{Fault, Memory, Size};

/// One decoded instruction.
///
/// `operands` is filled the same way for every form, and handlers read the slots their `Op` uses:
///
/// | slot | contents                                                                                  |
/// |------|-------------------------------------------------------------------------------------------|
/// | 0    | the `R` field (register in bits 11-9)                                                     |
/// | 1    | the `r` field (register in the low bits)                                                  |
/// | 2    | the first other opword field (`q`, `c`, `d`, `v`, `a`, `k` or `p`); MOVEQ's is sign-extended |
/// | 3    | the last extension before the EA extension words (`w`, `ext` as read; `l`; `imm` by size)  |
///
/// `ea` is the `m`/`e` field and `ea2` the `M`/`E` field (MOVE's destination), [`Ea::NONE`] when absent.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Decoded {
    pub op: Op,
    /// The form's size; Long for unsized forms.
    pub size: Size,
    /// Length in bytes, opword included.
    pub len: u8,
    pub operands: [u32; 4],
    pub ea: Ea,
    pub ea2: Ea,
}

/// Why an instruction could not be decoded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecodeError {
    /// Fetching the opword or an extension word failed.
    Fault(Fault),
    /// The instruction takes this exception vector instead of executing: 4 illegal instruction, 10 and 11
    /// unimplemented line A and line F (CFPRM Table 11-1), 3 address error for a bad index extension word.
    Vector(u8),
}

impl From<Fault> for DecodeError {
    fn from(f: Fault) -> Self {
        DecodeError::Fault(f)
    }
}

/// The vector an opword no form describes takes (CFPRM Table 11-1).
fn undecodable(opword: u16) -> u8 {
    match opword >> 12 {
        0xA => 10,
        0xF => 11,
        _ => 4,
    }
}

/// Reads instruction-stream words in order.
struct Stream<'a> {
    mem: &'a mut Memory,
    at: u32,
}

impl Stream<'_> {
    fn word(&mut self) -> Result<u32, Fault> {
        let w = self.mem.read(self.at, Size::Word)?;
        self.at = self.at.wrapping_add(2);
        Ok(w)
    }

    fn long(&mut self) -> Result<u32, Fault> {
        Ok((self.word()? << 16) | self.word()?)
    }

    /// Immediate data of `size` (CFPRM Table 2-2: a byte is the low byte of its word).
    fn imm(&mut self, size: Size) -> Result<u32, Fault> {
        Ok(match size {
            Size::Byte => self.word()? & 0xFF,
            Size::Word => self.word()?,
            Size::Long => self.long()?,
        })
    }

    /// The extension words of the EA in `field` (mode, register), for an operand of `size`. `pc` is the opword's
    /// address; PC-relative modes use PC + 2 (CFPRM 2.2.8, 2.2.9).
    fn ea(&mut self, field: Option<(u32, u32)>, size: Size, pc: u32) -> Result<Ea, DecodeError> {
        let Some((m, r)) = field else {
            return Ok(Ea::NONE);
        };
        let mode = Mode::from_fields(m, r).ok_or(DecodeError::Vector(4))?;
        let mut ea = Ea {
            mode,
            reg: r as u8,
            ..Ea::NONE
        };
        match mode {
            Mode::Dn | Mode::An | Mode::Ind | Mode::PostInc | Mode::PreDec => {}
            Mode::Disp => ea.value = self.word()? as u16 as i16 as u32,
            Mode::AbsW => (ea.reg, ea.value) = (0, self.word()? as u16 as i16 as u32),
            Mode::AbsL => (ea.reg, ea.value) = (0, self.long()?),
            Mode::PcDisp => {
                (ea.reg, ea.value) = (
                    0,
                    pc.wrapping_add(2)
                        .wrapping_add(self.word()? as u16 as i16 as u32),
                )
            }
            Mode::Index | Mode::PcIndex => {
                let ext = self.word()?;
                // RM 3.3.4.2: a word index (W/L = 0), scale 8 or the full extension format (bit 8) is an address
                // error.
                if ext & 0x0800 == 0 || (ext >> 9) & 3 == 3 || ext & 0x0100 != 0 {
                    return Err(DecodeError::Vector(3));
                }
                ea.index = (ext >> 12) as u8;
                ea.scale = ((ext >> 9) & 3) as u8;
                let d8 = ext as u8 as i8 as u32;
                if mode == Mode::PcIndex {
                    (ea.reg, ea.value) = (0, pc.wrapping_add(2).wrapping_add(d8));
                } else {
                    ea.value = d8;
                }
            }
            Mode::Imm => (ea.reg, ea.value) = (0, self.imm(size)?),
        }
        Ok(ea)
    }
}

/// Decode the instruction at `pc`. Returns the form too, for its mnemonic.
pub fn decode(mem: &mut Memory, pc: u32) -> Result<(Decoded, &'static Form), DecodeError> {
    let mut s = Stream { mem, at: pc };
    let opword = s.word()? as u16;
    let form = isa::decode(opword).ok_or(DecodeError::Vector(undecodable(opword)))?;
    let size = form.size.unwrap_or(Size::Long);
    let f = &form.fields;

    let mut operands = [0u32; 4];
    operands[0] = f.get('R', opword).unwrap_or(0);
    operands[1] = f.get('r', opword).unwrap_or(0);
    operands[2] = f
        .iter()
        .find(|x| b"qcdvakp".contains(&x.letter))
        .map_or(0, |x| (opword as u32 >> x.shift) & ((1 << x.width) - 1));
    for ext in form.ext.iter() {
        operands[3] = match ext {
            Ext::Word | Ext::Ext => s.word()?,
            Ext::Long => s.long()?,
            Ext::Imm => s.imm(size)?,
        };
    }
    if form.id == Op::Moveq {
        operands[2] = sign_extend(Size::Byte, operands[2]);
    }

    let ea = s.ea(f.ea(opword), size, pc)?;
    let ea2 = s.ea(f.ea2(opword), size, pc)?;
    let len = s.at.wrapping_sub(pc);
    debug_assert_eq!(len, form.length(opword), "{} at {pc:#x}", form.mnemonic);
    Ok((
        Decoded {
            op: form.id,
            size,
            len: len as u8,
            operands,
            ea,
            ea2,
        },
        form,
    ))
}
