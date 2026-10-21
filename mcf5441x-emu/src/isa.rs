//! The V4e instruction set: forms parsed from `isa/v4e.isa` and a 65,536-entry opword decode table.
//!
//! The description is the source of truth (CFPRM ch. 4 to 9, RM ch. 3, 5 and 26); this module only parses it,
//! checks it and indexes it. See the header of `isa/v4e.isa` for the line format.

use std::fmt;
use std::sync::OnceLock;

use crate::mem::Size;

/// The instruction description, one form per line.
pub const SOURCE: &str = include_str!("../isa/v4e.isa");

/// The ISA revision or optional unit a form belongs to (CFPRM Table 3-16; RM Table 3-9).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Isa {
    /// ISA_A.
    A,
    /// ISA_B additions.
    B,
    /// ISA_C (and ISA_A+) additions.
    C,
    /// The hardware divider: DIVS, DIVU, REMS, REMU.
    Div,
    /// The enhanced multiply-accumulate unit (CFPRM ch. 6).
    Emac,
    /// The cryptographic acceleration unit's coprocessor instructions (RM ch. 26).
    Cau,
    /// Floating-point unit instructions; this chip has no FPU.
    Fpu,
    /// The MMU's supervisor instructions: MOVE to/from USP (RM 4.3.9).
    Mmu,
}

impl Isa {
    fn parse(s: &str) -> Option<Isa> {
        Some(match s {
            "A" => Isa::A,
            "B" => Isa::B,
            "C" => Isa::C,
            "Div" => Isa::Div,
            "Emac" => Isa::Emac,
            "Cau" => Isa::Cau,
            "Fpu" => Isa::Fpu,
            "Mmu" => Isa::Mmu,
            _ => return None,
        })
    }
}

macro_rules! ops {
    ($($(#[$doc:meta])* $name:ident,)*) => {
        /// One instruction form: a mnemonic with one operand shape. Sized variants of a shape share an `Op`
        /// and differ in [`Form::size`].
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
        pub enum Op {
            $($(#[$doc])* $name,)*
        }

        impl Op {
            /// Every variant, in declaration order.
            pub const ALL: &'static [Op] = &[$(Op::$name,)*];

            /// The variant named `name` in the description.
            pub fn from_name(name: &str) -> Option<Op> {
                match name {
                    $(stringify!($name) => Some(Op::$name),)*
                    _ => None,
                }
            }
        }
    };
}

ops! {
    // line 0
    /// ORI.L #imm,Dx
    Ori,
    /// BITREV.L Dx
    Bitrev,
    /// ANDI.L #imm,Dx
    Andi,
    /// BYTEREV.L Dx
    Byterev,
    /// SUBI.L #imm,Dx
    Subi,
    /// FF1.L Dx
    Ff1,
    /// ADDI.L #imm,Dx
    Addi,
    /// EORI.L #imm,Dx
    Eori,
    /// CMPI.sz #imm,Dx
    Cmpi,
    /// BTST #n,`<ea>`
    BtstImmEa,
    /// BCHG #n,`<ea>`
    BchgImmEa,
    /// BCLR #n,`<ea>`
    BclrImmEa,
    /// BSET #n,`<ea>`
    BsetImmEa,
    /// BTST Dy,`<ea>`
    BtstDnEa,
    /// BCHG Dy,`<ea>`
    BchgDnEa,
    /// BCLR Dy,`<ea>`
    BclrDnEa,
    /// BSET Dy,`<ea>`
    BsetDnEa,
    // lines 1-3
    /// MOVE.sz `<ea>`y,`<ea>`x
    Move,
    /// MOVEA.sz `<ea>`y,Ax
    Movea,
    // line 4
    /// NEGX.L Dx
    Negx,
    /// MOVE.W SR,Dx
    MoveFromSr,
    /// STRLDSR #imm
    Strldsr,
    /// LEA `<ea>`y,Ax
    Lea,
    /// CLR.sz `<ea>`x
    Clr,
    /// MOVE.W CCR,Dx
    MoveFromCcr,
    /// NEG.L Dx
    Neg,
    /// MOVE.B `<ea>`y,CCR
    MoveToCcr,
    /// NOT.L Dx
    Not,
    /// MOVE.W `<ea>`y,SR
    MoveToSr,
    /// SWAP.W Dx
    Swap,
    /// PEA `<ea>`y
    Pea,
    /// EXT.W / EXT.L Dx
    Ext,
    /// EXTB.L Dx
    Extb,
    /// MOVEM.L #list,`<ea>`x
    MovemRegsEa,
    /// MOVEM.L `<ea>`y,#list
    MovemEaRegs,
    /// TST.sz `<ea>`y
    Tst,
    /// TAS.B `<ea>`x
    Tas,
    /// HALT
    Halt,
    /// PULSE
    Pulse,
    /// ILLEGAL
    Illegal,
    /// MULS.L / MULU.L `<ea>`y,Dx (extension bit 11 selects signed)
    MulL,
    /// DIVS.L / DIVU.L / REMS.L / REMU.L `<ea>`y,Dw:Dx (extension word selects)
    DivL,
    /// SATS.L Dx
    Sats,
    /// TRAP #vector
    Trap,
    /// LINK.W Ay,#d16
    Link,
    /// UNLK Ax
    Unlk,
    /// MOVE.L Ay,USP
    MoveToUsp,
    /// MOVE.L USP,Ax
    MoveFromUsp,
    /// NOP
    Nop,
    /// STOP #imm
    Stop,
    /// RTE
    Rte,
    /// RTS
    Rts,
    /// MOVEC.L Ry,Rc
    Movec,
    /// JSR `<ea>`y
    Jsr,
    /// JMP `<ea>`y
    Jmp,
    // line 5
    /// ADDQ.L #q,`<ea>`x
    Addq,
    /// SUBQ.L #q,`<ea>`x
    Subq,
    /// Scc.B Dx
    Scc,
    /// TPF, TPF.W #imm, TPF.L #imm
    Tpf,
    // line 6
    /// BRA.sz
    Bra,
    /// BSR.sz
    Bsr,
    /// Bcc.sz
    Bcc,
    // line 7
    /// MOVEQ.L #imm,Dx
    Moveq,
    /// MVS.sz `<ea>`y,Dx
    Mvs,
    /// MVZ.sz `<ea>`y,Dx
    Mvz,
    // line 8
    /// OR.L `<ea>`y,Dx
    OrEaDn,
    /// OR.L Dy,`<ea>`x
    OrDnEa,
    /// DIVU.W `<ea>`y,Dx
    DivuW,
    /// DIVS.W `<ea>`y,Dx
    DivsW,
    // line 9
    /// SUB.L `<ea>`y,Dx
    SubEaDn,
    /// SUB.L Dy,`<ea>`x
    SubDnEa,
    /// SUBX.L Dy,Dx
    Subx,
    /// SUBA.L `<ea>`y,Ax
    Suba,
    // line A
    /// MAC / MSAC Ry,Rx,ACCx (extension bit 8 selects MSAC)
    Mac,
    /// MAC / MSAC Ry,Rx,`<ea>`y,Rw,ACCx
    MacLoad,
    /// MOVE.L `<ea>`y,ACCx
    MoveToAcc,
    /// MOVE.L ACCy,ACCx
    MoveAccAcc,
    /// MOVE.L ACCy,Rx
    MoveFromAcc,
    /// MOVCLR.L ACCy,Rx
    Movclr,
    /// MOVE.L `<ea>`y,MACSR
    MoveToMacsr,
    /// MOVE.L MACSR,Rx
    MoveFromMacsr,
    /// MOVE.L MACSR,CCR
    MoveMacsrCcr,
    /// MOVE.L `<ea>`y,ACCext01
    MoveToAccext01,
    /// MOVE.L ACCext01,Rx
    MoveFromAccext01,
    /// MOVE.L `<ea>`y,MASK
    MoveToMask,
    /// MOVE.L MASK,Rx
    MoveFromMask,
    /// MOVE.L `<ea>`y,ACCext23
    MoveToAccext23,
    /// MOVE.L ACCext23,Rx
    MoveFromAccext23,
    /// MOV3Q.L #q,`<ea>`x
    Mov3q,
    // line B
    /// CMP.sz `<ea>`y,Dx
    Cmp,
    /// CMPA.sz `<ea>`y,Ax
    Cmpa,
    /// EOR.L Dy,`<ea>`x
    Eor,
    // line C
    /// AND.L `<ea>`y,Dx
    AndEaDn,
    /// AND.L Dy,`<ea>`x
    AndDnEa,
    /// MULU.W `<ea>`y,Dx
    MuluW,
    /// MULS.W `<ea>`y,Dx
    MulsW,
    // line D
    /// ADD.L `<ea>`y,Dx
    AddEaDn,
    /// ADD.L Dy,`<ea>`x
    AddDnEa,
    /// ADDX.L Dy,Dx
    Addx,
    /// ADDA.L `<ea>`y,Ax
    Adda,
    // line E
    /// ASR.L #q,Dx
    AsrImm,
    /// ASR.L Dy,Dx
    AsrReg,
    /// ASL.L #q,Dx
    AslImm,
    /// ASL.L Dy,Dx
    AslReg,
    /// LSR.L #q,Dx
    LsrImm,
    /// LSR.L Dy,Dx
    LsrReg,
    /// LSL.L #q,Dx
    LslImm,
    /// LSL.L Dy,Dx
    LslReg,
    // line F
    /// An FPU general instruction (opword 0xF200 | ea plus a command word)
    FpuOp,
    /// FBcc.sz
    Fbcc,
    /// FSAVE `<ea>`x
    Fsave,
    /// FRESTORE `<ea>`y
    Frestore,
    /// INTOUCH (Ay)
    Intouch,
    /// CPUSHL cache,(Ax)
    Cpushl,
    /// WDDATA.sz `<ea>`y
    Wddata,
    /// WDEBUG.L `<ea>`y
    Wdebug,
    /// CAU coprocessor load (cp0ld.l)
    Cp0ld,
    /// CAU coprocessor store (cp0st.l)
    Cp0st,
}

/// An effective addressing mode (CFPRM Table 2-3).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Mode {
    /// Dn
    Dn,
    /// An
    An,
    /// (An)
    Ind,
    /// (An)+
    PostInc,
    /// -(An)
    PreDec,
    /// (d16,An)
    Disp,
    /// (d8,An,Xi*SF)
    Index,
    /// (xxx).W
    AbsW,
    /// (xxx).L
    AbsL,
    /// (d16,PC)
    PcDisp,
    /// (d8,PC,Xi*SF)
    PcIndex,
    /// #`<data>`
    Imm,
}

impl Mode {
    const ALL: [Mode; 12] = [
        Mode::Dn,
        Mode::An,
        Mode::Ind,
        Mode::PostInc,
        Mode::PreDec,
        Mode::Disp,
        Mode::Index,
        Mode::AbsW,
        Mode::AbsL,
        Mode::PcDisp,
        Mode::PcIndex,
        Mode::Imm,
    ];

    /// The mode encoded by a 3-bit mode field and 3-bit register field; `None` for the reserved mode-7 registers.
    pub fn from_fields(mode: u32, reg: u32) -> Option<Mode> {
        Some(match (mode & 7, reg & 7) {
            (0, _) => Mode::Dn,
            (1, _) => Mode::An,
            (2, _) => Mode::Ind,
            (3, _) => Mode::PostInc,
            (4, _) => Mode::PreDec,
            (5, _) => Mode::Disp,
            (6, _) => Mode::Index,
            (7, 0) => Mode::AbsW,
            (7, 1) => Mode::AbsL,
            (7, 2) => Mode::PcDisp,
            (7, 3) => Mode::PcIndex,
            (7, 4) => Mode::Imm,
            _ => return None,
        })
    }

    /// Extension words this mode adds (CFPRM ch. 2: brief-format index words only); `None` for an unsized #imm.
    pub fn ext_words(self, size: Option<Size>) -> Option<u32> {
        Some(match self {
            Mode::Dn | Mode::An | Mode::Ind | Mode::PostInc | Mode::PreDec => 0,
            Mode::Disp | Mode::Index | Mode::AbsW | Mode::PcDisp | Mode::PcIndex => 1,
            Mode::AbsL => 2,
            Mode::Imm => match size? {
                Size::Byte | Size::Word => 1,
                Size::Long => 2,
            },
        })
    }

    fn token(self) -> &'static str {
        match self {
            Mode::Dn => "Dn",
            Mode::An => "An",
            Mode::Ind => "(An)",
            Mode::PostInc => "(An)+",
            Mode::PreDec => "-(An)",
            Mode::Disp => "(d16,An)",
            Mode::Index => "(d8,An,Xi)",
            Mode::AbsW => "abs.W",
            Mode::AbsL => "abs.L",
            Mode::PcDisp => "(d16,PC)",
            Mode::PcIndex => "(d8,PC,Xi)",
            Mode::Imm => "#imm",
        }
    }
}

/// A set of addressing modes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Modes(u16);

impl Modes {
    /// No modes (the form has no such EA field).
    pub const NONE: Modes = Modes(0);

    /// Whether `mode` is in the set.
    pub fn contains(self, mode: Mode) -> bool {
        self.0 & (1 << mode as u16) != 0
    }

    pub fn is_empty(self) -> bool {
        self.0 == 0
    }

    fn of(modes: &[Mode]) -> Modes {
        Modes(modes.iter().fold(0, |m, &x| m | 1 << x as u16))
    }

    /// Parse a CFPRM Table 2-3 category name or a comma list of mode tokens.
    fn parse(s: &str) -> Result<Modes, String> {
        use Mode::*;
        let category = |m: &[Mode]| Ok(Modes::of(m));
        match s {
            "none" => Ok(Modes::NONE),
            "all" => Ok(Modes::of(&Mode::ALL)),
            "data" => category(&[
                Dn, Ind, PostInc, PreDec, Disp, Index, PcDisp, PcIndex, AbsW, AbsL, Imm,
            ]),
            "memory" => category(&[
                Ind, PostInc, PreDec, Disp, Index, PcDisp, PcIndex, AbsW, AbsL, Imm,
            ]),
            "control" => category(&[Ind, Disp, Index, PcDisp, PcIndex, AbsW, AbsL]),
            "alterable" => category(&[Dn, An, Ind, PostInc, PreDec, Disp, Index]),
            "data-alterable" => category(&[Dn, Ind, PostInc, PreDec, Disp, Index]),
            "memory-alterable" => category(&[Ind, PostInc, PreDec, Disp, Index]),
            _ => {
                let mut set = 0u16;
                for tok in split_list(s) {
                    let mode = Mode::ALL
                        .into_iter()
                        .find(|m| m.token() == tok)
                        .ok_or_else(|| format!("unknown addressing mode `{tok}`"))?;
                    if set & (1 << mode as u16) != 0 {
                        return Err(format!("addressing mode `{tok}` listed twice"));
                    }
                    set |= 1 << mode as u16;
                }
                Ok(Modes(set))
            }
        }
    }
}

impl fmt::Display for Modes {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.is_empty() {
            return f.write_str("none");
        }
        let names: Vec<&str> = Mode::ALL
            .into_iter()
            .filter(|m| self.contains(*m))
            .map(Mode::token)
            .collect();
        f.write_str(&names.join(","))
    }
}

/// The addressing modes a form accepts in its EA fields.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct EaClass {
    /// The `m`/`e` field (bits 5-0).
    pub ea: Modes,
    /// The `M`/`E` field (MOVE destination: mode bits 8-6, register bits 11-9).
    pub ea2: Modes,
}

/// A field of the opword: the pattern letter and its bit position.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Field {
    pub letter: u8,
    pub shift: u8,
    pub width: u8,
}

const MAX_FIELDS: usize = 6;

/// The opword fields of a form, from its pattern letters.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Fields {
    list: [Field; MAX_FIELDS],
    len: u8,
}

impl Fields {
    /// The fields, most significant first.
    pub fn iter(&self) -> impl Iterator<Item = &Field> {
        self.list[..self.len as usize].iter()
    }

    /// The value of field `letter` in `opword`; `None` when the form has no such field.
    pub fn get(&self, letter: char, opword: u16) -> Option<u32> {
        let f = self.iter().find(|f| f.letter as char == letter)?;
        Some((opword as u32 >> f.shift) & ((1 << f.width) - 1))
    }

    /// The `m`/`e` effective address as (mode, register).
    pub fn ea(&self, opword: u16) -> Option<(u32, u32)> {
        Some((self.get('m', opword)?, self.get('e', opword)?))
    }

    /// The `M`/`E` effective address (MOVE destination) as (mode, register).
    pub fn ea2(&self, opword: u16) -> Option<(u32, u32)> {
        Some((self.get('M', opword)?, self.get('E', opword)?))
    }

    fn from_pattern(pattern: &[u8; 16]) -> Result<Fields, String> {
        let mut fields = Fields::default();
        let mut i = 0;
        while i < 16 {
            let c = pattern[i];
            if c == b'0' || c == b'1' {
                i += 1;
                continue;
            }
            if !b"meMErRqcdvaxykp".contains(&c) {
                return Err(format!("unknown field letter `{}`", c as char));
            }
            let start = i;
            while i < 16 && pattern[i] == c {
                i += 1;
            }
            if fields.iter().any(|f| f.letter == c) {
                return Err(format!("field `{}` is not contiguous", c as char));
            }
            if fields.len as usize == MAX_FIELDS {
                return Err("too many fields".into());
            }
            fields.list[fields.len as usize] = Field {
                letter: c,
                shift: (16 - i) as u8,
                width: (i - start) as u8,
            };
            fields.len += 1;
        }
        Ok(fields)
    }
}

/// Words between the opword and the EA extension words.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Ext {
    /// One 16-bit operand word (displacement, bit number, immediate word).
    Word,
    /// Two words (a 32-bit displacement or immediate).
    Long,
    /// An extension word holding fields (EMAC, MOVEM mask, MULx.L/DIVx.L, MOVEC, FPU command, CAU command).
    Ext,
    /// Immediate data sized by [`Form::size`].
    Imm,
}

const MAX_EXTS: usize = 2;

/// The extension words of a form, in instruction-stream order.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Exts {
    list: [Option<Ext>; MAX_EXTS],
}

impl Exts {
    pub fn iter(&self) -> impl Iterator<Item = Ext> + '_ {
        self.list.iter().flatten().copied()
    }

    fn parse(s: &str) -> Result<Exts, String> {
        let mut exts = Exts::default();
        if s == "-" {
            return Ok(exts);
        }
        for (i, tok) in s.split(',').enumerate() {
            let ext = match tok {
                "w" => Ext::Word,
                "l" => Ext::Long,
                "ext" => Ext::Ext,
                "imm" => Ext::Imm,
                _ => return Err(format!("unknown extension `{tok}`")),
            };
            *exts.list.get_mut(i).ok_or("too many extension tokens")? = Some(ext);
        }
        Ok(exts)
    }
}

/// One line of the description.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Form {
    /// Lower-case CFPRM mnemonic without a size suffix.
    pub mnemonic: &'static str,
    pub id: Op,
    /// Operation size; also the size of #imm operands and `imm` extensions.
    pub size: Option<Size>,
    pub isa: Isa,
    pub fields: Fields,
    pub ea: EaClass,
    pub ext: Exts,
    /// Fixed bits of the pattern.
    pub mask: u16,
    /// Values of the fixed bits.
    pub value: u16,
    /// Line number in the description, for messages.
    pub line: u32,
}

impl Form {
    /// Whether `opword` is an instance of this form (fixed bits and EA modes).
    pub fn accepts(&self, opword: u16) -> bool {
        let ok = |modes: Modes, ea: Option<(u32, u32)>| match ea {
            None => modes.is_empty(),
            Some((m, r)) => Mode::from_fields(m, r).is_some_and(|mode| modes.contains(mode)),
        };
        opword & self.mask == self.value
            && ok(self.ea.ea, self.fields.ea(opword))
            && ok(self.ea.ea2, self.fields.ea2(opword))
    }

    /// Instruction length in bytes for `opword`, an instance of this form.
    pub fn length(&self, opword: u16) -> u32 {
        let ext: u32 = self
            .ext
            .iter()
            .map(|e| match e {
                Ext::Word | Ext::Ext => 1,
                Ext::Long => 2,
                Ext::Imm => Mode::Imm.ext_words(self.size).expect("checked when parsed"),
            })
            .sum();
        let ea = |ea: Option<(u32, u32)>| {
            ea.and_then(|(m, r)| Mode::from_fields(m, r))
                .map_or(0, |mode| mode.ext_words(self.size).expect("checked"))
        };
        2 * (1 + ext + ea(self.fields.ea(opword)) + ea(self.fields.ea2(opword)))
    }

    fn fixed_bits(&self) -> u32 {
        self.mask.count_ones()
    }
}

/// The parsed description and its opword index.
#[derive(Clone)]
pub struct Table {
    forms: Vec<Form>,
    /// Form index per opword; `u16::MAX` for none.
    index: Vec<u16>,
}

impl fmt::Debug for Table {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let decoded = self.index.iter().filter(|&&i| i != u16::MAX).count();
        write!(
            f,
            "Table {{ {} forms, {decoded} opwords decode }}",
            self.forms.len()
        )
    }
}

impl Table {
    /// Parse and index a description, rejecting malformed lines and ambiguous opwords.
    pub fn parse(src: &'static str) -> Result<Table, String> {
        let mut forms = Vec::new();
        let mut errors = Vec::new();
        for (n, line) in src.lines().enumerate() {
            let n = n as u32 + 1;
            match parse_line(line, n) {
                Ok(Some(form)) => forms.push(form),
                Ok(None) => {}
                Err(e) => errors.push(format!("line {n}: {e}")),
            }
        }
        if !errors.is_empty() {
            return Err(errors.join("\n"));
        }
        if forms.len() >= u16::MAX as usize {
            return Err("too many forms".into());
        }
        // Bucket by the top nibble, which every pattern fixes.
        let mut buckets: Vec<Vec<usize>> = vec![Vec::new(); 16];
        for (i, f) in forms.iter().enumerate() {
            if f.mask & 0xf000 != 0xf000 {
                return Err(format!("line {}: bits 15-12 must be fixed", f.line));
            }
            buckets[(f.value >> 12) as usize].push(i);
        }
        let mut index = vec![u16::MAX; 65536];
        for op in 0..=u16::MAX {
            let mut best: Option<usize> = None;
            let mut tie: Option<usize> = None;
            for &i in &buckets[(op >> 12) as usize] {
                let f = &forms[i];
                if !f.accepts(op) {
                    continue;
                }
                match best {
                    Some(b) if forms[b].fixed_bits() > f.fixed_bits() => {}
                    Some(b) if forms[b].fixed_bits() == f.fixed_bits() => tie = Some(i),
                    _ => {
                        best = Some(i);
                        tie = None;
                    }
                }
            }
            if let (Some(b), Some(t)) = (best, tie) {
                errors.push(format!(
                    "opword {op:#06x} is ambiguous: line {} ({}) and line {} ({})",
                    forms[b].line, forms[b].mnemonic, forms[t].line, forms[t].mnemonic
                ));
            }
            if let Some(b) = best {
                index[op as usize] = b as u16;
            }
        }
        if !errors.is_empty() {
            let n = errors.len();
            errors.truncate(20);
            return Err(format!("{n} errors:\n{}", errors.join("\n")));
        }
        Ok(Table { forms, index })
    }

    /// Every form, in description order.
    pub fn forms(&self) -> &[Form] {
        &self.forms
    }

    /// The form `opword` is an instance of.
    pub fn decode(&self, opword: u16) -> Option<&Form> {
        self.forms.get(self.index[opword as usize] as usize)
    }
}

/// Split a comma list, keeping commas inside parentheses: `Dn,(d16,An)` is two tokens.
fn split_list(s: &str) -> Vec<&str> {
    let (mut out, mut depth, mut start) = (Vec::new(), 0, 0);
    for (i, c) in s.char_indices() {
        match c {
            '(' => depth += 1,
            ')' => depth -= 1,
            ',' if depth == 0 => {
                out.push(&s[start..i]);
                start = i + 1;
            }
            _ => {}
        }
    }
    out.push(&s[start..]);
    out
}

fn parse_line(line: &'static str, n: u32) -> Result<Option<Form>, String> {
    // A comment starts at `#` followed by a blank or the end of the line (`#imm` is a mode token).
    let at = line
        .match_indices('#')
        .map(|(i, _)| i)
        .find(|&i| line[i + 1..].chars().next().is_none_or(char::is_whitespace));
    let (body, comment) = at.map_or((line, ""), |i| (&line[..i], &line[i + 1..]));
    let cols: Vec<&'static str> = body.split_whitespace().collect();
    if cols.is_empty() {
        return Ok(None);
    }
    let [mnemonic, isa, size, pattern, ea, ext, op] = cols[..] else {
        return Err(format!("expected 7 columns, found {}", cols.len()));
    };
    if !comment.contains("CFPRM") {
        return Err("no CFPRM citation in the comment".into());
    }
    let isa = Isa::parse(isa).ok_or_else(|| format!("unknown isa `{isa}`"))?;
    let size = match size {
        "B" => Some(Size::Byte),
        "W" => Some(Size::Word),
        "L" => Some(Size::Long),
        "-" => None,
        _ => return Err(format!("unknown size `{size}`")),
    };
    let pattern: &[u8; 16] = pattern
        .as_bytes()
        .try_into()
        .map_err(|_| "pattern is not 16 characters")?;
    let (mut mask, mut value) = (0u16, 0u16);
    for (i, &c) in pattern.iter().enumerate() {
        let bit = 1 << (15 - i);
        match c {
            b'0' => mask |= bit,
            b'1' => {
                mask |= bit;
                value |= bit;
            }
            _ => {}
        }
    }
    let fields = Fields::from_pattern(pattern)?;
    let has = |l: char| fields.iter().any(|f| f.letter as char == l);
    let (ea_src, ea2_src) = ea.split_once('>').map_or((ea, None), |(a, b)| (a, Some(b)));
    let ea = EaClass {
        ea: Modes::parse(ea_src)?,
        ea2: ea2_src.map(Modes::parse).transpose()?.unwrap_or_default(),
    };
    for (field, modes, mode_l, reg_l) in [("m/e", ea.ea, 'm', 'e'), ("M/E", ea.ea2, 'M', 'E')] {
        let present = has(mode_l) && has(reg_l);
        if present != !modes.is_empty() || has(mode_l) != has(reg_l) {
            return Err(format!(
                "the {field} field and its ea-class ({modes}) do not agree"
            ));
        }
        if fields
            .iter()
            .any(|f| (f.letter as char == mode_l || f.letter as char == reg_l) && f.width != 3)
        {
            return Err(format!("the {field} field must be 3+3 bits"));
        }
        if modes.contains(Mode::Imm) && size.is_none() {
            return Err("#imm needs a size".into());
        }
    }
    let ext = Exts::parse(ext)?;
    if ext.iter().any(|e| e == Ext::Imm) && size.is_none() {
        return Err("`imm` needs a size".into());
    }
    let id = Op::from_name(op).ok_or_else(|| format!("unknown op `{op}`"))?;
    Ok(Some(Form {
        mnemonic,
        id,
        size,
        isa,
        fields,
        ea,
        ext,
        mask,
        value,
        line: n,
    }))
}

fn table() -> &'static Table {
    static TABLE: OnceLock<Table> = OnceLock::new();
    TABLE.get_or_init(|| Table::parse(SOURCE).unwrap_or_else(|e| panic!("isa/v4e.isa: {e}")))
}

/// The form `opword` is an instance of; `None` for an opword this core does not implement or recognise.
pub fn decode(opword: u16) -> Option<&'static Form> {
    table().decode(opword)
}

/// Every form of the description.
pub fn forms() -> &'static [Form] {
    table().forms()
}

/// Length in bytes of the instruction at `pc`: opword, extension words and EA extension words. `None` when the
/// opword cannot be read or does not decode.
pub fn length(mem: &impl Fn(u32) -> Option<u16>, pc: u32) -> Option<u32> {
    let opword = mem(pc)?;
    Some(decode(opword)?.length(opword))
}
