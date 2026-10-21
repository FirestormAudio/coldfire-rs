//! The interrupt controllers INTC0-2 and the global IACK registers (RM ch. 17).
//!
//! Each controller has 64 sources. A source requests when its ICR holds a level (1-7) and either a peripheral
//! asserts its line ([`IrqLines`]) with its IMRH:IMRL bit clear, or its bit in INTFRCH:INTFRCL is set: a forced
//! request is not affected by IMR (RM 17.2.3; measured on an MCF54415). The controller reports
//! its highest request to the core: the highest level, and within a level the highest source number (RM 17.3.1).
//! The vector is `64 + 64 * index + source` (RM 17.3.1.3). Across controllers INTC0 wins at an equal level (RM
//! 17.3.2): add the three to the machine in order.
//!
//! ICONFIG, CLMASK and SLMASK exist once for the three controllers, in INTC0's space (RM 17.2.4, RM 17.2.7, RM
//! 17.2.8). With ICONFIG\[EMASK\] set, an interrupt acknowledge (the core's, or a read of an LnIACK register)
//! saves CLMASK in SLMASK and loads CLMASK with the level acknowledged, and requests at levels up to CLMASK are
//! masked in every controller. [`Intc::chip`] builds the three controllers sharing that state, and the global
//! IACK registers ([`GlobalIack`], at 0xFC05_4000, RM Table 17-1) that read over all three.
//!
//! Accesses outside the register map, accesses of the wrong size to the 16- and 32-bit registers, and writes to
//! the IACK registers are refused with [`Stop::Unmodelled`], the detail naming the controller and the register.

use std::cell::RefCell;
use std::rc::Rc;

use crate::device::{Device, DeviceCx};
use crate::irq::{InterruptController, Irq, IrqLines};
use crate::mem::Size;
use crate::stop::Stop;

/// ICONFIG\[EMASK\] (RM Table 17-9).
const EMASK: u16 = 0x0020;
/// The ICONFIG bits that are not reserved: ELVLPRI\[7:1\] and EMASK (RM Table 17-9).
const ICONFIG_BITS: u16 = 0xFE00 | EMASK;
/// CLMASK and SLMASK at reset: no level masked (RM 17.2.7, RM 17.2.8).
const NO_MASK: u8 = 0x0F;
/// The spurious-interrupt vector an LnIACK read returns with no request at that level (RM 17.2.10).
const SPURIOUS: u32 = 0x18;
/// SIMR\[SALL\] and CIMR\[CALL\] (RM Table 17-10, RM Table 17-11).
const ALL: u8 = 0x40;

/// The 32-bit registers at 0x00-0x17, by offset / 4 (RM Table 17-2).
const LONG_NAMES: [&str; 6] = ["IPRH", "IPRL", "IMRH", "IMRL", "INTFRCH", "INTFRCL"];

/// One controller's own registers.
#[derive(Debug, Clone)]
struct Regs {
    /// ICR0-63: the level, bits 2:0.
    icr: [u8; 64],
    /// IMRH:IMRL.
    imr: u64,
    /// INTFRCH:INTFRCL.
    force: u64,
}

impl Default for Regs {
    fn default() -> Regs {
        Regs {
            icr: [0; 64],
            imr: !0,
            force: 0,
        }
    }
}

/// The state of the three controllers: their registers and the one ICONFIG, CLMASK and SLMASK.
#[derive(Debug)]
struct Chip {
    iconfig: u16,
    /// 0-7, or 0x0F; other values are reserved and refused.
    clmask: u8,
    slmask: u8,
    intc: [Regs; 3],
}

impl Chip {
    fn new() -> Chip {
        Chip {
            iconfig: 0,
            clmask: NO_MASK,
            slmask: NO_MASK,
            intc: Default::default(),
        }
    }

    /// Whether CLMASK masks requests at `level`: only with EMASK set (RM 17.3.1.2: the level masking is performed
    /// if the feature is enabled; RM Table 17-12: only levels above CLMASK are processed, 0x0F masks nothing).
    fn level_masked(&self, level: u8) -> bool {
        self.iconfig & EMASK != 0 && self.clmask <= 7 && level <= self.clmask
    }

    /// The highest request of controller `i` whose level `want` accepts, as (level, source), with `asserted` its
    /// asserted lines: asserted and not masked by IMR, or forced (RM 17.2.3: IMR does not affect a forced request;
    /// measured on an MCF54415); not masked by CLMASK; ICR not 0. The highest level wins, then
    /// the highest source (RM 17.3.1).
    fn best(&self, i: usize, asserted: u64, want: impl Fn(u8) -> bool) -> Option<(u8, u8)> {
        let r = &self.intc[i];
        let mut live = asserted & !r.imr | r.force;
        let mut best: Option<(u8, u8)> = None;
        // Sources from the highest down, so the first at a level is the one that wins it.
        while live != 0 {
            let s = 63 - live.leading_zeros() as u8;
            live &= !(1 << s);
            let level = r.icr[s as usize];
            if level == 0 || self.level_masked(level) || !want(level) {
                continue;
            }
            if best.is_none_or(|(l, _)| level > l) {
                best = Some((level, s));
            }
        }
        best
    }

    /// The highest request over the controllers in `which` (in priority order: INTC0 first at an equal level, RM
    /// 17.3.2), as (level, vector).
    fn best_of(
        &self,
        which: impl Iterator<Item = usize>,
        lines: &IrqLines,
        want: impl Fn(u8) -> bool + Copy,
    ) -> Option<(u8, u32)> {
        let mut best: Option<(u8, u32)> = None;
        for i in which {
            if let Some((level, s)) = self.best(i, lines.asserted(i), want)
                && best.is_none_or(|(l, _)| level > l)
            {
                best = Some((level, vector(i, s)));
            }
        }
        best
    }

    /// An interrupt acknowledge at `level`: with EMASK, CLMASK is saved in SLMASK and loaded with the level (RM
    /// 17.2.7, RM 17.3.1.3).
    fn acknowledge(&mut self, level: u8) {
        if self.iconfig & EMASK != 0 {
            self.slmask = self.clmask;
            self.clmask = level;
        }
    }

    /// A read of the software IACK (`n` = 0) or the level-`n` IACK over the controllers in `which`: the vector of
    /// the highest request (at level `n`), or 0 (SWIACK) or 0x18 (LnIACK) when there is none (RM 17.2.10). A
    /// level-`n` IACK that finds a request is an acknowledge.
    // guess: the RM has the level-n IACK load CLMASK, not the software IACK, which only looks; and an IACK sees only
    // the requests CLMASK lets through (with EMASK), so a masked level reads as spurious and loads nothing.
    fn iack(&mut self, n: u32, which: impl Iterator<Item = usize>, lines: &IrqLines) -> u32 {
        if n == 0 {
            return self.best_of(which, lines, |_| true).map_or(0, |(_, v)| v);
        }
        match self.best_of(which, lines, |l| u32::from(l) == n) {
            Some((level, v)) => {
                self.acknowledge(level);
                v
            }
            None => SPURIOUS,
        }
    }
}

/// The vector of source `s` of controller `i` (RM 17.3.1.3).
fn vector(i: usize, s: u8) -> u32 {
    64 + 64 * i as u32 + u32::from(s)
}

/// The IACK register at `SWIACK + 4 * n`.
fn iack_name(n: u32) -> String {
    match n {
        0 => "SWIACK".to_string(),
        n => format!("L{n}IACK"),
    }
}

/// The number of bytes in an access of `size`.
fn len(size: Size) -> u32 {
    match size {
        Size::Byte => 1,
        Size::Word => 2,
        Size::Long => 4,
    }
}

/// A register (or run of byte registers) an access decodes to.
enum Reg {
    /// One of [`LONG_NAMES`], by offset / 4.
    Long(usize),
    Iconfig,
    /// Byte registers (SIMR, CIMR, CLMASK, SLMASK, ICRn): each byte of the access is one, big-endian.
    Bytes,
    /// SWIACK (0) or LnIACK.
    Iack(u32),
}

/// Interrupt controller `index` (0-2): the device at 0xFC04_8000, 0xFC04_C000 or 0xFC05_0000, with a window of
/// 0x4000 (registers at 0x00-0xFF, RM Table 17-2), and the requests of [`IrqLines`] controller `index`.
///
/// Its registers (RM 17.2):
/// - IPRH/IPRL (read-only, writes ignored): the sources peripherals assert, masked or not; never the forced
///   ones, masked or unmasked (measured on an MCF54415).
/// - IMRH/IMRL (reset all ones) and INTFRCH/INTFRCL (reset 0): read/write; a forced source requests whatever its
///   IMR bit (RM 17.2.3; measured on an MCF54415: a forced source with its IMR bit set reads from LnIACK and
///   GLnIACK as when unmasked).
/// - SIMR/CIMR (bytes, read 0): set or clear IMR bit `value & 63`, or every bit with SALL/CALL (0x40). Written
///   together (a word at SIMR) the clear is applied first, so SIMR wins (RM 17.2.6).
/// - ICR0-63 (bytes at 0x40-0x7F, reset 0): the level in bits 2:0; an access of any size covers consecutive
///   ICRs, big-endian. The byte registers SIMR, CIMR, CLMASK and SLMASK pack the same way.
/// - ICONFIG (16-bit), CLMASK and SLMASK (bytes, reset 0x0F): INTC0 only, shared by the controllers of one
///   [`Intc::chip`]. CLMASK and SLMASK values other than 0-7 and 0x0F are reserved and refused.
/// - SWIACK and L1IACK-L7IACK (bytes at 0xE0 + 4n, read-only): see [`GlobalIack`]'s registers; these look at this
///   controller only.
///
/// The 32-bit registers take only 32-bit accesses and ICONFIG only 16-bit ones: other sizes are refused (a
/// tripwire for code that relies on narrow accesses the RM does not describe).
#[derive(Debug)]
pub struct Intc {
    index: usize,
    chip: Rc<RefCell<Chip>>,
}

impl Intc {
    /// The chip's three controllers, INTC0-2, sharing one ICONFIG, CLMASK and SLMASK, and the global IACK
    /// registers over them. This is the only way to build them: the level masks exist once for all three.
    pub fn chip() -> ([Intc; 3], GlobalIack) {
        let chip = Rc::new(RefCell::new(Chip::new()));
        let intc = [0, 1, 2].map(|index| Intc {
            index,
            chip: chip.clone(),
        });
        (intc, GlobalIack { chip })
    }

    /// Which controller this is.
    pub fn index(&self) -> usize {
        self.index
    }

    /// Refuse the access in flight with the detail `"INTCn <what>"`.
    fn refuse(&self, cx: &mut DeviceCx, what: impl std::fmt::Display) {
        cx.refuse(Stop::Unmodelled, format!("INTC{} {what}", self.index));
    }

    /// The register an access of `size` at `offset` decodes to, or why it is refused.
    fn decode(&self, offset: u32, size: Size) -> Result<Reg, String> {
        let end = offset + len(size);
        let reg = match offset {
            0x00..0x18 => {
                let k = (offset / 4) as usize;
                if size != Size::Long || offset % 4 != 0 {
                    return Err(format!(
                        "{}: {size:?} access to a 32-bit register",
                        LONG_NAMES[k]
                    ));
                }
                Reg::Long(k)
            }
            0x1A | 0x1B => {
                if self.index != 0 {
                    return Err("ICONFIG: only in INTC0's space".to_string());
                }
                if size != Size::Word || offset != 0x1A {
                    return Err(format!("ICONFIG: {size:?} access to a 16-bit register"));
                }
                Reg::Iconfig
            }
            0x1C..0x20 if end <= 0x20 => {
                if self.index != 0 {
                    for (o, name) in [(0x1E, "CLMASK"), (0x1F, "SLMASK")] {
                        if (offset..end).contains(&o) {
                            return Err(format!("{name}: only in INTC0's space"));
                        }
                    }
                }
                Reg::Bytes
            }
            0x40..0x80 if end <= 0x80 => Reg::Bytes,
            0xE0..0x100 if offset % 4 == 0 => {
                let n = (offset - 0xE0) / 4;
                if size != Size::Byte {
                    return Err(format!(
                        "{}: {size:?} access to an 8-bit register",
                        iack_name(n)
                    ));
                }
                Reg::Iack(n)
            }
            _ => return Err(format!("+{offset:#x}: no register there (RM Table 17-2)")),
        };
        Ok(reg)
    }
}

impl Device for Intc {
    fn read(&mut self, offset: u32, size: Size, cx: &mut DeviceCx) -> u32 {
        let reg = match self.decode(offset, size) {
            Ok(reg) => reg,
            Err(why) => {
                self.refuse(cx, why);
                return 0;
            }
        };
        let i = self.index;
        let mut chip = self.chip.borrow_mut();
        match reg {
            Reg::Long(k) => {
                let r = &chip.intc[i];
                let v = match k {
                    // Measured on an MCF54415: IPR never shows forced sources.
                    0 | 1 => cx.irq.asserted(i),
                    2 | 3 => r.imr,
                    _ => r.force,
                };
                if k % 2 == 0 {
                    (v >> 32) as u32
                } else {
                    v as u32
                }
            }
            Reg::Iconfig => u32::from(chip.iconfig),
            Reg::Bytes => (offset..offset + len(size)).fold(0, |acc, o| {
                let b = match o {
                    0x1E => chip.clmask,
                    0x1F => chip.slmask,
                    0x40..0x80 => chip.intc[i].icr[(o - 0x40) as usize],
                    // SIMR and CIMR read as zero.
                    _ => 0,
                };
                acc << 8 | u32::from(b)
            }),
            Reg::Iack(n) => chip.iack(n, i..i + 1, cx.irq),
        }
    }

    fn write(&mut self, offset: u32, size: Size, value: u32, cx: &mut DeviceCx) {
        let reg = match self.decode(offset, size) {
            Ok(reg) => reg,
            Err(why) => return self.refuse(cx, why),
        };
        let i = self.index;
        match reg {
            Reg::Long(k) => {
                let mut chip = self.chip.borrow_mut();
                let r = &mut chip.intc[i];
                let field = match k {
                    // IPR is read-only: writes are ignored (RM 17.2.1).
                    0 | 1 => return,
                    2 | 3 => &mut r.imr,
                    _ => &mut r.force,
                };
                let shift = if k % 2 == 0 { 32 } else { 0 };
                *field = *field & !(0xFFFF_FFFF << shift) | u64::from(value) << shift;
            }
            Reg::Iconfig => self.chip.borrow_mut().iconfig = value as u16 & ICONFIG_BITS,
            Reg::Bytes => {
                let n = len(size);
                let bytes: Vec<(u32, u8)> = (0..n)
                    .map(|j| (offset + j, (value >> (8 * (n - 1 - j))) as u8))
                    .collect();
                for &(o, b) in &bytes {
                    if matches!(o, 0x1E | 0x1F) && b > 7 && b != 0x0F {
                        let name = if o == 0x1E { "CLMASK" } else { "SLMASK" };
                        return self.refuse(
                            cx,
                            format_args!(
                                "{name}: reserved value {b:#x} (RM Table 17-12, RM Table 17-13)"
                            ),
                        );
                    }
                }
                let mut chip = self.chip.borrow_mut();
                // guess: SIMR and CIMR written together (RM 17.2.6: SIMR has priority) apply the clear first.
                let cimr_first = bytes.iter().filter(|(o, _)| *o == 0x1D);
                for &(o, b) in cimr_first.chain(bytes.iter().filter(|(o, _)| *o != 0x1D)) {
                    let r = &mut chip.intc[i];
                    match o {
                        0x1C if b & ALL != 0 => r.imr = !0,
                        0x1C => r.imr |= 1 << (b & 63),
                        0x1D if b & ALL != 0 => r.imr = 0,
                        0x1D => r.imr &= !(1 << (b & 63)),
                        0x1E => chip.clmask = b,
                        0x1F => chip.slmask = b,
                        _ => r.icr[(o - 0x40) as usize] = b & 7,
                    }
                }
            }
            Reg::Iack(n) => self.refuse(
                cx,
                format_args!(
                    "{}: write (an error termination, RM Table 17-18)",
                    iack_name(n)
                ),
            ),
        }
    }

    fn poll_counted(&self) -> bool {
        false
    }
}

impl InterruptController for Intc {
    fn pending(&self, lines: &IrqLines, allow_level7: bool) -> Option<Irq> {
        let chip = self.chip.borrow();
        let (level, s) = chip.best(self.index, lines.asserted(self.index), |l| {
            l != 7 || allow_level7
        })?;
        Some(Irq {
            level,
            vector: vector(self.index, s) as u8,
        })
    }

    fn level7(&self, lines: &IrqLines) -> bool {
        let chip = self.chip.borrow();
        chip.best(self.index, lines.asserted(self.index), |l| l == 7)
            .is_some()
    }

    /// The core's IACK cycle: with EMASK, CLMASK is saved in SLMASK and loaded with the level taken (RM 17.3.1.3).
    fn acknowledge(&mut self, irq: Irq, _cx: &mut DeviceCx) {
        self.chip.borrow_mut().acknowledge(irq.level);
    }
}

/// The global IACK registers (RM 17.2.10, at 0xFC05_4000 with a window of 0x4000, from [`Intc::chip`]): GSWIACK
/// (byte at 0xE0) reads the vector of the highest request over all three controllers, or 0 when there is none;
/// GLnIACK (bytes at 0xE0 + 4n, n = 1-7) the vector of the highest request at level n, or 0x18 when there is none.
/// At an equal level INTC0 wins, then INTC1 (RM 17.3.2). With ICONFIG\[EMASK\] set, a GLnIACK read that finds a
/// request is an acknowledge: CLMASK is saved in SLMASK and loaded with n. Writes, other sizes and other offsets
/// are refused.
#[derive(Debug)]
pub struct GlobalIack {
    chip: Rc<RefCell<Chip>>,
}

impl GlobalIack {
    /// The register at `offset` for an access of `size`, or why it is refused.
    fn decode(offset: u32, size: Size) -> Result<u32, String> {
        match offset {
            0xE0..0x100 if offset % 4 == 0 => {
                let n = (offset - 0xE0) / 4;
                if size == Size::Byte {
                    Ok(n)
                } else {
                    Err(format!(
                        "G{}: {size:?} access to an 8-bit register",
                        iack_name(n)
                    ))
                }
            }
            _ => Err(format!("+{offset:#x}: no register there (RM Table 17-2)")),
        }
    }
}

impl Device for GlobalIack {
    fn read(&mut self, offset: u32, size: Size, cx: &mut DeviceCx) -> u32 {
        match GlobalIack::decode(offset, size) {
            Ok(n) => self.chip.borrow_mut().iack(n, 0..3, cx.irq),
            Err(why) => {
                cx.refuse(Stop::Unmodelled, format!("INTC global IACK {why}"));
                0
            }
        }
    }

    fn write(&mut self, offset: u32, size: Size, _value: u32, cx: &mut DeviceCx) {
        let why = match GlobalIack::decode(offset, size) {
            Ok(n) => format!(
                "G{}: write (an error termination, RM Table 17-18)",
                iack_name(n)
            ),
            Err(why) => why,
        };
        cx.refuse(Stop::Unmodelled, format!("INTC global IACK {why}"));
    }

    fn poll_counted(&self) -> bool {
        false
    }
}
