//! The enhanced multiply-accumulate unit (RM ch. 5, CFPRM ch. 6): four 48-bit accumulators, MACSR and MASK.
//!
//! Each accumulator is held as its two registers, ACCn (32 bits) and ACCextn (16 bits), exactly as MOVE to and
//! from them sees them. The 48-bit value they form depends on the mode (RM 5.3): `{ACCextn[15:0], ACCn}` for
//! integers and `{ACCextn[15:8], ACCn, ACCextn[7:0]}` for fractions, so changing MACSR\[F/I\] reinterprets the
//! same bits, as the hardware does. The arithmetic works on that 48-bit value in an `i64` (or `u64` for unsigned
//! integers), wide enough that every sum, product and rounding step is exact before overflow is checked.

/// MACSR\[PAV0\]; PAVn is `PAV0 << n`.
pub const PAV0: u32 = 0x100;
/// Overflow saturation mode.
pub const OMC: u32 = 0x80;
/// Signed/unsigned integers; in fractional mode, 16-bit rounding on stores.
pub const SU: u32 = 0x40;
/// Fractional/integer.
pub const FI: u32 = 0x20;
/// Round/truncate.
pub const RT: u32 = 0x10;
pub const N: u32 = 0x08;
pub const Z: u32 = 0x04;
pub const V: u32 = 0x02;
pub const EV: u32 = 0x01;
/// The implemented MACSR bits (RM 5.2.1: 31-12 are reserved and read as zero).
const MACSR_MASK: u32 = 0x0FFF;

const MASK48: u64 = (1 << 48) - 1;
const MASK40: u64 = (1 << 40) - 1;

/// The operating mode MACSR\[S/U, F/I\] selects (RM 5.2.1 Table 5-3).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Mode {
    Signed,
    Unsigned,
    Fractional,
}

/// One multiply-accumulate, with the fields of its extension word.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MacOp {
    /// The accumulator, 0-3.
    pub acc: usize,
    /// Longword operands (else words).
    pub long: bool,
    /// The scale factor field: 00 none, 01 `<< 1`, 10 reserved, 11 `>> 1`.
    pub sf: u32,
    /// MSAC: subtract the product.
    pub subtract: bool,
    /// Use the upper word of Rx (Ry) for a word operation.
    pub upper_x: bool,
    pub upper_y: bool,
}

/// The EMAC's programmer-visible registers.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Emac {
    /// ACC0-ACC3.
    pub acc: [u32; 4],
    /// ACCext0-ACCext3: the upper extension byte in bits 15-8, the lower in 7-0.
    pub ext: [u16; 4],
    /// MACSR, bits 11-0.
    pub macsr: u32,
    /// MASK as read: bits 31-16 are always set (RM 5.2.2).
    pub mask: u32,
}

impl Default for Emac {
    fn default() -> Self {
        Emac::new()
    }
}

/// Sign-extend the low 48 bits.
#[inline(always)]
fn sext48(v: u64) -> i64 {
    ((v << 16) as i64) >> 16
}

/// Sign-extend the low 40 bits.
#[inline(always)]
fn sext40(v: u64) -> i64 {
    ((v << 24) as i64) >> 24
}

/// `v >> k`, rounded to nearest with ties to even (RM 5.3.1.1).
#[inline(always)]
fn round(v: i64, k: u32) -> i64 {
    let q = v >> k;
    let r = v & ((1 << k) - 1);
    let half = 1 << (k - 1);
    if r > half || (r == half && q & 1 != 0) {
        q + 1
    } else {
        q
    }
}

/// Whether `v` fits a signed `bits`-bit field.
#[inline(always)]
fn fits(v: i64, bits: u32) -> bool {
    (-(1i64 << (bits - 1))..(1i64 << (bits - 1))).contains(&v)
}

impl Emac {
    /// The state after reset (RM Table 5-1): MACSR zero, MASK all ones. The accumulators are undefined after
    /// reset; this core clears them.
    pub fn new() -> Emac {
        Emac {
            acc: [0; 4],
            ext: [0; 4],
            macsr: 0,
            mask: 0xFFFF_FFFF,
        }
    }

    fn mode(&self) -> Mode {
        match (self.macsr & SU != 0, self.macsr & FI != 0) {
            (false, false) => Mode::Signed,
            (true, false) => Mode::Unsigned,
            (_, true) => Mode::Fractional,
        }
    }

    /// The 48-bit accumulator `n` in the current mode (RM 5.3).
    pub fn full(&self, n: usize) -> u64 {
        let (acc, ext) = (self.acc[n] as u64, self.ext[n] as u64);
        match self.mode() {
            Mode::Fractional => ((ext >> 8) << 40) | (acc << 8) | (ext & 0xFF),
            _ => (ext << 32) | acc,
        }
    }

    /// Write the 48-bit accumulator `n` in the current mode.
    pub fn set_full(&mut self, n: usize, v: u64) {
        let v = v & MASK48;
        match self.mode() {
            Mode::Fractional => {
                self.acc[n] = (v >> 8) as u32;
                self.ext[n] = (((v >> 40) << 8) | (v & 0xFF)) as u16;
            }
            _ => {
                self.acc[n] = v as u32;
                self.ext[n] = (v >> 32) as u16;
            }
        }
    }

    fn pav(&self, n: usize) -> bool {
        self.macsr & (PAV0 << n) != 0
    }

    fn set_pav(&mut self, n: usize, on: bool) {
        if on {
            self.macsr |= PAV0 << n;
        } else {
            self.macsr &= !(PAV0 << n);
        }
    }

    /// N, Z and EV from accumulator `n` and V from `v` (RM 5.3.5; CFPRM ch. 6 MOVE ACC to ACC, MOVE to ACC). EV is
    /// set when the value does not fit the 32 (integer) or 40 (fractional) low bits.
    fn set_flags(&mut self, n: usize, v: bool) {
        let full = self.full(n);
        let ev = match self.mode() {
            Mode::Signed => !fits(sext48(full), 32),
            Mode::Unsigned => full >> 32 != 0,
            Mode::Fractional => !fits(sext48(full), 40),
        };
        let mut f = 0;
        if full & (1 << 47) != 0 {
            f |= N;
        }
        if full == 0 {
            f |= Z;
        }
        if v {
            f |= V;
        }
        if ev {
            f |= EV;
        }
        self.macsr = (self.macsr & !(N | Z | V | EV)) | f;
    }

    /// MOVE to MACSR.
    pub fn set_macsr(&mut self, v: u32) {
        self.macsr = v & MACSR_MASK;
    }

    /// MOVE to MASK: the low 16 bits are loaded (RM 5.2.2).
    pub fn set_mask(&mut self, v: u32) {
        self.mask = 0xFFFF_0000 | (v & 0xFFFF);
    }

    // guess: the CFPRM ch. 6 MOVE to/from ACCext01 tables put ACCext1 in bits 31-16 and ACCext0 in bits 15-0
    // (ACCext3 and ACCext2 likewise); RM Figure 5-5 draws ACC0U/ACC0L in bits 31-16 instead. This core follows the
    // instruction pages.
    /// MOVE from ACCext01 (`pair` 0) or ACCext23 (`pair` 1).
    pub fn accext(&self, pair: usize) -> u32 {
        ((self.ext[2 * pair + 1] as u32) << 16) | self.ext[2 * pair] as u32
    }

    /// MOVE to ACCext01 (`pair` 0) or ACCext23 (`pair` 1).
    pub fn set_accext(&mut self, pair: usize, v: u32) {
        self.ext[2 * pair + 1] = (v >> 16) as u16;
        self.ext[2 * pair] = v as u16;
    }

    // guess: "N: set if the msb of the result is set" is read as ACC[47], as RM 5.3.5 sets N for MAC; an unsigned
    // load (extension cleared) therefore never sets N.
    /// MOVE to ACC (CFPRM ch. 6): the extension is the sign of `v` in signed integer mode, zero in unsigned
    /// integer mode, and in fractional mode the sign in the upper byte and zero in the lower. PAVn, V and EV are
    /// cleared; N and Z come from the result.
    pub fn load(&mut self, n: usize, v: u32) {
        let sign = if v & 0x8000_0000 != 0 { 0xFFFF } else { 0 };
        self.acc[n] = v;
        self.ext[n] = match self.mode() {
            Mode::Signed => sign,
            Mode::Unsigned => 0,
            Mode::Fractional => sign & 0xFF00,
        };
        self.set_pav(n, false);
        self.set_flags(n, false);
        self.macsr &= !EV;
    }

    /// MOVE ACCy,ACCx: the 48 bits and the PAV flag; N Z EV from the result, V from the copied flag.
    pub fn copy(&mut self, from: usize, to: usize) {
        self.acc[to] = self.acc[from];
        self.ext[to] = self.ext[from];
        let pav = self.pav(from);
        self.set_pav(to, pav);
        self.set_flags(to, pav);
    }

    /// The value MOVE ACCn,Rx stores: CFPRM ch. 6 MOVE from ACC, with its saturation and rounding per mode.
    pub fn store(&self, n: usize) -> u32 {
        let full = self.full(n);
        let omc = self.macsr & OMC != 0;
        match self.mode() {
            Mode::Signed => {
                let v = sext48(full);
                if !omc || fits(v, 32) {
                    full as u32
                } else if v >= 0 {
                    0x7FFF_FFFF
                } else {
                    0x8000_0000
                }
            }
            Mode::Unsigned => {
                if !omc || full >> 32 == 0 {
                    full as u32
                } else {
                    0xFFFF_FFFF
                }
            }
            Mode::Fractional if self.macsr & SU == 0 => {
                // A 32-bit fraction: ACC[39:8], truncated or (R/T) rounded by ACC[7:0].
                let v = sext48(full);
                let t = if self.macsr & RT != 0 {
                    round(v, 8)
                } else {
                    v >> 8
                };
                if !omc || fits(t, 32) {
                    t as u32
                } else if t >= 0 {
                    0x7FFF_FFFF
                } else {
                    0x8000_0000
                }
            }
            Mode::Fractional => {
                // A 16-bit fraction in the low word: ACC[39:24] rounded by ACC[23:0].
                let t = round(sext48(full), 24);
                if !omc || fits(t, 16) {
                    t as u32 & 0xFFFF
                } else if t >= 0 {
                    0x7FFF
                } else {
                    0x8000
                }
            }
        }
    }

    /// MOVCLR: the store of MOVE from ACC, then ACCn, ACCextn and PAVn are cleared.
    pub fn store_and_clear(&mut self, n: usize) -> u32 {
        let v = self.store(n);
        self.acc[n] = 0;
        self.ext[n] = 0;
        self.set_pav(n, false);
        v
    }

    /// MAC or MSAC of `ry` and `rx` into `op.acc`, following the algorithm of RM 5.3.5 for each mode.
    ///
    /// With OMC set and PAVn already set the accumulator is left alone (it holds a saturated value). Otherwise the
    /// product is formed, a product overflow (integer longwords only) or a 48-bit accumulation overflow sets PAVn,
    /// and with OMC the result saturates. V is PAVn afterwards; N, Z and EV come from the accumulator.
    pub fn mac(&mut self, op: MacOp, ry: u32, rx: u32) {
        let n = op.acc;
        let omc = self.macsr & OMC != 0;
        // guess: RM 5.3.5 clears PAVn before each operation, but RM Table 5-2 ("once set, each flag remains set
        // until V is cleared by a move.l, MACSR instruction or the accumulator is loaded directly") and the CFPRM
        // ch. 6 MAC/MSAC pages ("PAVx: ... unchanged otherwise") make it sticky; it is never cleared here.
        if !(omc && self.pav(n)) {
            let half = |r: u32, upper: bool| if upper { r >> 16 } else { r & 0xFFFF };
            let (result, overflow) = match self.mode() {
                Mode::Signed => {
                    let (y, x) = if op.long {
                        (ry as i32 as i64, rx as i32 as i64)
                    } else {
                        (
                            half(ry, op.upper_y) as u16 as i16 as i64,
                            half(rx, op.upper_x) as u16 as i16 as i64,
                        )
                    };
                    self.signed(n, op, y * x, omc)
                }
                Mode::Unsigned => {
                    let (y, x) = if op.long {
                        (ry as u64, rx as u64)
                    } else {
                        (half(ry, op.upper_y) as u64, half(rx, op.upper_x) as u64)
                    };
                    self.unsigned(n, op, y * x, omc)
                }
                Mode::Fractional => {
                    let (y, x) = if op.long {
                        (ry as i32 as i128, rx as i32 as i128)
                    } else {
                        (
                            (half(ry, op.upper_y) << 16) as i32 as i128,
                            (half(rx, op.upper_x) << 16) as i32 as i128,
                        )
                    };
                    self.fractional(n, op, y * x, omc)
                }
            };
            self.set_full(n, result);
            if overflow {
                self.set_pav(n, true);
            }
        }
        let pav = self.pav(n);
        self.set_flags(n, pav);
    }

    /// Signed integers: the 48-bit result and whether a product or accumulation overflow happened.
    fn signed(&self, n: usize, op: MacOp, product: i64, omc: bool) -> (u64, bool) {
        // A product that does not fit 40 bits (product[63:39] neither all zeros nor all ones).
        let product_overflow = !fits(product, 40);
        if product_overflow && omc {
            let positive = (product >= 0) != op.subtract;
            return (Self::saturate_signed(positive), true);
        }
        // guess: without OMC, RM 5.3.5 assigns no result after a product overflow; this core accumulates the low
        // 40 bits of the product, sign-extended, as for any integer product (RM 5.3: "the low-order 40 bits of the
        // product are formed and used with the destination accumulator").
        let p = sext40(product as u64);
        let p = match op.sf {
            1 => p << 1,
            3 => p >> 1,
            _ => p,
        };
        let acc = sext48(self.full(n));
        let sum = if op.subtract { acc - p } else { acc + p };
        if !fits(sum, 48) {
            if omc {
                return (Self::saturate_signed(sum >= 0), true);
            }
            return (sum as u64, true);
        }
        (sum as u64, product_overflow)
    }

    /// The signed integer saturation values: 0x0000_7FFF_FFFF or 0xFFFF_8000_0000 (RM 5.3.5).
    fn saturate_signed(positive: bool) -> u64 {
        if positive {
            0x0000_7FFF_FFFF
        } else {
            0xFFFF_8000_0000
        }
    }

    /// Unsigned integers.
    fn unsigned(&self, n: usize, op: MacOp, product: u64, omc: bool) -> (u64, bool) {
        // Saturation is all ones for MAC and zero for MSAC (RM 5.3.5).
        let saturated = if op.subtract { 0 } else { MASK48 };
        let product_overflow = product >> 40 != 0;
        if product_overflow && omc {
            return (saturated, true);
        }
        let p = product & MASK40;
        let p = match op.sf {
            1 => p << 1,
            3 => p >> 1,
            _ => p,
        };
        let acc = self.full(n);
        let (sum, carry) = if op.subtract {
            acc.overflowing_sub(p)
        } else {
            (acc + p, acc + p > MASK48)
        };
        if carry {
            return (if omc { saturated } else { sum & MASK48 }, true);
        }
        (sum, product_overflow)
    }

    /// Signed fractions: `product` is Y * X of the 32-bit operands. The scale factor is ignored (RM 5.3.1.4).
    fn fractional(&self, n: usize, op: MacOp, product: i128, omc: bool) -> (u64, bool) {
        // product[63:0] = (Y * X) << 1; held exactly, so -1 * -1 is +2^63, which is the zero-filled
        // product[71:64] RM 5.3.5 asks for in that case and the sign extension in every other.
        let mut p = product << 1;
        if self.macsr & RT != 0 {
            let low = p & 0xFF_FFFF;
            if low > 0x80_0000 || (low == 0x80_0000 && p & (1 << 24) != 0) {
                p += 1 << 24;
            }
        }
        let p = (p >> 24) as i64;
        let acc = sext48(self.full(n));
        let sum = if op.subtract { acc - p } else { acc + p };
        if !fits(sum, 48) {
            if omc {
                let v = if sum >= 0 {
                    0x007F_FFFF_FF00
                } else {
                    0xFF80_0000_0000
                };
                return (v, true);
            }
            return (sum as u64, true);
        }
        (sum as u64, false)
    }
}
