//! The Enhanced Multiply-Accumulate unit (RM ch. 5): four 48-bit accumulators, MACSR modes, `mac`/`msac`.
//! Read an accumulator with `read` (32 bits per the MACSR mode) or `take` (read and clear).
//!
//! The EMAC is global state, and `coldfire_rt::exception_entry!` saves only d0/d1/a0/a1: an interrupt handler
//! that uses it changes MACSR and the accumulators under the interrupted code. Use it from one context, or save
//! and restore MACSR, the accumulators and their extension bytes around a handler's use (RM ch. 5).

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct Macsr(pub u32);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    SignedInteger,
    UnsignedInteger,
    Fractional,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Acc {
    A0,
    A1,
    A2,
    A3,
}

impl Macsr {
    pub const EV: u32 = 1 << 0;
    pub const V: u32 = 1 << 1;
    pub const Z: u32 = 1 << 2;
    pub const N: u32 = 1 << 3;
    pub const RT: u32 = 1 << 4;
    pub const FI: u32 = 1 << 5;
    pub const SU: u32 = 1 << 6;
    pub const OMC: u32 = 1 << 7;

    pub const fn with_mode(self, m: Mode) -> Macsr {
        let base = self.0 & !(Self::SU | Self::FI);
        Macsr(match m {
            Mode::SignedInteger => base,
            Mode::UnsignedInteger => base | Self::SU,
            Mode::Fractional => base | Self::FI,
        })
    }
    pub const fn saturating(self, on: bool) -> Macsr {
        Macsr(if on { self.0 | Self::OMC } else { self.0 & !Self::OMC })
    }
    pub const fn rounding(self, on: bool) -> Macsr {
        Macsr(if on { self.0 | Self::RT } else { self.0 & !Self::RT })
    }
    /// The product/accumulation overflow flag of `acc` (bits 8–11).
    pub const fn pav(self, acc: Acc) -> bool {
        self.0 & (1 << (8 + acc as u32)) != 0
    }
}

#[cfg(target_arch = "m68k")]
mod ops {
    use super::{Acc, Macsr};
    use core::arch::asm;

    pub fn read_macsr() -> Macsr {
        let v: u32;
        unsafe { asm!("move.l %macsr,{0}", out(reg_data) v, options(nostack)) };
        Macsr(v)
    }
    pub fn write_macsr(m: Macsr) {
        unsafe { asm!("move.l {0},%macsr", in(reg_data) m.0, options(nostack)) };
    }

    /// The accumulator number is part of the mnemonic's operand: one `asm!` per accumulator, the template
    /// ending in `%acc` and the number appended.
    macro_rules! per_acc {
        ($acc:expr, $tmpl:literal, $($operands:tt)*) => {
            unsafe {
                match $acc {
                    Acc::A0 => asm!(concat!($tmpl, "0"), $($operands)*),
                    Acc::A1 => asm!(concat!($tmpl, "1"), $($operands)*),
                    Acc::A2 => asm!(concat!($tmpl, "2"), $($operands)*),
                    Acc::A3 => asm!(concat!($tmpl, "3"), $($operands)*),
                }
            }
        };
    }

    /// Load `v` into the accumulator (sign- or zero-extended per MACSR; clears its PAV flag).
    pub fn load(acc: Acc, v: u32) {
        per_acc!(acc, "move.l {0},%acc", in(reg_data) v, options(nostack));
    }
    /// The accumulator's 32-bit value (rounded or truncated per MACSR in fractional mode).
    pub fn read(acc: Acc) -> u32 {
        let v: u32;
        unsafe {
            match acc {
                Acc::A0 => asm!("move.l %acc0,{0}", out(reg_data) v, options(nostack)),
                Acc::A1 => asm!("move.l %acc1,{0}", out(reg_data) v, options(nostack)),
                Acc::A2 => asm!("move.l %acc2,{0}", out(reg_data) v, options(nostack)),
                Acc::A3 => asm!("move.l %acc3,{0}", out(reg_data) v, options(nostack)),
            }
        }
        v
    }
    /// Read the accumulator and clear it.
    pub fn take(acc: Acc) -> u32 {
        let v: u32;
        unsafe {
            match acc {
                Acc::A0 => asm!("movclr.l %acc0,{0}", out(reg_data) v, options(nostack)),
                Acc::A1 => asm!("movclr.l %acc1,{0}", out(reg_data) v, options(nostack)),
                Acc::A2 => asm!("movclr.l %acc2,{0}", out(reg_data) v, options(nostack)),
                Acc::A3 => asm!("movclr.l %acc3,{0}", out(reg_data) v, options(nostack)),
            }
        }
        v
    }
    /// acc += a × b (32-bit operands).
    pub fn mac(acc: Acc, a: u32, b: u32) {
        per_acc!(acc, "mac.l {0},{1},%acc", in(reg_data) a, in(reg_data) b, options(nostack));
    }
    /// acc -= a × b (32-bit operands).
    pub fn msac(acc: Acc, a: u32, b: u32) {
        per_acc!(acc, "msac.l {0},{1},%acc", in(reg_data) a, in(reg_data) b, options(nostack));
    }
}
#[cfg(target_arch = "m68k")]
pub use ops::*;

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn macsr_modes_and_flags_use_the_manuals_bits() {
        assert_eq!(Macsr(0).with_mode(Mode::UnsignedInteger).0, 1 << 6);
        assert_eq!(Macsr(0).with_mode(Mode::Fractional).0, 1 << 5);
        assert_eq!(Macsr(0xFF).with_mode(Mode::SignedInteger).0, 0xFF & !(1 << 6 | 1 << 5));
        assert_eq!(Macsr(0).saturating(true).rounding(true).0, 1 << 7 | 1 << 4);
        assert!(Macsr(1 << 10).pav(Acc::A2) && !Macsr(1 << 10).pav(Acc::A1));
    }
}
