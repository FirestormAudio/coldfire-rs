//! Lazily evaluated condition codes.
//!
//! A flag-setting instruction records what it did ([`Kind`], its size, operands and result) and the flags are
//! worked out only when something reads them. X is held apart: it either has a fixed value or follows the C of
//! the recorded operation, so instructions that leave X alone (CFPRM ch. 4: MOVE, logic, ...) never compute it.
//!
//! Adding a kind: add a variant to [`Kind`], a recorder that stores its inputs, and its N Z V C in
//! [`Flags::nzvc`].

use crate::cpu::ea::size_mask;
use crate::mem::Size;

pub const X: u8 = 0x10;
pub const N: u8 = 0x08;
pub const Z: u8 = 0x04;
pub const V: u8 = 0x02;
pub const C: u8 = 0x01;

/// The operation whose flags are pending.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    /// N Z V C are stored in the low four bits of `res`.
    Known,
    /// N and Z from the result; V and C clear (MOVE, MOVEQ, MVS, MVZ, logic).
    Logic,
    /// `res = dst + src`.
    Add,
    /// `res = dst - src`.
    Sub,
}

/// The condition-code register, lazily.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Flags {
    kind: Kind,
    size: Size,
    src: u32,
    dst: u32,
    res: u32,
    /// X, unless `x_is_c`.
    x: bool,
    /// X is the C of the pending operation.
    x_is_c: bool,
}

impl Default for Flags {
    fn default() -> Self {
        Flags {
            kind: Kind::Known,
            size: Size::Long,
            src: 0,
            dst: 0,
            res: 0,
            x: false,
            x_is_c: false,
        }
    }
}

impl Flags {
    /// Fix X before recording an operation that does not change it.
    #[inline(always)]
    fn keep_x(&mut self) {
        if self.x_is_c {
            self.x = self.nzvc() & C != 0;
            self.x_is_c = false;
        }
    }

    /// N and Z from `res` at `size`, V and C cleared, X unchanged.
    #[inline(always)]
    pub fn logic(&mut self, size: Size, res: u32) {
        self.keep_x();
        self.kind = Kind::Logic;
        self.size = size;
        self.res = res;
    }

    /// Flags of `dst + src = res` at `size`; X = C.
    #[inline(always)]
    pub fn add(&mut self, size: Size, src: u32, dst: u32, res: u32) {
        *self = Flags {
            kind: Kind::Add,
            size,
            src,
            dst,
            res,
            x: false,
            x_is_c: true,
        };
    }

    /// Flags of `dst - src = res` at `size`; X = C.
    #[inline(always)]
    pub fn sub(&mut self, size: Size, src: u32, dst: u32, res: u32) {
        *self = Flags {
            kind: Kind::Sub,
            size,
            src,
            dst,
            res,
            x: false,
            x_is_c: true,
        };
    }

    /// Set X N Z V C from the low five bits of `ccr`.
    pub fn set_ccr(&mut self, ccr: u8) {
        *self = Flags {
            kind: Kind::Known,
            size: Size::Long,
            src: 0,
            dst: 0,
            res: (ccr & 0x0F) as u32,
            x: ccr & X != 0,
            x_is_c: false,
        };
    }

    /// N Z V C in bits 3-0.
    pub fn nzvc(&self) -> u8 {
        let mask = size_mask(self.size);
        let msb = mask ^ (mask >> 1);
        let (s, d, r) = (self.src, self.dst, self.res);
        let nz = |r: u32| (if r & msb != 0 { N } else { 0 }) | (if r & mask == 0 { Z } else { 0 });
        let bit = |v: u32, f: u8| if v & msb != 0 { f } else { 0 };
        match self.kind {
            Kind::Known => self.res as u8 & 0x0F,
            Kind::Logic => nz(r),
            Kind::Add => nz(r) | bit((s ^ r) & (d ^ r), V) | bit((s & d) | (!r & (s | d)), C),
            Kind::Sub => nz(r) | bit((s ^ d) & (r ^ d), V) | bit((s & !d) | (r & !d) | (s & r), C),
        }
    }

    /// X N Z V C in bits 4-0.
    pub fn ccr(&self) -> u8 {
        let nzvc = self.nzvc();
        let x = if self.x_is_c { nzvc & C != 0 } else { self.x };
        nzvc | if x { X } else { 0 }
    }

    pub fn x(&self) -> bool {
        self.ccr() & X != 0
    }
}
