//! Rapid GPIO (RM ch. 16), minimal: the 16 pins' direction, enable and output latch, and inputs the host drives.

use crate::device::{Device, DeviceCx};
use crate::mem::Size;

/// The write map's offsets (RM Table 16-3): DIR, DATA, ENB, CLR, SET, TOG. Writes at +0x8 and +0xC change nothing.
const DIR: u32 = 0x0;
const DATA: u32 = 0x2;
const ENB: u32 = 0x4;
const CLR: u32 = 0x6;
const SET: u32 = 0xA;
const TOG: u32 = 0xE;

/// The RGPIO module: 16 pins at 0x8C00_0000 (RM Table 1-2), one 16-bit register per name (RM 16.3).
///
/// - A write goes to DIR (+0), DATA (+2), ENB (+4), or clears (CLR +6), sets (SET +0xA) or toggles (TOG +0xE) bits of
///   DATA; the writes at +8 and +0xC, and past +0xF, are dropped (RM 16.3).
/// - A read gives DIR at +0, +8 and +0xC, ENB at +4, and DATA at +2, +6, +0xA and +0xE (RM Table 16-4). DATA reads
///   the host's input level ([`Rgpio::set_input`]) for a pin that is enabled and an input, else the output latch (RM
///   16.3.3 leaves a disabled pin's read undefined; the latch is taken).
/// - Reads past +0xF are 0.
///
/// Inputs are 0 until the host sets them.
pub struct Rgpio {
    dir: u16,
    data: u16,
    enb: u16,
    input: u16,
}

impl Rgpio {
    /// An RGPIO at reset (every register 0) with all inputs low.
    pub fn new() -> Rgpio {
        Rgpio {
            dir: 0,
            data: 0,
            enb: 0,
            input: 0,
        }
    }

    /// Drive the 16 input pins to `levels` (bit n: pin n high); they show in DATA for the pins that are enabled
    /// inputs.
    pub fn set_input(&mut self, levels: u16) {
        self.input = levels;
    }

    fn data_read(&self) -> u16 {
        let sampled = self.enb & !self.dir;
        (self.input & sampled) | (self.data & !sampled)
    }

    fn read_word(&self, offset: u32) -> u16 {
        match offset & !1 {
            0x0 | 0x8 | 0xC => self.dir,
            0x4 => self.enb,
            0x2 | 0x6 | 0xA | 0xE => self.data_read(),
            _ => 0,
        }
    }

    /// A write of `v` over the byte lane `mask` (0xFF00 or 0x00FF) of the register at `reg`.
    fn write_lane(&mut self, reg: u32, v: u16, mask: u16) {
        let merge = |old: u16| (old & !mask) | (v & mask);
        match reg {
            DIR => self.dir = merge(self.dir),
            DATA => self.data = merge(self.data),
            ENB => self.enb = merge(self.enb),
            CLR => self.data &= !mask | v,
            SET => self.data |= v & mask,
            TOG => self.data ^= v & mask,
            _ => {}
        }
    }
}

impl Default for Rgpio {
    fn default() -> Rgpio {
        Rgpio::new()
    }
}

impl Device for Rgpio {
    fn read(&mut self, offset: u32, size: Size, _cx: &mut DeviceCx) -> u32 {
        (0..size.bytes()).fold(0, |v, i| {
            let at = offset + i;
            let w = self.read_word(at);
            v << 8 | u32::from(if at & 1 == 0 { (w >> 8) as u8 } else { w as u8 })
        })
    }

    fn write(&mut self, offset: u32, size: Size, value: u32, _cx: &mut DeviceCx) {
        let n = size.bytes();
        for i in 0..n {
            let at = offset + i;
            let b = u16::from((value >> (8 * (n - 1 - i))) as u8);
            let (v, mask) = if at & 1 == 0 {
                (b << 8, 0xFF00)
            } else {
                (b, 0x00FF)
            };
            self.write_lane(at & !1 & 0xF, v, mask);
        }
    }

    fn poll_counted(&self) -> bool {
        false
    }
}
