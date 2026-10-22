//! The edge port EPORT0 (RM ch. 18).

use super::{crosses_register, crossing, lane_merge, lane_read};
use crate::device::{Device, DeviceCx};
use crate::mem::Size;
use crate::stop::Stop;

/// The register longwords: EPPAR, EPDDR and EPIER at +0; EPDR, EPPDR and EPFR at +4.
const PAR: u32 = 0x0;
const DATA: u32 = 0x4;

/// The edge port: EPPAR at +0, EPDDR at +2, EPIER at +3, EPDR at +4, EPPDR at +5, EPFR at +6.
///
/// - EPPAR (reset 0): per pin, level-sensitive (active low), rising, falling or both edges (RM Table 18-3).
/// - EPIER (reset 0): pin *n* asserts source *n* of interrupt controller `controller` while its EPIE bit is set and
///   either its EPF bit is set (edge modes) or the pin is low (level mode) (RM Table 18-4).
/// - EPFR (reset 0): set by the selected edge on a pin in an edge mode, whatever EPIER holds; writing 1 clears a
///   bit (RM Table 18-5). Pin changes in level mode leave it alone.
/// - EPDDR, EPDR and EPPDR: RM Table 18-2 lists only EPPAR, EPIER and EPFR. guess: they sit where other ColdFire
///   edge ports have them (EPDDR +2, EPDR +4, EPPDR +5, all reset 0); a pin with its EPDDR bit set is driven by
///   its EPDR bit, and EPPDR reads the levels on the pins, from which the edges are detected too.
///
/// The pins IRQ1-IRQ7 are inputs the host drives with [`Eport::set_pin`], inside
/// [`Machine::with_device`](crate::Machine::with_device); a change asserts or drops the interrupt at once (the
/// synchronizer's delay, RM 18.3, is not modelled). guess: every pin starts high (inactive); which pins a board
/// wires is the board's. IRQ0 has no interrupt source; its register bits are stored.
///
/// Byte and word accesses address their lanes; a longword access is a bus error (RM 18.4) and is refused with
/// [`Stop::Unmodelled`]. The supervisor-only access of EPPAR and EPIER (RM Table 18-2) is not checked. The rest of
/// the window reads 0 and ignores writes. The registers change only by writes and by the host, so reads of them are
/// poll-counted: a program waiting on a pin the host never changes stops with [`Stop::Poll`].
#[derive(Debug)]
pub struct Eport {
    controller: usize,
    eppar: u16,
    epddr: u8,
    epier: u8,
    epdr: u8,
    epfr: u8,
    /// The levels the host drives on the pins.
    inputs: u8,
}

impl Eport {
    /// The edge port, driving sources 1-7 of interrupt controller `controller` (its
    /// [`Intc::index`](super::Intc::index)).
    pub fn new(controller: usize) -> Eport {
        Eport {
            controller,
            eppar: 0,
            epddr: 0,
            epier: 0,
            epdr: 0,
            epfr: 0,
            inputs: 0xFF,
        }
    }

    /// Drive pin IRQ`n` (1-7) high or low, at `cx.now`. Panics when `n` is not 1-7.
    pub fn set_pin(&mut self, n: u8, high: bool, cx: &mut DeviceCx) {
        assert!((1..=7).contains(&n), "no edge port pin IRQ{n}: IRQ1-IRQ7");
        let was = self.pins();
        if high {
            self.inputs |= 1 << n;
        } else {
            self.inputs &= !(1 << n);
        }
        self.changed(was, cx);
    }

    /// The level the host drives on pin IRQ`n` (0-7).
    pub fn pin(&self, n: u8) -> bool {
        self.inputs >> n & 1 != 0
    }

    /// The levels on the pins, as EPPDR reads them.
    pub fn pins(&self) -> u8 {
        (self.inputs & !self.epddr) | (self.epdr & self.epddr)
    }

    /// The mode of pin `n`: 0 level, 1 rising, 2 falling, 3 both (RM Table 18-3).
    fn mode(&self, n: u8) -> u16 {
        self.eppar >> (2 * n) & 3
    }

    /// Latch the edges from `was` to the levels now, then drive the lines.
    fn changed(&mut self, was: u8, cx: &mut DeviceCx) {
        let now = self.pins();
        for n in 0..8 {
            let (a, b) = (was >> n & 1, now >> n & 1);
            let hit = match self.mode(n) {
                1 => a == 0 && b == 1,
                2 => a == 1 && b == 0,
                3 => a != b,
                _ => false,
            };
            if hit {
                self.epfr |= 1 << n;
            }
        }
        self.drive(cx);
    }

    fn drive(&self, cx: &mut DeviceCx) {
        let pins = self.pins();
        for n in 1..8 {
            let active = if self.mode(n) == 0 {
                pins >> n & 1 == 0
            } else {
                self.epfr >> n & 1 != 0
            };
            let on = self.epier >> n & 1 != 0 && active;
            cx.irq.set(self.controller, n, on);
        }
    }

    fn register(&self, reg: u32) -> u32 {
        match reg {
            PAR => u32::from(self.eppar) << 16 | u32::from(self.epddr) << 8 | u32::from(self.epier),
            DATA => {
                u32::from(self.epdr) << 24
                    | u32::from(self.pins()) << 16
                    | u32::from(self.epfr) << 8
            }
            _ => 0,
        }
    }
}

impl Device for Eport {
    fn read(&mut self, offset: u32, size: Size, cx: &mut DeviceCx) -> u32 {
        if size == Size::Long {
            cx.refuse(
                Stop::Unmodelled,
                "Edge port: longword read (a bus error, RM 18.4)",
            );
            return 0;
        }
        if crosses_register(offset, size) {
            cx.refuse(
                Stop::Unmodelled,
                crossing("Edge port", offset, size, "read"),
            );
            return 0;
        }
        lane_read(self.register(offset & !3), offset, size)
    }

    fn write(&mut self, offset: u32, size: Size, value: u32, cx: &mut DeviceCx) {
        if size == Size::Long {
            return cx.refuse(
                Stop::Unmodelled,
                "Edge port: longword write (a bus error, RM 18.4)",
            );
        }
        if crosses_register(offset, size) {
            return cx.refuse(
                Stop::Unmodelled,
                crossing("Edge port", offset, size, "write"),
            );
        }
        let reg = offset & !3;
        let covered = lane_merge(0, offset, size, u32::MAX);
        let new = lane_merge(self.register(reg), offset, size, value);
        let was = self.pins();
        match reg {
            PAR => {
                self.eppar = (new >> 16) as u16;
                self.epddr = (new >> 8) as u8;
                self.epier = new as u8;
            }
            DATA => {
                self.epdr = (new >> 24) as u8;
                if covered & 0xFF00 != 0 {
                    self.epfr &= !(lane_merge(0, offset, size, value) >> 8) as u8;
                }
            }
            _ => return,
        }
        self.changed(was, cx);
    }
}
