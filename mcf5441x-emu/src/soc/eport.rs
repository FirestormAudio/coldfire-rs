//! The edge port EPORT0 (RM ch. 18).

use super::{crosses_register, crossing, lane_merge, lane_read};
use crate::device::{Device, DeviceCx};
use crate::mem::Size;
use crate::stop::Stop;

/// The register longwords: EPPAR and EPIER at +0, EPFR at +4.
const PAR: u32 = 0x0;
const FLAGS: u32 = 0x4;

/// The edge port: EPPAR at +0, EPIER at +3, EPFR at +6 (RM Table 18-2).
///
/// - EPPAR (reset 0): per pin, level-sensitive (active low), rising, falling or both edges (RM Table 18-3).
/// - EPIER (reset 0): pin *n* asserts source *n* of interrupt controller `controller` while its EPIE bit is set and
///   either its EPF bit is set, in any mode, or it is level-sensitive and low (RM Table 18-4 lists the two
///   conditions; the first has no mode qualifier, so an EPF bit left from an edge mode still requests in level
///   mode).
/// - EPFR (reset 0): set by the selected edge on a pin in an edge mode, whatever EPIER holds; writing 1 clears a
///   bit (RM Table 18-5). Pin changes in level mode leave it alone.
///
/// There is no EPDDR, EPDR or EPPDR (as RM Table 18-2 lists): +2, +4, +5 and +7 read 0 and drop writes, and every
/// byte of +0..+7 reads 0 after reset, while EPPAR and EPIER read back what was written; the rest of the window
/// repeats those eight bytes every 8 (writes at +8 and +0x10 landed in EPPAR, +0x3FFE read EPFR's word; measured on
/// an MCF54415). The pins are inputs only. A program reads a pin's level through the interrupt controller: with
/// the pin level-sensitive and its EPIE bit set, its source requests while it is low.
///
/// On the part the edge port sees a pad only while the pad is muxed to IRQ*n* (measured on an MCF54415); the model
/// sees every pin whatever its PAR field, which it doesn't consult.
///
/// The pins IRQ1-IRQ7 are inputs the host drives with [`Eport::set_pin`], inside
/// [`Machine::with_device`](crate::Machine::with_device); a change asserts or drops the interrupt at once (the
/// synchronizer's delay, RM 18.3, is not modelled). guess: every pin starts high (inactive); which pins a board
/// wires is the board's. IRQ0 has no interrupt source; its register bits are stored.
///
/// Byte and word accesses address their lanes; a longword access is a bus error (RM 18.4) and is refused with
/// [`Stop::Unmodelled`]. The supervisor-only access of EPPAR and EPIER (RM Table 18-2) is not checked. The
/// registers change only by writes and by the host, so reads of them are poll-counted: a program waiting on an
/// edge the host never makes stops with [`Stop::Poll`].
///
/// While its clock gate is closed ([`Device::gate`]) [`Eport::set_pin`] still changes the pin levels, but no edge
/// is latched into EPFR, and a level-sensitive pin keeps requesting while it is low. guess: not measured; this
/// carries RM 18.2's stop mode over to a closed gate (the RM says that in stop mode "no clocks are available to
/// perform the edge-detect function. Only the level-detect logic is active"; that a gated module is the same is
/// this model's reading). Writes are dropped while gated, so the lines always follow EPIER, EPFR and the pins as
/// they are; when the gate opens nothing is re-latched: a pin that changed while it was closed made no edge
/// (guess).
#[derive(Debug)]
pub struct Eport {
    controller: usize,
    eppar: u16,
    epier: u8,
    epfr: u8,
    /// The levels the host drives on the pins.
    inputs: u8,
    /// Its clock gate is closed: no edges are detected.
    gated: bool,
}

impl Eport {
    /// The edge port, driving sources 1-7 of interrupt controller `controller` (its
    /// [`Intc::index`](super::Intc::index)).
    pub fn new(controller: usize) -> Eport {
        Eport {
            controller,
            eppar: 0,
            epier: 0,
            epfr: 0,
            inputs: 0xFF,
            gated: false,
        }
    }

    /// Drive pin IRQ`n` (1-7) high or low, at `cx.now`; while the edge port's clock gate is closed no edge is
    /// latched (guess, see [`Eport`]). Panics when `n` is not 1-7.
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

    /// The levels on the pins IRQ0-IRQ7, one bit each.
    pub fn pins(&self) -> u8 {
        self.inputs
    }

    /// The mode of pin `n`: 0 level, 1 rising, 2 falling, 3 both (RM Table 18-3).
    fn mode(&self, n: u8) -> u16 {
        self.eppar >> (2 * n) & 3
    }

    /// Latch the edges from `was` to the levels now, then drive the lines.
    fn changed(&mut self, was: u8, cx: &mut DeviceCx) {
        let now = self.pins();
        for n in (0..8).filter(|_| !self.gated) {
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
            let active = self.epfr >> n & 1 != 0 || (self.mode(n) == 0 && pins >> n & 1 == 0);
            let on = self.epier >> n & 1 != 0 && active;
            cx.irq.set(self.controller, n, on);
        }
    }

    fn register(&self, reg: u32) -> u32 {
        match reg {
            PAR => u32::from(self.eppar) << 16 | u32::from(self.epier),
            FLAGS => u32::from(self.epfr) << 8,
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
        let offset = offset & 7;
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
        let offset = offset & 7;
        let reg = offset & !3;
        let covered = lane_merge(0, offset, size, u32::MAX);
        let new = lane_merge(self.register(reg), offset, size, value);
        match reg {
            PAR => {
                self.eppar = (new >> 16) as u16;
                self.epier = new as u8;
            }
            FLAGS => {
                if covered & 0xFF00 != 0 {
                    self.epfr &= !(lane_merge(0, offset, size, value) >> 8) as u8;
                }
            }
            _ => return,
        }
        self.drive(cx);
    }

    // guess: a gated edge port latches no edges; its level-sensitive pins still request (see the type's doc).
    fn gate(&mut self, closed: bool, _now: u64) {
        self.gated = closed;
    }
}
