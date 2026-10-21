//! The PLL's registers in the clock module (RM ch. 8).

use super::{crosses_register, crossing, lane_merge, lane_read};
use crate::device::{Device, DeviceCx};
use crate::mem::Size;
use crate::stop::Stop;

const PLL_CR: u32 = 0x0;
const PLL_DR: u32 = 0x4;
const PLL_SR: u32 = 0x8;
/// The defined bits of PLL_CR: LOCIRQ, LOCRE, LOCEN, LOLIRQ, LOLRE, LOLEN, REFDIV, FBKDIV (RM Figure 8-3).
const CR_BITS: u32 = 0x0007_773F;
/// The defined bits of PLL_DR: OUTDIV5..OUTDIV1 (RM Figure 8-4).
const DR_BITS: u32 = 0x03FF_7FFF;
/// PLL_SR LOCKS and LOCK (RM Table 8-5).
const LOCKS: u32 = 1 << 5;
const LOCK: u32 = 1 << 4;
/// PLL_SR LOLF, write-1-to-clear (RM Table 8-5).
const LOLF: u32 = 1 << 6;
/// PLL_CR LOLIRQ, LOLRE and LOLEN (RM Figure 8-3): with LOLEN set, a loss of lock is detected, and LOLIRQ
/// sets LOLF and requests an interrupt, LOLRE requests a reset; with LOLEN clear, both have no effect.
const LOLIRQ: u32 = 1 << 14;
const LOLRE: u32 = 1 << 13;
const LOLEN: u32 = 1 << 12;

/// The PLL (RM ch. 8), mapped at 0xFC0C_0000: locked, unless the host says otherwise ([`Pll::set_locked`]).
///
/// Registers:
/// - PLL_CR (+0) and PLL_DR (+4): their reset values come from the reset configuration (RM Figures 8-3 and 8-4:
///   RCON, the serial boot facility's SBF_RCON or the parallel configuration), so they are the board's and the
///   constructor's. Writes are stored, the reserved bits reading 0 (RM Figures 8-3 and 8-4). A narrow write to
///   PLL_DR is refused with [`Stop::Unmodelled`] (RM 8.2.2: it takes a single 32-bit write); other narrow accesses
///   use the register's big-endian byte lanes.
/// - PLL_SR (+8): LOCK and LOCKS (0x30), measured on an MCF54415 after reset. With PLL_CR\[LOLEN\] set, both
///   clear while the host has the PLL unlocked ([`Pll::set_locked`]), and LOCKS stays clear after (RM 8.3.4); with
///   LOLEN clear, LOCK is time-based and stays set until reset (RM Table 8-5, RM 8.3.3), so the loss is not seen.
///   LOLF is set when the host takes the lock away with PLL_CR\[LOLIRQ\] and \[LOLEN\] both set (RM Table 8-5,
///   RM 8.3.4.2), and a write of 1 clears it. The model never loses its clock, so LOCF and LOC read 0. guess:
///   there is no relock time; a new CR or DR leaves the PLL locked.
///
/// The rest of the window reads 0 and ignores writes. An access that crosses a register (from a bus master) is
/// refused with [`Stop::Unmodelled`]. The interrupt enables PLL_CR\[LOCIRQ\] and \[LOLIRQ\] are stored; the model
/// never loses its clock, and loses lock only at the host's word. A loss of lock with PLL_CR\[LOLEN\] set and
/// \[LOLRE\] or \[LOLIRQ\] set stops the next run, before its first instruction, with [`Stop::Unmodelled`]: the
/// reset request (RM 8.3.4.1) and LOLF's interrupt request (INTC2 source 12, RM Table 17-17) are not modelled.
/// Nothing derives a clock from the dividers: time is the board's, in ticks. Every accepted write is recorded
/// ([`Pll::writes`]) unless [`Pll::record`] turned the log off.
#[derive(Debug)]
pub struct Pll {
    cr: u32,
    dr: u32,
    writes: Vec<(u32, u32)>,
    record: bool,
    locked: bool,
    /// Lock was lost since reset: LOCKS stays clear (RM 8.3.4).
    lost: bool,
    lolf: bool,
    /// A loss of lock asked for a reset or an interrupt the model does not have: the detail the run stops with.
    pending: Option<&'static str>,
}

impl Pll {
    /// The PLL with the PLL_CR and PLL_DR values the board's reset configuration (or the boot stage before) left.
    pub fn new(cr: u32, dr: u32) -> Pll {
        Pll {
            cr,
            dr,
            writes: Vec::new(),
            record: true,
            locked: true,
            lost: false,
            lolf: false,
            pending: None,
        }
    }

    /// Every accepted write so far: (offset, value), in order.
    pub fn writes(&self) -> &[(u32, u32)] {
        &self.writes
    }

    /// Every accepted write so far, emptying the log.
    pub fn take_writes(&mut self) -> Vec<(u32, u32)> {
        std::mem::take(&mut self.writes)
    }

    /// Whether writes are logged (default on). Off changes only the log, never the registers.
    pub fn record(&mut self, on: bool) {
        self.record = on;
    }

    /// Host knob: whether the PLL is locked (default on). Off models a PLL that loses lock (or never locks).
    ///
    /// With PLL_CR\[LOLEN\] set, PLL_SR\[LOCK\] and \[LOCKS\] read 0 (RM 8.3.4). On again restores LOCK only:
    /// LOCKS is sticky and "remains cleared" after the PLL re-locks (RM 8.3.4); only a reset sets it again, and the
    /// model has no reset of the PLL. guess: a write to PLL_SR does not restore LOCKS (the RM says to clear it by
    /// writing a one, which leaves it 0). Taking the lock away while PLL_CR\[LOLIRQ\] is also set sets
    /// PLL_SR\[LOLF\] (RM Table 8-5), which stays set, lock or no lock, until a write of 1 clears it. Taking it away
    /// while PLL_CR\[LOLRE\] or \[LOLIRQ\] is set stops the next run with [`Stop::Unmodelled`]: "PLL: loss-of-lock
    /// reset not modelled" (LOLRE, checked first) or "PLL: loss-of-lock interrupt request (INTC2 source 12) not
    /// modelled" (LOLIRQ).
    ///
    /// With LOLEN clear there is no loss-of-lock detection: LOCK, once set, "remains set until reset is asserted"
    /// (RM Table 8-5), LOLIRQ and LOLRE have no effect (RM Figure 8-3), and turning the knob off changes nothing.
    /// guess: LOCKS stays set too (no loss of lock is detected to clear it), and a later write that sets LOLEN
    /// does not detect a loss made while it was clear.
    pub fn set_locked(&mut self, on: bool) {
        if !on && self.cr & LOLEN == 0 {
            return;
        }
        if self.locked && !on {
            if self.cr & LOLIRQ != 0 {
                self.lolf = true;
            }
            if self.cr & LOLRE != 0 {
                self.pending = Some("PLL: loss-of-lock reset not modelled");
            } else if self.cr & LOLIRQ != 0 {
                self.pending =
                    Some("PLL: loss-of-lock interrupt request (INTC2 source 12) not modelled");
            }
        }
        self.locked = on;
        self.lost |= !on;
    }

    fn register(&self, reg: u32) -> u32 {
        match reg {
            PLL_CR => self.cr,
            PLL_DR => self.dr,
            PLL_SR => {
                (if self.locked { LOCK } else { 0 })
                    | (if self.lost { 0 } else { LOCKS })
                    | (if self.lolf { LOLF } else { 0 })
            }
            _ => 0,
        }
    }
}

impl Device for Pll {
    fn read(&mut self, offset: u32, size: Size, cx: &mut DeviceCx) -> u32 {
        if crosses_register(offset, size) {
            cx.refuse(Stop::Unmodelled, crossing("PLL", offset, size, "read"));
            return 0;
        }
        lane_read(self.register(offset & !3), offset, size)
    }

    fn write(&mut self, offset: u32, size: Size, value: u32, cx: &mut DeviceCx) {
        if crosses_register(offset, size) {
            return cx.refuse(Stop::Unmodelled, crossing("PLL", offset, size, "write"));
        }
        let reg = offset & !3;
        if reg == PLL_DR && size != Size::Long {
            return cx.refuse(
                Stop::Unmodelled,
                format!(
                    "PLL: {}-byte write to PLL_DR (it takes a single 32-bit write)",
                    size.bytes()
                ),
            );
        }
        if self.record {
            self.writes.push((offset, value));
        }
        let new = lane_merge(self.register(reg), offset, size, value);
        match reg {
            PLL_CR => self.cr = new & CR_BITS,
            PLL_DR => self.dr = new & DR_BITS,
            PLL_SR if lane_merge(0, offset, size, value) & LOLF != 0 => self.lolf = false,
            _ => {}
        }
    }

    /// Due at once while a loss of lock waits to stop the run ([`Pll::set_locked`]).
    fn next_event(&self) -> Option<u64> {
        self.pending.map(|_| 0)
    }

    fn advance(&mut self, _now: u64, cx: &mut DeviceCx) {
        if let Some(why) = self.pending.take() {
            cx.halt(Stop::Unmodelled, why);
        }
    }

    fn poll_counted(&self) -> bool {
        false
    }

    /// No interrupts: the only event stops the run.
    fn wake_sources(&self, _out: &mut Vec<(usize, u8)>) -> bool {
        true
    }
}
