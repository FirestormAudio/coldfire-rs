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

/// The PLL (RM ch. 8), mapped at 0xFC0C_0000, always locked.
///
/// Registers:
/// - PLL_CR (+0) and PLL_DR (+4): their reset values come from the reset configuration (RM Figures 8-3 and 8-4:
///   RCON, the serial boot facility's SBF_RCON or the parallel configuration), so they are the board's and the
///   constructor's. Writes are stored, the reserved bits reading 0 (RM Figures 8-3 and 8-4). A narrow write to
///   PLL_DR is refused with [`Stop::Unmodelled`] (RM 8.2.2: it takes a single 32-bit write); other narrow accesses
///   use the register's big-endian byte lanes.
/// - PLL_SR (+8): LOCK and LOCKS (0x30), measured on an MCF54415 after reset. LOCF and LOLF are write-1-to-clear
///   and LOC is status (RM Table 8-5); the model never loses lock or clock, so they read 0 and no write changes SR.
///   guess: there is no relock time; a new CR or DR leaves the PLL locked.
///
/// The rest of the window reads 0 and ignores writes. An access that crosses a register (from a bus master) is
/// refused with [`Stop::Unmodelled`]. The interrupt enables PLL_CR\[LOCIRQ\] and \[LOLIRQ\] are stored, but the
/// model never loses clock or lock, so no interrupt is raised (and none is wired). Nothing derives a clock from
/// the dividers: time is the board's, in ticks. Every accepted write is recorded ([`Pll::writes`]).
#[derive(Debug)]
pub struct Pll {
    cr: u32,
    dr: u32,
    writes: Vec<(u32, u32)>,
}

impl Pll {
    /// The PLL with the PLL_CR and PLL_DR values the board's reset configuration (or the boot stage before) left.
    pub fn new(cr: u32, dr: u32) -> Pll {
        Pll {
            cr,
            dr,
            writes: Vec::new(),
        }
    }

    /// Every accepted write so far: (offset, value), in order.
    pub fn writes(&self) -> &[(u32, u32)] {
        &self.writes
    }

    fn register(&self, reg: u32) -> u32 {
        match reg {
            PLL_CR => self.cr,
            PLL_DR => self.dr,
            PLL_SR => LOCK | LOCKS,
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
        self.writes.push((offset, value));
        let new = lane_merge(self.register(reg), offset, size, value);
        match reg {
            PLL_CR => self.cr = new & CR_BITS,
            PLL_DR => self.dr = new & DR_BITS,
            _ => {}
        }
    }

    fn poll_counted(&self) -> bool {
        false
    }

    /// No events, no interrupts.
    fn wake_sources(&self, _out: &mut Vec<(usize, u8)>) -> bool {
        true
    }
}
