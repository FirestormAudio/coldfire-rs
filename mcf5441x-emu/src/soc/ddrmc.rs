//! The DDR SDRAM memory controller, DDRMC (RM ch. 21): its registers only; the DDR memory itself is RAM.

use super::{crosses_register, crossing, lane_merge, lane_read};
use crate::device::{Device, DeviceCx};
use crate::mem::Size;
use crate::stop::Stop;

/// DDR_CR09 START (RM Table 21-14).
const CR09: u32 = 0x24;
const START: u32 = 1 << 24;
/// DDR_CR25 INTACK, write-only (RM Table 21-30).
const CR25: u32 = 0x64;
const INTACK: u32 = 0x00FF_0000;
/// DDR_CR27 INTSTATUS (RM Table 21-32): bit 3 DRAM initialization complete, bit 8 the OR of bits 7:0.
const CR27: u32 = 0x6C;
const INIT_DONE: u32 = 1 << 3;
const INT_ANY: u32 = 1 << 8;
/// The read-only registers: CR27, CR30, CR44, CR53, CR55 (RM Table 21-4).
const READ_ONLY: [u32; 5] = [CR27, 0x78, 0xB0, 0xD4, 0xDC];
/// DDR_RCR; RST is write-only and reads 0 (RM Table 21-58).
const RCR: u32 = 0x180;
const RCR_RST: u32 = 1 << 30;
/// DDR_PADCR; bit 9 is reserved and reads 1 (RM Figure 21-56).
const PADCR: u32 = 0x1AC;
const PADCR_ONES: u32 = 1 << 9;

/// Reset values of CR00..CR63 that are not 0, measured on an MCF54415 with the controller's clock on. RM Table 21-4
/// agrees on CR13, CR20, CR21, CR23 and CR44; it lists 0 for CR03 and CR22.
const CR_RESET: [(usize, u32); 7] = [
    (3, 0x100),
    (13, 0x0002_0000),
    (20, 0x0C00_0000),
    (21, 0x400),
    (22, 0xD00),
    (23, 0x1000_0000),
    (44, 0x2040),
];
/// guess: RM Figure 21-56 resets PADCR's reserved bit 9 ("must be set") to 1; Table 21-4 lists 0. Not measured.
const PADCR_RESET: u32 = 0x200;

/// The DDR controller's registers (RM ch. 21), mapped at 0xFC0B_8000. Nothing behind them is modelled: the DDR
/// memory is plain RAM that works whatever the parameters say.
///
/// Registers, with their reset values:
/// - DDR_CR00..DDR_CR63 (+0x00..+0xFC): all 0 except CR03 0x100, CR13 0x0002_0000, CR20 0x0C00_0000, CR21 0x400,
///   CR22 0xD00, CR23 0x1000_0000 and CR44 0x2040 (measured on an MCF54415; RM Table 21-4 agrees except for CR03 and
///   CR22, which it lists as 0). Each reads back what was written, reserved bits included (guess: the RM's
///   "must be cleared" bits are not masked), except:
///   - the read-only CR27, CR30, CR44 (VERSION, 0x2040), CR53 and CR55 (RM Table 21-4) ignore writes;
///   - CR27 INTSTATUS bit 3 (DRAM initialization complete) sets when a write sets CR09\[START\] while it was clear
///     (the controller's initialization takes no time; guess: rewriting START while set does not restart it), with
///     bit 8 the OR of bits 7:0 (RM Table 21-32): 0x108 after START, measured on an MCF54415. And CR25\[INTACK\] (write-only, reads 0) clears the INTSTATUS
///     bits it names (RM Table 21-30).
/// - DDR_RCR (+0x180, 0): RST is write-only and reads 0 (RM Table 21-58); the rest reads back.
/// - DDR_PADCR (+0x1AC, 0x200; guess: RM Figure 21-56's bit 9, where Table 21-4 says 0): reads back, bit 9 always
///   1 (guess: the figure shows it read-only, "must be set").
///
/// The rest of the window reads 0 and ignores writes. Narrow accesses read and write the big-endian byte lanes of a
/// register (guess: the RM gives only 32-bit widths); one that crosses a register (from a bus master) is refused
/// with [`Stop::Unmodelled`]. No interrupt is raised: CR26\[INTMASK\] is stored like the rest of CR26, but the
/// model never asserts the controller's interrupt and it is wired to no interrupt source. Every write is recorded
/// ([`Ddrmc::writes`]).
#[derive(Debug)]
pub struct Ddrmc {
    cr: [u32; 64],
    rcr: u32,
    padcr: u32,
    /// INTSTATUS bits 7:0.
    int_status: u32,
    writes: Vec<(u32, Size, u32)>,
    record: bool,
    init_completes: bool,
}

impl Default for Ddrmc {
    fn default() -> Ddrmc {
        Ddrmc::new()
    }
}

impl Ddrmc {
    /// The controller as it leaves reset.
    pub fn new() -> Ddrmc {
        let mut cr = [0; 64];
        for (i, v) in CR_RESET {
            cr[i] = v;
        }
        Ddrmc {
            cr,
            rcr: 0,
            padcr: PADCR_RESET,
            int_status: 0,
            writes: Vec::new(),
            record: true,
            init_completes: true,
        }
    }

    /// Every write so far: (offset, size, value), in order.
    pub fn writes(&self) -> &[(u32, Size, u32)] {
        &self.writes
    }

    /// Every write so far, emptying the log.
    pub fn take_writes(&mut self) -> Vec<(u32, Size, u32)> {
        std::mem::take(&mut self.writes)
    }

    /// Whether writes are logged (default on). Off changes only the log, never the registers.
    pub fn record(&mut self, on: bool) {
        self.record = on;
    }

    /// Host knob: whether setting CR09\[START\] completes the initialization (default on). Off models a DRAM
    /// that never initializes: CR27 INIT_DONE (and the status OR bit) stay clear after START. A START made while
    /// the knob was off does not complete when it is turned back on; the next START edge does.
    pub fn set_init_completes(&mut self, on: bool) {
        self.init_completes = on;
    }

    fn register(&self, reg: u32) -> u32 {
        match reg {
            CR27 => {
                let any = if self.int_status != 0 { INT_ANY } else { 0 };
                self.cr[(CR27 / 4) as usize] | any | self.int_status
            }
            0..=0xFC => self.cr[(reg / 4) as usize],
            RCR => self.rcr,
            PADCR => self.padcr,
            _ => 0,
        }
    }
}

impl Device for Ddrmc {
    fn read(&mut self, offset: u32, size: Size, cx: &mut DeviceCx) -> u32 {
        if crosses_register(offset, size) {
            cx.refuse(
                Stop::Unmodelled,
                crossing("DDR controller", offset, size, "read"),
            );
            return 0;
        }
        lane_read(self.register(offset & !3), offset, size)
    }

    fn write(&mut self, offset: u32, size: Size, value: u32, cx: &mut DeviceCx) {
        if crosses_register(offset, size) {
            return cx.refuse(
                Stop::Unmodelled,
                crossing("DDR controller", offset, size, "write"),
            );
        }
        if self.record {
            self.writes.push((offset, size, value));
        }
        let reg = offset & !3;
        let new = lane_merge(self.register(reg), offset, size, value);
        match reg {
            _ if READ_ONLY.contains(&reg) => {}
            CR09 => {
                let old = self.cr[(CR09 / 4) as usize];
                if new & START != 0 && old & START == 0 && self.init_completes {
                    self.int_status |= INIT_DONE;
                }
                self.cr[(CR09 / 4) as usize] = new;
            }
            CR25 => {
                self.int_status &= !(new >> 16 & 0xFF);
                self.cr[(CR25 / 4) as usize] = new & !INTACK;
            }
            0..=0xFC => self.cr[(reg / 4) as usize] = new,
            RCR => self.rcr = new & !RCR_RST,
            PADCR => self.padcr = new | PADCR_ONES,
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
