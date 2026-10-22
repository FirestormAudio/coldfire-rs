//! The SCM's peripheral power management registers (RM 9.2.2-9.2.4): the clock gates of the peripheral slots.

use std::ops::Range;

use crate::device::{Device, DeviceCx};
use crate::mem::Size;

/// The SCM offsets the power management registers take: PPMSR0 at +0x2C to PPMLR1 at +0x3C (RM Table 9-1).
pub const POWER_MANAGEMENT: Range<u32> = 0x2C..0x40;

const PPMSR0: u32 = 0x2C;
const PPMCR0: u32 = 0x2D;
const PPMSR1: u32 = 0x2E;
const PPMCR1: u32 = 0x2F;
/// SAMCD and CAMCD: set or clear every bit of the pair (RM Table 9-3, Table 9-4).
const ALL: u8 = 0x40;
/// SMCD and CMCD: the CDn to set or clear.
const CD: u8 = 0x3F;

/// The clock gates as the machine holds them ([`Machine::clock_gates`](crate::Machine::clock_gates)), as the
/// registers read: PPMHR0, PPMLR0, PPMHR1, PPMLR1. Gate `64 s + n` is CDn of PPM{H,L}R`s`: CD0-CD31 are PPMLR`s`
/// and CD32-CD63 PPMHR`s`.
pub fn registers(closed: u128) -> [u32; 4] {
    [
        (closed >> 32) as u32,
        closed as u32,
        (closed >> 96) as u32,
        (closed >> 64) as u32,
    ]
}

/// The clock gates the registers PPMHR0, PPMLR0, PPMHR1, PPMLR1 hold (the inverse of [`registers`]).
pub fn gates([hr0, lr0, hr1, lr1]: [u32; 4]) -> u128 {
    u128::from(hr1) << 96 | u128::from(lr1) << 64 | u128::from(hr0) << 32 | u128::from(lr0)
}

/// The peripheral power management registers (RM 9.2.2-9.2.4) at their SCM offsets ([`POWER_MANAGEMENT`]),
/// over the machine's clock gates ([`DeviceCx::clock_gates`]): they hold no state of their own.
///
/// - PPMSR0, PPMCR0, PPMSR1, PPMCR1 (+0x2C-+0x2F, bytes, write-only, read 0): a write sets (PPMSRn) or clears
///   (PPMCRn) CDn bit `value[5:0]` of PPM{H,L}Rn, or with bit 6 (SAMCD, CAMCD) all 64 of them.
/// - PPMHR0, PPMLR0, PPMHR1, PPMLR1 (+0x30-+0x3C, read/write): the gates, bit set while the slot's clock is off.
///
/// Accesses address bytes: a word or longword access covers the registers of each of its bytes, in address order
/// (a longword write at +0x2C is a PPMSR0, PPMCR0, PPMSR1 and PPMCR1 write).
// guess: a word or longword write over PPMSRn and PPMCRn acts as its byte writes in address order (not measured).
// guess: every bit of PPM{H,L}Rn is stored, the reserved ones too; RM Figures 9-4 to 9-7 show some reserved bits as
// fixed, which is not modelled (not measured).
#[derive(Debug, Default)]
pub struct PowerManagement;

impl PowerManagement {
    /// The registers.
    pub fn new() -> PowerManagement {
        PowerManagement
    }
}

/// The byte at `offset` of the registers for gates `closed`.
fn byte(closed: u128, offset: u32) -> u8 {
    match offset {
        0x30..0x40 => {
            let regs = registers(closed);
            let k = (offset - 0x30) as usize;
            regs[k / 4].to_be_bytes()[k % 4]
        }
        _ => 0,
    }
}

/// `closed` after a write of `value` to the byte at `offset`.
fn write_byte(closed: u128, offset: u32, value: u8) -> u128 {
    let pair = |n: u32| -> u128 { u128::from(u64::MAX) << (64 * n) };
    let bit = |n: u32| -> u128 { 1u128 << (64 * n + u32::from(value & CD)) };
    let one = |n: u32| if value & ALL != 0 { pair(n) } else { bit(n) };
    match offset {
        PPMSR0 => closed | one(0),
        PPMCR0 => closed & !one(0),
        PPMSR1 => closed | one(1),
        PPMCR1 => closed & !one(1),
        0x30..0x40 => {
            let mut regs = registers(closed);
            let k = (offset - 0x30) as usize;
            let mut b = regs[k / 4].to_be_bytes();
            b[k % 4] = value;
            regs[k / 4] = u32::from_be_bytes(b);
            gates(regs)
        }
        _ => closed,
    }
}

impl Device for PowerManagement {
    fn read(&mut self, offset: u32, size: Size, cx: &mut DeviceCx) -> u32 {
        let closed = cx.clock_gates();
        (0..size.bytes()).fold(0, |v, k| v << 8 | u32::from(byte(closed, offset + k)))
    }

    fn write(&mut self, offset: u32, size: Size, value: u32, cx: &mut DeviceCx) {
        let n = size.bytes();
        let closed = (0..n).fold(cx.clock_gates(), |c, k| {
            write_byte(c, offset + k, (value >> (8 * (n - 1 - k))) as u8)
        });
        cx.set_clock_gates(closed);
    }

    fn poll_counted(&self) -> bool {
        false
    }

    /// No events, no interrupts.
    fn wake_sources(&self, _out: &mut Vec<(usize, u8)>) -> bool {
        true
    }
}
