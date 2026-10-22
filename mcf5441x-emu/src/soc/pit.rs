//! The programmable interrupt timers PIT0-PIT3 (RM ch. 38).

use crate::device::{Device, DeviceCx};
use crate::mem::Size;

const EN: u16 = 0x0001;
const RLD: u16 = 0x0002;
const PIF: u16 = 0x0004;
const PIE: u16 = 0x0008;
const OVW: u16 = 0x0010;
/// The writable PCSR bits: PRE, DOZE, DBG, OVW, PIE, RLD, EN (PIF is write-1-to-clear).
const WRITABLE: u16 = 0x0F7B;

/// One programmable interrupt timer: PCSR at +0, PMR at +2, PCNTR at +4 (RM 38.2).
///
/// One tick of the machine is one count of the prescaler input; the period of a timer is
/// `(modulus + 1) << PRE` ticks, and what a tick is in time is the board's choice.
///
/// - PCSR (reset 0): EN, RLD, PIF (write 1 to clear), PIE, OVW, DBG and DOZE (stored only), PRE in bits 11:8.
///   Enabling loads the counter from PMR (the RM does not say so). Changing PRE while enabled keeps the count and
///   continues it at the new rate. guess: the prescaler's partial count is dropped (RM 38.2.1 says changing PRE
///   resets the prescaler counter).
/// - PMR (reset 0xFFFF): the modulus. Writing it clears PIF (RM 38.3.4). The counter takes it at its next reload,
///   or at once with OVW. A write with the timer disabled and OVW set loads the held counter (guess: RM 38.2.2
///   says PMR is then transparent to the counter).
/// - PCNTR (read-only, reset 0xFFFF): the down-counter. A disabled timer holds its value (RM 38.2.1).
/// - With RLD set the counter reloads PMR when it passes 0; with RLD clear it rolls over to 0xFFFF and counts
///   a full 65536 counts (RM 38.3.2). PIF is set at each expiry, which is the reload, one count after PCNTR
///   reads 0 (guess: RM 38.3 says "when the counter reaches 0x0000"; this is the older model's grid). Expiries
///   stay on the original grid when the timer is serviced late.
///
/// Byte and word accesses address their lanes of the registers: a byte at +0 is the high byte of PCSR (PRE and the
/// reserved bits), at +1 its low byte, at +2 and +3 the bytes of PMR, and so on for PCNTR; a byte write to PCSR
/// or PMR takes effect as the word write of that lane would. Longword accesses are not allowed (RM ch. 38): a
/// longword read of any register returns 0xFFFFFFFF and a longword write changes nothing, with no exception
/// (measured on an MCF54415).
///
/// The timer drives source `source` of interrupt controller `controller` with PIF and PIE.
///
/// While its clock gate is closed ([`Device::gate`]) the counter stops and no expiry happens; opening the gate
/// continues the count where it stopped. guess: not measured (a module without its clock is taken not to count).
#[derive(Debug)]
pub struct Pit {
    controller: usize,
    source: u8,
    pcsr: u16,
    pmr: u16,
    /// The modulus the counter is running with.
    modulus: u32,
    /// The tick of the last load.
    start: u64,
    /// The tick of the next expiry while enabled.
    next: Option<u64>,
    /// The counter while disabled.
    held: u16,
    expiries: u64,
    /// The tick its clock gate closed, while it is closed.
    gated_at: Option<u64>,
}

impl Pit {
    /// A timer driving `source` of interrupt controller `controller` (its [`Intc::index`](super::Intc::index)).
    pub fn new(controller: usize, source: u8) -> Pit {
        Pit {
            controller,
            source,
            pcsr: 0,
            pmr: 0xFFFF,
            modulus: 0xFFFF,
            start: 0,
            next: None,
            held: 0xFFFF,
            expiries: 0,
            gated_at: None,
        }
    }

    /// How many times the counter has expired.
    pub fn expiries(&self) -> u64 {
        self.expiries
    }

    fn pre(&self) -> u32 {
        u32::from(self.pcsr >> 8) & 0xF
    }

    fn period(&self) -> u64 {
        (u64::from(self.modulus) + 1) << self.pre()
    }

    fn restart(&mut self, now: u64) {
        self.start = now;
        self.modulus = u32::from(self.pmr);
        self.next = Some(now + self.period());
    }

    fn pcntr(&self, now: u64) -> u16 {
        match self.next {
            None => self.held,
            Some(_) => {
                let counts = (now - self.start) >> self.pre();
                (u64::from(self.modulus) - counts % (u64::from(self.modulus) + 1)) as u16
            }
        }
    }

    fn drive(&self, cx: &mut DeviceCx) {
        let on = self.pcsr & PIF != 0 && self.pcsr & PIE != 0;
        cx.irq.set(self.controller, self.source, on);
    }

    /// Write the PCSR bits in `mask` (already in place in `value`).
    fn write_pcsr(&mut self, mask: u16, value: u16, cx: &mut DeviceCx) {
        let value = value & mask;
        let mut pif = self.pcsr & PIF;
        if value & PIF != 0 {
            pif = 0;
        }
        let new = (self.pcsr & !mask & WRITABLE) | (value & WRITABLE) | pif;
        let (was, now_en) = (self.pcsr & EN != 0, new & EN != 0);
        let new_pre = u32::from(new >> 8) & 0xF;
        if was && now_en && new_pre != self.pre() {
            // Continue from the current count at the new rate.
            let c = self.pcntr(cx.now);
            self.start = cx.now;
            self.modulus = u32::from(c);
            self.next = Some(cx.now + ((u64::from(c) + 1) << new_pre));
        }
        if was && !now_en {
            self.held = self.pcntr(cx.now);
            self.next = None;
        }
        self.pcsr = new;
        if now_en && !was {
            self.restart(cx.now);
        }
        self.drive(cx);
    }

    fn write_pmr(&mut self, value: u16, cx: &mut DeviceCx) {
        self.pmr = value;
        self.pcsr &= !PIF;
        if self.pcsr & OVW != 0 {
            if self.pcsr & EN != 0 {
                self.restart(cx.now);
            } else {
                self.held = value;
            }
        }
        self.drive(cx);
    }
}

fn lane_mask(size: Size) -> u32 {
    match size {
        Size::Byte => 0xFF,
        Size::Word => 0xFFFF,
        Size::Long => u32::MAX,
    }
}

impl Device for Pit {
    fn read(&mut self, offset: u32, size: Size, cx: &mut DeviceCx) -> u32 {
        if size == Size::Long {
            return u32::MAX;
        }
        let (base, lane, n) = (offset & !3, offset & 3, size.bytes());
        if lane + n > 4 {
            return 0;
        }
        let image = match base {
            0 => u32::from(self.pcsr) << 16 | u32::from(self.pmr),
            4 => u32::from(self.pcntr(cx.now)) << 16,
            _ => 0,
        };
        (image >> ((4 - lane - n) * 8)) & lane_mask(size)
    }

    fn write(&mut self, offset: u32, size: Size, value: u32, cx: &mut DeviceCx) {
        let (lane, n) = (offset & 3, size.bytes());
        if size == Size::Long || offset >= 4 || lane + n > 4 {
            return; // PCNTR and the rest of the window ignore writes
        }
        let shift = (4 - lane - n) * 8;
        let mask = lane_mask(size) << shift;
        let value = (value << shift) & mask;
        let (mask_lo, value_lo) = ((mask & 0xFFFF) as u16, (value & 0xFFFF) as u16);
        if mask_lo != 0 {
            let merged = (self.pmr & !mask_lo) | value_lo;
            self.write_pmr(merged, cx);
        }
        let (mask_hi, value_hi) = ((mask >> 16) as u16, (value >> 16) as u16);
        if mask_hi != 0 {
            self.write_pcsr(mask_hi, value_hi, cx);
        }
    }

    fn next_event(&self) -> Option<u64> {
        self.next
    }

    fn advance(&mut self, now: u64, cx: &mut DeviceCx) {
        while let Some(at) = self.next {
            if at > now {
                break;
            }
            self.expiries += 1;
            self.pcsr |= PIF;
            self.start = at;
            // RM 38.3.1 and RM 38.3.2: reload PMR, or roll over to 0xFFFF.
            self.modulus = if self.pcsr & RLD != 0 {
                u32::from(self.pmr)
            } else {
                0xFFFF
            };
            self.next = Some(at + self.period());
        }
        self.drive(cx);
    }

    fn poll_counted(&self) -> bool {
        false
    }

    // guess: a gated timer stops counting and resumes where it stopped (not measured).
    fn gate(&mut self, closed: bool, now: u64) {
        if closed {
            self.gated_at = Some(now);
        } else if let Some(at) = self.gated_at.take() {
            let d = now - at;
            self.start += d;
            self.next = self.next.map(|t| t + d);
        }
    }

    fn wake_sources(&self, out: &mut Vec<(usize, u8)>) -> bool {
        if self.next.is_some() && self.pcsr & PIE != 0 {
            out.push((self.controller, self.source));
        }
        true
    }
}
