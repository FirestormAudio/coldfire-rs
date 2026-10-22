//! The DMA timers DTIM0-DTIM3 (RM ch. 39).

use super::{crosses_register, crossing, lane_merge, lane_read};
use crate::device::{Device, DeviceCx};
use crate::mem::Size;
use crate::stop::Stop;

/// DTMR (RM Table 39-2).
const RST: u16 = 0x0001;
const CLK: u16 = 0x0006;
const FRR: u16 = 0x0008;
const ORRI: u16 = 0x0010;
const OM: u16 = 0x0020;
const CE: u16 = 0x00C0;
/// DTXMR (RM Table 39-3).
const MODE16: u8 = 0x01;
const EPD: u8 = 0x10;
const EN1588: u8 = 0x20;
const HALTED: u8 = 0x40;
const DMAEN: u8 = 0x80;
/// DTER (RM Table 39-4).
const CAP: u8 = 0x01;
const REF: u8 = 0x02;

/// The register offsets (RM Table 39-1): DTMR, DTXMR and DTER share the first longword.
const DTMR: u32 = 0x0;
const DTRR: u32 = 0x4;
const DTCR: u32 = 0x8;
const DTCN: u32 = 0xC;

/// The inverse of 65537 modulo 2^32: (1 + 2^16)(1 - 2^16) = 1 - 2^32.
const INV_65537: u32 = 0xFFFF_0001;

/// One DMA timer: DTMR at +0, DTXMR at +2, DTER at +3, DTRR at +4, DTCR at +8, DTCN at +0xC (RM Table 39-1).
///
/// One tick of the machine is one internal bus clock, the clock the PIT counts too (RM 39.3.1: fsys/2); DMA timer 0
/// at CLK = bus clock / 1 counts the bus clock (measured on an MCF54415). What a tick is in time is the board's
/// choice. The input clock (DTMR CLK) is the bus clock, the bus clock divided by 16,
/// or the DT*n*IN pin, whose falling edges come every [`Dtim::new`]'s `dtin` ticks; it is divided by PS + 1 and
/// each prescaler output increments DTCN by 1, or by 65 537 with DTXMR MODE16 (RM Table 39-3).
///
/// - DTMR (reset 0): RST, CLK, FRR, ORRI and PS. The counter runs while RST is set and CLK is not 0 (RM Table
///   39-2), and holds its value otherwise. guess: a change of CLK, PS or MODE16, a write to DTCN, or a start drops
///   the prescaler's partial count; the divide-by-16 runs freely from tick 0 (RM Table 39-2: it is not synchronized
///   with the timer), and so does the DT*n*IN edge grid; the two to three bus clocks of DT*n*IN synchronization (RM
///   39.3.1) are not modelled.
/// - RST from 1 to 0 "resets register values" (RM Table 39-2). guess: it clears DTCN and DTER; DTRR, DTCR and
///   DTXMR keep their values.
/// - RM Table 39-2: "Avoid setting CLK when RST is already set. Doing so causes CLK to zero". guess: a write that
///   changes CLK to a non-zero value while RST stays set leaves CLK 0 (the count stops); a write that keeps CLK
///   keeps counting.
/// - DTXMR (reset 0): MODE16 and HALTED are stored (HALTED has no effect: the core is never halted while it
///   runs). DMAEN (no DMA request line is modelled), EN1588 (no 1588 timebase) and EPD are refused with
///   [`Stop::Unmodelled`], as are DTMR CE (capture mode) and OM set (toggle output, RM 39.3.4).
/// - DTER (reset 0): REF is set when the prescaler output would increment DTCN past DTRR, that is after DTRR + 1
///   increments from 0 (RM 39.2.4, Eqn. 39-1); with FRR set the count restarts at 0 there, otherwise it goes on
///   (RM 39.3.3). Writing 1 clears REF and CAP (RM 39.2.3). CAP is never set.
/// - DTRR (reset 0xFFFF_FFFF): the reference. DTCR (reset 0): read-only (RM Figure 39-6; guess: a write is
///   ignored, although RM Table 39-1 lists it read/write). DTCN (reset 0): any write clears it (RM Table 39-7;
///   guess: at any size, all 32 bits).
///
/// Byte, word and longword accesses address their lanes of the registers (a long access at +0 is DTMR, DTXMR and
/// DTER together). The rest of the window reads 0 and ignores writes.
///
/// The timer drives source `source` of interrupt controller `controller` with REF and ORRI (RM Table 39-4).
///
/// While its clock gate is closed ([`Device::gate`]) the counter stops and no reference event happens; opening the
/// gate continues the count where it stopped. guess: not measured (a module without its clock is taken not to
/// count); the prescaler's partial count is dropped when the gate opens.
#[derive(Debug)]
pub struct Dtim {
    n: u8,
    controller: usize,
    source: u8,
    /// Ticks between the falling edges of DT*n*IN, if it toggles.
    dtin: Option<u64>,
    dtmr: u16,
    dtxmr: u8,
    dter: u8,
    dtrr: u32,
    dtcr: u32,
    /// The counter at `origin`, or while it is held.
    base: u32,
    /// The input clock edges up to the tick the count last started from (`tick / divisor`).
    origin: u64,
    /// The tick of the next reference event while counting.
    next: Option<u64>,
    gated: bool,
    references: u64,
}

impl Dtim {
    /// DMA timer `n`, driving `source` of interrupt controller `controller` (its
    /// [`Intc::index`](super::Intc::index)), with a falling edge on DT*n*IN every `dtin` ticks, or none (`None`: the
    /// pin never toggles). Panics when `dtin` is `Some(0)`.
    pub fn new(n: u8, controller: usize, source: u8, dtin: Option<u64>) -> Dtim {
        assert!(dtin != Some(0), "DMA timer {n}: DTIN period of 0 ticks");
        Dtim {
            n,
            controller,
            source,
            dtin,
            dtmr: 0,
            dtxmr: 0,
            dter: 0,
            dtrr: u32::MAX,
            dtcr: 0,
            base: 0,
            origin: 0,
            next: None,
            gated: false,
            references: 0,
        }
    }

    /// How many reference events (DTER REF set) there have been.
    pub fn references(&self) -> u64 {
        self.references
    }

    /// Ticks per input clock edge for the clock DTMR selects; `None` when it does not count.
    fn divisor(&self) -> Option<u64> {
        match (self.dtmr & CLK) >> 1 {
            1 => Some(1),
            2 => Some(16),
            3 => self.dtin,
            _ => None,
        }
    }

    /// The divisor while the counter runs.
    fn running(&self) -> Option<u64> {
        if self.gated || self.dtmr & RST == 0 {
            return None;
        }
        self.divisor()
    }

    fn prescale(&self) -> u64 {
        u64::from(self.dtmr >> 8) + 1
    }

    fn step(&self) -> u32 {
        if self.dtxmr & MODE16 != 0 { 65537 } else { 1 }
    }

    /// The increments since the count started, at `now`.
    fn increments(&self, d: u64, now: u64) -> u64 {
        (now / d).saturating_sub(self.origin) / self.prescale()
    }

    fn dtcn(&self, now: u64) -> u32 {
        match self.running() {
            None => self.base,
            Some(d) => {
                let k = self.increments(d, now) as u32;
                self.base.wrapping_add(k.wrapping_mul(self.step()))
            }
        }
    }

    /// Hold the count at `now` (before a change to how it counts).
    fn hold(&mut self, now: u64) {
        self.base = self.dtcn(now);
    }

    /// Start counting from `base` at `now` (after a change to how it counts), and schedule the next event.
    fn start(&mut self, now: u64) {
        if let Some(d) = self.running() {
            self.origin = now / d;
        }
        self.schedule(now);
    }

    /// The tick of the next reference event from `now`: the increment after the first one at or after `now` that
    /// leaves DTCN equal to DTRR.
    fn schedule(&mut self, now: u64) {
        self.next = self.running().and_then(|d| {
            let k_now = self.increments(d, now);
            let inverse = if self.dtxmr & MODE16 != 0 {
                INV_65537
            } else {
                1
            };
            let k0 = u64::from(self.dtrr.wrapping_sub(self.base).wrapping_mul(inverse));
            let k = if k0 >= k_now {
                k0
            } else {
                k0 + ((k_now - k0).div_ceil(1 << 32) << 32)
            };
            let edges = (k + 1)
                .checked_mul(self.prescale())?
                .checked_add(self.origin)?;
            edges.checked_mul(d)
        });
    }

    fn drive(&self, cx: &mut DeviceCx) {
        let on = self.dter & REF != 0 && self.dtmr & ORRI != 0 && self.dtxmr & DMAEN == 0;
        cx.irq.set(self.controller, self.source, on);
    }

    fn refuse(&self, cx: &mut DeviceCx, what: &str) {
        cx.refuse(
            Stop::Unmodelled,
            format!("DMA timer {}: {what} not modelled", self.n),
        );
    }

    fn write_dtmr(&mut self, mut value: u16, now: u64) {
        let was = self.dtmr;
        // guess: changing CLK to a running clock while RST stays set stops the count (RM Table 39-2).
        if was & RST != 0 && value & RST != 0 && value & CLK != was & CLK && value & CLK != 0 {
            value &= !CLK;
        }
        let how = |r: u16| (r & (RST | CLK), r >> 8);
        if how(value) == how(was) {
            self.dtmr = value;
            self.schedule(now);
            return;
        }
        self.hold(now);
        if was & RST != 0 && value & RST == 0 {
            // guess: the software reset clears the counter and the events.
            self.base = 0;
            self.dter = 0;
        }
        self.dtmr = value;
        self.start(now);
    }

    fn write_dtxmr(&mut self, value: u8, now: u64) {
        let value = value & (MODE16 | HALTED);
        if (value ^ self.dtxmr) & MODE16 != 0 {
            self.hold(now);
            self.dtxmr = value;
            self.start(now);
        } else {
            self.dtxmr = value;
        }
    }

    fn register(&self, reg: u32, now: u64) -> u32 {
        match reg {
            DTMR => u32::from(self.dtmr) << 16 | u32::from(self.dtxmr) << 8 | u32::from(self.dter),
            DTRR => self.dtrr,
            DTCR => self.dtcr,
            DTCN => self.dtcn(now),
            _ => 0,
        }
    }
}

impl Device for Dtim {
    fn read(&mut self, offset: u32, size: Size, cx: &mut DeviceCx) -> u32 {
        if crosses_register(offset, size) {
            let what = format!("DMA timer {}", self.n);
            cx.refuse(Stop::Unmodelled, crossing(&what, offset, size, "read"));
            return 0;
        }
        lane_read(self.register(offset & !3, cx.now), offset, size)
    }

    fn write(&mut self, offset: u32, size: Size, value: u32, cx: &mut DeviceCx) {
        if crosses_register(offset, size) {
            let what = format!("DMA timer {}", self.n);
            return cx.refuse(Stop::Unmodelled, crossing(&what, offset, size, "write"));
        }
        let now = cx.now;
        match offset & !3 {
            DTMR => {
                let covered = lane_merge(0, offset, size, u32::MAX);
                let new = lane_merge(self.register(DTMR, now), offset, size, value);
                let (dtmr, dtxmr) = ((new >> 16) as u16, (new >> 8) as u8);
                if covered & 0xFFFF_0000 != 0 {
                    if dtmr & CE != 0 {
                        return self.refuse(cx, "capture mode (DTMR CE)");
                    }
                    if dtmr & OM != 0 {
                        return self.refuse(cx, "toggle output (DTMR OM)");
                    }
                }
                if covered & 0xFF00 != 0 {
                    if dtxmr & DMAEN != 0 {
                        return self.refuse(cx, "DMA request (DTXMR DMAEN)");
                    }
                    if dtxmr & EN1588 != 0 {
                        return self.refuse(cx, "IEEE 1588 timebase (DTXMR EN1588)");
                    }
                    if dtxmr & EPD != 0 {
                        return self.refuse(cx, "programmable delay (DTXMR EPD)");
                    }
                }
                if covered & 0xFF != 0 {
                    self.dter &= !(new as u8 & (REF | CAP));
                }
                if covered & 0xFF00 != 0 {
                    self.write_dtxmr(dtxmr, now);
                }
                if covered & 0xFFFF_0000 != 0 {
                    self.write_dtmr(dtmr, now);
                }
            }
            DTRR => {
                self.dtrr = lane_merge(self.dtrr, offset, size, value);
                self.schedule(now);
            }
            DTCN => {
                self.hold(now);
                self.base = 0;
                self.start(now);
            }
            _ => {} // DTCR is read-only; the rest of the window ignores writes
        }
        self.drive(cx);
    }

    fn next_event(&self) -> Option<u64> {
        self.next
    }

    fn advance(&mut self, now: u64, cx: &mut DeviceCx) {
        while let Some(at) = self.next {
            if at > now {
                break;
            }
            self.references += 1;
            self.dter |= REF;
            if self.dtmr & FRR != 0
                && let Some(d) = self.running()
            {
                // RM 39.3.3: a new count starts.
                self.base = 0;
                self.origin = at / d;
            }
            self.schedule(at);
        }
        self.drive(cx);
    }

    fn poll_counted(&self) -> bool {
        false
    }

    // guess: a gated timer stops counting and resumes where it stopped (not measured).
    fn gate(&mut self, closed: bool, now: u64) {
        if closed {
            self.hold(now);
            self.gated = true;
            self.next = None;
        } else {
            self.gated = false;
            self.start(now);
        }
    }

    fn wake_sources(&self, out: &mut Vec<(usize, u8)>) -> bool {
        if self.next.is_some() && self.dtmr & ORRI != 0 {
            out.push((self.controller, self.source));
        }
        true
    }
}
