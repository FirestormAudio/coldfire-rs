//! The enhanced secure digital host controller, eSDHC (RM ch. 25), with an [`Emmc`] behind it.

use std::collections::VecDeque;

use crate::device::{Device, DeviceCx};
use crate::mem::Size;
use crate::parts::emmc::{Emmc, Response, Transfer};
use crate::stop::Stop;

const DSADDR: u32 = 0x00;
const BLKATTR: u32 = 0x04;
const CMDARG: u32 = 0x08;
const XFERTYP: u32 = 0x0C;
const CMDRSP0: u32 = 0x10;
const CMDRSP3: u32 = 0x1C;
const DATPORT: u32 = 0x20;
const PRSSTAT: u32 = 0x24;
const PROCTL: u32 = 0x28;
const SYSCTL: u32 = 0x2C;
const IRQSTAT: u32 = 0x30;
const IRQSTATEN: u32 = 0x34;
const IRQSIGEN: u32 = 0x38;
const HOSTCAPBLT: u32 = 0x40;
const WML: u32 = 0x44;
const FEVT: u32 = 0x50;
const ADMASAR: u32 = 0x58;
const VENDOR: u32 = 0xC0;
const HOSTVER: u32 = 0xFC;

/// Reset values (RM Table 25-2, measured on an MCF54415).
const BLKATTR_RESET: u32 = 0x0001_0000;
const PROCTL_RESET: u32 = 0x20;
const SYSCTL_RESET: u32 = 0x8008;
const IRQSTATEN_RESET: u32 = 0x117F_013F;
const WML_RESET: u32 = 0x0810_0810;
const VENDOR_RESET: u32 = 1;
const HOSTCAPBLT_VALUE: u32 = 0x07F3_0000;
const HOSTVER_VALUE: u32 = 0x1201;

/// Ticks from CMD14's data phase to its word and status when no one reads IRQSTAT first (guess: not measured).
const WORD_DELAY: u64 = 512;

/// Ticks from the first IRQSTAT read that shows CMD14's CC to its word and status (guess: not measured; room for a
/// driver to check what it read, with a second read, before they come).
const SEEN_DELAY: u64 = 64;

/// XFERTYP (RM Table 25-6).
const DPSEL: u32 = 1 << 21;
const MSBSEL: u32 = 1 << 5;
const BCEN: u32 = 1 << 1;
const DMAEN: u32 = 1 << 0;

/// PRSSTAT (RM Table 25-11): the bits the model drives, and the rest as they read at reset: DLSL and CLSL high, the
/// clocks gated off while idle, SDSTB.
const CDIHB: u32 = 1 << 1;
const DLA: u32 = 1 << 2;
const BWEN: u32 = 1 << 10;
const BREN: u32 = 1 << 11;
const DAT0: u32 = 1 << 24;
const PRSSTAT_LEVELS: u32 = 0xFF88_00F8;

/// PROCTL (RM Table 25-12).
const DTW_4BIT: u32 = 1 << 1;
const PROCTL_BITS: u32 = 0x070F_03FE;

/// SYSCTL (RM Table 25-13).
const RSTA: u32 = 1 << 24;
const RSTC: u32 = 1 << 25;
const RSTD: u32 = 1 << 26;
const INITA: u32 = 1 << 27;
const SDCLKEN: u32 = 1 << 3;
/// SDCLKFS and DVS.
const DIVIDER: u32 = 0xFFF0;
const SYSCTL_BITS: u32 = 0x000F_FFFF;

/// IRQSTAT (RM Table 25-14).
const CC: u32 = 1 << 0;
const TC: u32 = 1 << 1;
const CTOE: u32 = 1 << 16;
const DTOE: u32 = 1 << 20;
const DCE: u32 = 1 << 21;
/// The status bits IRQSTATEN and IRQSIGEN define.
const IRQ_BITS: u32 = 0x117F_01FF;
/// The status bits RSTD clears: BRR, BWR, DINT, BGE and TC (RM Table 25-13).
const RSTD_CLEARS: u32 = 0x3E;

/// WML: bits 27 and 11 read 1, the levels are bits 23:16 and 7:0 (RM Figure 25-17).
const WML_ONES: u32 = 0x0800_0800;
const WML_LEVELS: u32 = 0x00FF_00FF;

/// The board's wiring and timing of the eSDHC.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EsdhcConfig {
    /// The interrupt controller and source of the eSDHC interrupt (INTC2 source 31 on the chip, RM Table 17-17).
    pub controller: usize,
    pub source: u8,
    /// The DMA request line of the buffer (eDMA channel 59 on the chip, RM Table 19-6).
    pub dma: u8,
    /// Ticks from a data command to its data phase.
    pub data_latency: u64,
    /// Ticks the card holds DAT0 low (busy) after a CMD6.
    pub cmd6_busy: u64,
    /// Ticks the card holds DAT0 low (busy) after a CMD25's last block, before TC (guess: not measured).
    pub write_busy: u64,
    /// Ticks the card holds DAT0 low (busy) after a CMD38, before TC (guess: not measured).
    pub erase_busy: u64,
}

/// A data phase to come.
#[derive(Debug, Clone, Copy)]
struct Pending {
    at: u64,
    cmd: u8,
    arg: u32,
    blocks: u32,
    transfer: Transfer,
}

/// The eSDHC (RM ch. 25), mapped at 0xFC0C_C000, with an optional [`Emmc`] behind it; driven by an external DMA
/// (eDMA channel 59) through DATPORT. The internal DMA (XFERTYP\[DMAEN\]) is not modelled.
///
/// Registers (32-bit accesses only, RM Table 25-2; anything else is refused with [`Stop::Unmodelled`]), with their
/// reset values (RM Table 25-2, all measured on an MCF54415):
/// - DSADDR (0), BLKATTR (0x0001_0000), CMDARG (0), PROCTL (0x20), ADMASAR (0), VENDOR (1): stored. BLKATTR's block
///   size and count are used by block reads and writes; PROCTL's EMODE and DTW are checked; VENDOR\[EXT_DMA_EN\]
///   gates the DMA request. BLKATTR reads BLKCNT 1 whatever count was written (measured on an MCF54415 with no
///   transfer running, for counts of 2, 3, 0x20 and 0x40; the block size reads back); guess: the written count is
///   kept and is the one a transfer uses, and a transfer under way reads 1 as well.
/// - XFERTYP (0): a write sends the command (below).
/// - CMDRSP0-CMDRSP3: the response; a 48-bit one goes to CMDRSP0 alone, a 136-bit one to all four (RM Table 25-9).
/// - DATPORT: reads pop the buffer (an empty buffer reads 0, measured on an MCF54415); a write is the
///   bus-test word while PRSSTAT\[BWEN\] is set, and is lost otherwise.
/// - PRSSTAT: CDIHB and DLA while a data phase is pending or running (and after a data error, until RSTD or RSTA),
///   BREN and BWEN for the bus test; the rest reads as at reset (0xFF88_00F8: the lines high, SDSTB, the clocks
///   gated off — guess: the model's controller is always idle between instantaneous steps). DAT0 (bit 24) reads
///   low while the card is busy after CMD6. CIHB is never set: a response is immediate.
/// - SYSCTL (0x8008, SDCLKEN set): stored; RSTA, RSTC, RSTD and INITA act and read 0. A write with RSTA leaves
///   SYSCTL at 0x8008 and drops the rest of the written value (measured on an MCF54415). Changing SDCLKFS or DVS
///   while SDCLKEN is set in both the old and the new value is refused (RM Table 25-13: change the frequency with
///   SDCLKEN clear), but only when a SYSCTL write since the last reset (power-on or RSTA) set SDCLKEN: the reset
///   value's SDCLKEN does not count, as a driver's first write after reset sets its divider with SDCLKEN and works
///   on the chip.
/// - IRQSTAT: write 1 to clear. A status bit is set only when its IRQSTATEN bit is (RM 25.3.11).
/// - IRQSTATEN (0x117F_013F): reserved bits read 0; clearing a bit clears its status bit (RM 25.3.11).
/// - IRQSIGEN (0): a write setting a reserved bit is refused. The interrupt request is asserted while IRQSTAT and
///   IRQSIGEN share a bit.
/// - WML (0x0810_0810): the two levels are stored and read back with bits 27 and 11 set (RM Figure 25-17;
///   measured on an MCF54415: 0x0800_0800 with the levels written); not used.
/// - FEVT: a write sets the IRQSTAT bits it names that IRQSTATEN enables (RM 25.3.16); reads 0.
/// - HOSTCAPBLT (0x07F3_0000), HOSTVER (0x1201): constant. AUTOC12ERR and ADMAESR read 0; the rest of the window reads
///   0 and ignores writes.
///
/// Resets (SYSCTL): RSTD cancels the data phase, empties the buffer, clears PRSSTAT's CDIHB, DLA, BREN and BWEN and
/// IRQSTAT's BRR, BWR, DINT, BGE and TC (RM Table 25-13), and puts PROCTL back to 0x20 and WML to 0x0810_0810
/// (measured on an MCF54415, not in the RM's list). RSTC clears IRQSTAT\[CC\]. RSTA does both and puts every register
/// back to its reset value (measured on an MCF54415 for SYSCTL, DSADDR, BLKATTR, CMDARG, PROCTL, IRQSTATEN, IRQSIGEN,
/// WML, ADMASAR and VENDOR; guess for the rest: RM Table 25-13 has it reset the whole controller). The card is not reset: a card left at 4 bits by a reset host fails every read (below).
///
/// A command (a write of XFERTYP) is checked in this order; each failing check refuses the write with
/// [`Stop::Unmodelled`], the detail naming the eSDHC or the eMMC and the feature:
/// 1. the card does not model the command (its read-only set, [`Emmc`]);
/// 2. PROCTL\[EMODE\] other than address-invariant (RM Table 25-12: the only legal value);
/// 3. XFERTYP\[DMAEN\]: the internal DMA;
/// 4. the card busy after CMD6 (DAT0 low);
/// 5. a data command (DPSEL) while PRSSTAT\[CDIHB\] is set (RM 25.3.4 ignores it: the command would be lost);
/// 6. a data command the card has no data phase for, or a block read with a block size other than 512, an infinite
///    transfer (MSBSEL without BCEN) or a block count of 0.
///
/// Then the card answers at once: its response and IRQSTAT\[CC\], or IRQSTAT\[CTOE\] when it answers nothing (no
/// card, or a card stuck sending). XFERTYP\[AC12EN\] is accepted, but no automatic CMD12 reaches the card (harmless
/// while the card has no state machine to leave). The card holds DAT0 low for `cmd6_busy` ticks after CMD6: the busy
/// is keyed on the command index, not on RSPTYP R1b, so CMD7 and CMD12 never hold DAT0. A data command sets CDIHB and DLA, and its data phase comes `data_latency` ticks
/// later:
/// - A host bus width (PROCTL\[DTW\]) other than the card's fails it with DCE (measured on an MCF54415).
/// - Block reads (CMD8, CMD18): the data is loaded into the buffer as big-endian words, and the DMA request is
///   asserted while the buffer holds words (and VENDOR\[EXT_DMA_EN\] is set); the eDMA drains it through DATPORT in
///   the same tick. The read that empties it ends the transfer with TC. A buffer still holding words one tick after
///   the data phase began ends it with DTOE and is emptied (guess: the RM's data timeout, SYSCTL\[DTOCV\], is not
///   modelled). A read whose CRC fails (an injected fault) ends with DCE.
/// - CMD14: after CC, the card's word is put in the buffer with BREN and DCE is latched, `SEEN_DELAY` ticks after the first
///   IRQSTAT read that shows CC or `WORD_DELAY` ticks into the data phase (guess: not measured; a driver waits for a
///   status before it reads the word); the read that takes it ends the transfer with TC.
/// - CMD19: BWEN; the DATPORT write that follows goes to the card and ends the transfer with DCE (measured on an
///   MCF54415: even when the bus test passes) and TC (guess: the bus-test sequence waits for TC before CMD14).
///
/// TC frees the data line (CDIHB, DLA, BREN, BWEN); an error (DCE, DTOE) leaves it busy until RSTD or RSTA.
/// BRR and BWR are never set: the watermarks are not modelled. Every accepted register write is recorded
/// ([`Esdhc::writes`]) unless [`Esdhc::record`] turned the log off.
#[derive(Debug)]
pub struct Esdhc {
    config: EsdhcConfig,
    card: Option<Emmc>,
    dsaddr: u32,
    blkattr: u32,
    cmdarg: u32,
    xfertyp: u32,
    cmdrsp: [u32; 4],
    proctl: u32,
    sysctl: u32,
    irqstat: u32,
    irqstaten: u32,
    irqsigen: u32,
    wml: u32,
    admasar: u32,
    vendor: u32,
    /// CDIHB, DLA, BREN and BWEN.
    prs: u32,
    pending: Option<Pending>,
    buffer: VecDeque<u32>,
    /// The tick at which a block read still holding words times out.
    drain_by: Option<u64>,
    /// The tick a CMD14's word becomes ready: BREN and DCE come then, after the command's CC.
    word_at: Option<u64>,
    /// DAT0 is low until this tick.
    busy_until: u64,
    /// A CMD25's words still to come through DATPORT, and those that have.
    want: u32,
    incoming: Vec<u8>,
    write_arg: u32,
    /// The tick a busy that ends with TC (after CMD25's data or CMD38) ends.
    tc_at: Option<u64>,
    /// SDCLKEN was set by a SYSCTL write since the last reset (it then guards the divider).
    clock_written: bool,
    writes: Vec<(u32, u32)>,
    record: bool,
    /// The tick its clock gate closed, while it is closed.
    gated_at: Option<u64>,
}

impl Esdhc {
    /// An eSDHC at reset (RM Table 25-2) with `card` behind it (`None`: no card answers).
    pub fn new(config: EsdhcConfig, card: Option<Emmc>) -> Esdhc {
        Esdhc {
            config,
            card,
            dsaddr: 0,
            blkattr: BLKATTR_RESET,
            cmdarg: 0,
            xfertyp: 0,
            cmdrsp: [0; 4],
            proctl: PROCTL_RESET,
            sysctl: SYSCTL_RESET,
            irqstat: 0,
            irqstaten: IRQSTATEN_RESET,
            irqsigen: 0,
            wml: WML_RESET,
            admasar: 0,
            vendor: VENDOR_RESET,
            prs: 0,
            pending: None,
            buffer: VecDeque::new(),
            drain_by: None,
            word_at: None,
            busy_until: 0,
            want: 0,
            incoming: Vec::new(),
            write_arg: 0,
            tc_at: None,
            clock_written: false,
            writes: Vec::new(),
            record: true,
            gated_at: None,
        }
    }

    /// The card, if there is one.
    pub fn card(&self) -> Option<&Emmc> {
        self.card.as_ref()
    }

    /// The card, mutably.
    pub fn card_mut(&mut self) -> Option<&mut Emmc> {
        self.card.as_mut()
    }

    /// Every accepted register write, (offset, value), in order.
    pub fn writes(&self) -> &[(u32, u32)] {
        &self.writes
    }

    /// Every accepted register write so far, emptying the log.
    pub fn take_writes(&mut self) -> Vec<(u32, u32)> {
        std::mem::take(&mut self.writes)
    }

    /// Whether register writes are logged (default on). Off changes only the log, never the controller.
    pub fn record(&mut self, on: bool) {
        self.record = on;
    }

    fn signal(&self, cx: &mut DeviceCx) {
        let on = self.irqstat & self.irqsigen != 0;
        cx.irq.set(self.config.controller, self.config.source, on);
    }

    fn request(&self, cx: &mut DeviceCx) {
        let on = self.vendor & 1 != 0 && (!self.buffer.is_empty() || self.want > 0);
        cx.dma.set(self.config.dma, on);
    }

    /// Set the status bits of `status` that IRQSTATEN enables (RM 25.3.11).
    fn latch(&mut self, status: u32) {
        self.irqstat |= status & self.irqstaten;
    }

    /// The end of a data phase with `status`: TC frees the data line, an error leaves it busy.
    fn done(&mut self, status: u32, cx: &mut DeviceCx) {
        if status & TC != 0 {
            self.prs &= !(CDIHB | DLA | BREN | BWEN);
        }
        self.drain_by = None;
        self.latch(status);
        self.signal(cx);
    }

    /// Cancel any data phase and empty the buffer.
    fn reset_data(&mut self) {
        self.pending = None;
        self.buffer.clear();
        self.drain_by = None;
        self.word_at = None;
        self.want = 0;
        self.incoming.clear();
        self.tc_at = None;
        self.prs &= !(CDIHB | DLA | BREN | BWEN);
    }

    fn write_sysctl(&mut self, v: u32, cx: &mut DeviceCx) {
        let old = self.sysctl;
        if v & RSTA == 0 && self.clock_written && old & v & SDCLKEN != 0 && (old ^ v) & DIVIDER != 0
        {
            return cx.refuse(
                Stop::Unmodelled,
                format!(
                    "eSDHC: SYSCTL divider changed with the SD clock running ({old:#x} -> {v:#x})"
                ),
            );
        }
        if v & RSTA != 0 {
            // every register back to its reset value: measured on an MCF54415 for SYSCTL (the rest of the written
            // value is dropped), DSADDR, BLKATTR, CMDARG, PROCTL, IRQSTATEN, IRQSIGEN, WML, ADMASAR and VENDOR;
            // guess for the rest
            *self = Esdhc {
                card: self.card.take(),
                busy_until: self.busy_until,
                writes: std::mem::take(&mut self.writes),
                record: self.record,
                ..Esdhc::new(self.config, None)
            };
        }
        if v & (RSTA | RSTD) != 0 {
            self.reset_data();
            self.irqstat &= !RSTD_CLEARS;
            // measured on an MCF54415: not in RM Table 25-13's RSTD list
            self.proctl = PROCTL_RESET;
            self.wml = WML_RESET;
        }
        if v & (RSTA | RSTC) != 0 {
            self.irqstat &= !CC;
        }
        if v & RSTA == 0 {
            self.sysctl = v & !(RSTC | RSTD | INITA) & SYSCTL_BITS;
            self.clock_written = v & SDCLKEN != 0;
        }
        self.signal(cx);
        self.request(cx);
    }

    /// Why command `x` cannot be sent, if it cannot; else its data phase's transfer and block count.
    fn check(&self, x: u32, cx: &DeviceCx) -> Result<Option<(Transfer, u32)>, String> {
        let cmd = (x >> 24 & 0x3F) as u8;
        if let Some(why) = self
            .card
            .as_ref()
            .and_then(|c| c.unmodelled(cmd, self.cmdarg))
        {
            return Err(why);
        }
        if self.proctl >> 4 & 3 != 2 {
            return Err(format!(
                "eSDHC: CMD{cmd} with PROCTL EMODE other than address-invariant"
            ));
        }
        if x & DMAEN != 0 {
            return Err(format!(
                "eSDHC: CMD{cmd} with XFERTYP DMAEN: the internal DMA is not modelled"
            ));
        }
        if cx.now < self.busy_until {
            return Err(format!("eSDHC: CMD{cmd} while the card is busy (DAT0 low)"));
        }
        if x & DPSEL == 0 {
            return Ok(None);
        }
        if self.prs & CDIHB != 0 {
            return Err(format!("eSDHC: CMD{cmd} with the data line busy (CDIHB)"));
        }
        let Some(transfer) = Emmc::transfer(cmd) else {
            return Err(format!(
                "eSDHC: CMD{cmd} with DPSEL: the card has no data phase for it"
            ));
        };
        if !matches!(transfer, Transfer::Blocks | Transfer::BlocksWrite) {
            return Ok(Some((transfer, 0)));
        }
        let (count, size) = (self.blkattr >> 16, self.blkattr & 0x1FFF);
        if size != 512 {
            return Err(format!(
                "eSDHC: CMD{cmd} with BLKATTR BLKSIZE {size}: only 512-byte blocks are modelled"
            ));
        }
        let blocks = if x & MSBSEL == 0 {
            1
        } else if x & BCEN == 0 {
            return Err(format!(
                "eSDHC: CMD{cmd} as an infinite transfer (MSBSEL without BCEN) is not modelled"
            ));
        } else if count == 0 {
            return Err(format!(
                "eSDHC: CMD{cmd} with BLKATTR BLKCNT 0 is not modelled"
            ));
        } else {
            count
        };
        Ok(Some((transfer, blocks)))
    }

    fn command(&mut self, x: u32, cx: &mut DeviceCx) {
        let data = match self.check(x, cx) {
            Ok(d) => d,
            Err(why) => return cx.refuse(Stop::Unmodelled, why),
        };
        self.xfertyp = x;
        let (cmd, arg) = ((x >> 24 & 0x3F) as u8, self.cmdarg);
        let response = self.card.as_mut().and_then(|c| c.command(cmd, arg));
        let Some(response) = response else {
            self.latch(CTOE);
            return self.signal(cx);
        };
        match response {
            Response::Short(w) => self.cmdrsp[0] = w,
            Response::Long(r) => self.cmdrsp = r,
        }
        if cmd == 6 {
            self.busy_until = cx.now + self.config.cmd6_busy;
        }
        if cmd == 38 {
            // guess: an R1b command's busy ends with TC (RM 25.3.12), the busy time the board's
            self.busy_until = cx.now + self.config.erase_busy;
            self.tc_at = Some(self.busy_until);
        }
        if let Some((transfer, blocks)) = data {
            self.prs |= CDIHB | DLA;
            self.pending = Some(Pending {
                at: cx.now + self.config.data_latency,
                cmd,
                arg,
                blocks,
                transfer,
            });
        }
        self.latch(CC);
        self.signal(cx);
    }

    fn data_phase(&mut self, p: Pending, now: u64, cx: &mut DeviceCx) {
        let Some(card) = self.card.as_mut() else {
            return;
        };
        if (self.proctl & DTW_4BIT != 0) != card.wide() {
            // measured on an MCF54415: a host and card bus width mismatch fails every read's CRC
            return self.done(DCE, cx);
        }
        match p.transfer {
            Transfer::Blocks => match card.read_blocks(p.cmd, p.arg, p.blocks) {
                None => self.done(DCE, cx),
                Some(data) => {
                    self.buffer = data
                        .chunks(4)
                        .map(|w| w.iter().fold(0, |v, &b| v << 8 | u32::from(b)))
                        .collect();
                    // guess: one tick for the DMA to drain it (the RM's data timeout counter is not modelled)
                    self.drain_by = Some(now + 1);
                    self.request(cx);
                }
            },
            Transfer::BlocksWrite => {
                self.want = p.blocks * 128;
                self.incoming.clear();
                self.write_arg = p.arg;
                // guess: one tick for the DMA to fill it, as for a read
                self.drain_by = Some(now + 1);
                self.request(cx);
            }
            Transfer::BusTestRead => {
                self.buffer.push_back(card.bus_test_read());
                // guess: the bus-test word carries no CRC16, so the data phase ends with DCE, as CMD19's does
                // (measured). A driver waits for an interrupt status before it reads DATPORT; which status the chip
                // latches here is not measured. The word and the status come after the command's CC, so a driver
                // polling for CC alone (the HAL's bus test) sees no error: one tick after the first IRQSTAT read
                // that shows CC, or `WORD_DELAY` ticks after the data phase begins, whichever is first.
                self.word_at = Some(now + WORD_DELAY);
            }
            Transfer::BusTestWrite => self.prs |= BWEN,
        }
    }

    fn read_datport(&mut self, cx: &mut DeviceCx) -> u32 {
        let Some(v) = self.buffer.pop_front() else {
            return 0; // measured on an MCF54415: an empty buffer reads 0
        };
        if self.buffer.is_empty() {
            self.done(TC, cx);
        }
        self.request(cx);
        v
    }

    fn write_datport(&mut self, v: u32, cx: &mut DeviceCx) {
        if self.want > 0 {
            self.incoming.extend_from_slice(&v.to_be_bytes());
            self.want -= 1;
            if self.want == 0 {
                self.drain_by = None;
                self.request(cx);
                let data = std::mem::take(&mut self.incoming);
                let ok = self
                    .card
                    .as_mut()
                    .is_some_and(|c| c.write_blocks(self.write_arg, &data));
                if !ok {
                    return self.done(DCE, cx);
                }
                self.busy_until = cx.now + self.config.write_busy;
                self.tc_at = Some(self.busy_until);
            }
            return;
        }
        if self.prs & BWEN == 0 {
            return; // written before BWEN: lost
        }
        if let Some(card) = self.card.as_mut() {
            card.bus_test_write(v);
        }
        // measured on an MCF54415: CMD19 ends with DCE even when the bus test passes. guess: TC as well, freeing
        // the data line (the bus-test sequence waits for TC, then sends CMD14)
        self.done(TC | DCE, cx);
    }
}

impl Device for Esdhc {
    fn read(&mut self, offset: u32, size: Size, cx: &mut DeviceCx) -> u32 {
        if size != Size::Long || !offset.is_multiple_of(4) {
            cx.refuse(
                Stop::Unmodelled,
                format!(
                    "eSDHC: {}-byte read at +{offset:#x} (aligned 32-bit accesses only)",
                    size.bytes()
                ),
            );
            return 0;
        }
        match offset {
            DSADDR => self.dsaddr,
            // measured on an MCF54415: BLKCNT reads 1 (guess: the written count is kept for the transfer)
            BLKATTR => self.blkattr & 0x1FFF | 1 << 16,
            CMDARG => self.cmdarg,
            XFERTYP => self.xfertyp,
            CMDRSP0..=CMDRSP3 => self.cmdrsp[((offset - CMDRSP0) / 4) as usize],
            DATPORT => self.read_datport(cx),
            PRSSTAT => {
                let mut v = PRSSTAT_LEVELS | self.prs;
                if cx.now < self.busy_until {
                    v &= !DAT0;
                }
                v
            }
            PROCTL => self.proctl,
            SYSCTL => self.sysctl,
            IRQSTAT => {
                // guess: a driver that has seen the command complete finds the word's status a little later
                if self.irqstat & CC != 0
                    && let Some(t) = &mut self.word_at
                {
                    *t = (*t).min(cx.now + SEEN_DELAY);
                }
                self.irqstat
            }
            IRQSTATEN => self.irqstaten,
            IRQSIGEN => self.irqsigen,
            HOSTCAPBLT => HOSTCAPBLT_VALUE,
            WML => self.wml,
            ADMASAR => self.admasar,
            VENDOR => self.vendor,
            HOSTVER => HOSTVER_VALUE,
            _ => 0,
        }
    }

    fn write(&mut self, offset: u32, size: Size, value: u32, cx: &mut DeviceCx) {
        if size != Size::Long || !offset.is_multiple_of(4) {
            return cx.refuse(
                Stop::Unmodelled,
                format!(
                    "eSDHC: {}-byte write at +{offset:#x} (aligned 32-bit accesses only)",
                    size.bytes()
                ),
            );
        }
        match offset {
            DSADDR => self.dsaddr = value,
            BLKATTR => self.blkattr = value & 0xFFFF_1FFF,
            CMDARG => self.cmdarg = value,
            XFERTYP => self.command(value, cx),
            DATPORT => self.write_datport(value, cx),
            PROCTL => self.proctl = value & PROCTL_BITS,
            SYSCTL => self.write_sysctl(value, cx),
            IRQSTAT => {
                self.irqstat &= !value;
                self.signal(cx);
            }
            IRQSTATEN => {
                self.irqstaten = value & IRQ_BITS;
                self.irqstat &= self.irqstaten;
                self.signal(cx);
            }
            IRQSIGEN if value & !IRQ_BITS != 0 => cx.refuse(
                Stop::Unmodelled,
                format!("eSDHC: IRQSIGEN {value:#010x} sets reserved bits"),
            ),
            IRQSIGEN => {
                self.irqsigen = value;
                self.signal(cx);
            }
            WML => self.wml = WML_ONES | value & WML_LEVELS,
            FEVT => {
                self.latch(value & IRQ_BITS);
                self.signal(cx);
            }
            ADMASAR => self.admasar = value,
            VENDOR => {
                self.vendor = value & 1;
                self.request(cx);
            }
            _ => {}
        }
        if self.record && cx.refused.is_none() {
            self.writes.push((offset, value));
        }
    }

    fn next_event(&self) -> Option<u64> {
        [self.pending.map(|p| p.at), self.drain_by, self.word_at, self.tc_at]
            .into_iter()
            .flatten()
            .min()
    }

    fn advance(&mut self, now: u64, cx: &mut DeviceCx) {
        if let Some(p) = self.pending
            && now >= p.at
        {
            self.pending = None;
            self.data_phase(p, now, cx);
        }
        if let Some(t) = self.word_at
            && now >= t
        {
            self.word_at = None;
            self.prs |= BREN;
            self.latch(DCE);
            self.signal(cx);
            self.request(cx);
        }
        if let Some(t) = self.drain_by
            && now >= t
        {
            self.drain_by = None;
            if !self.buffer.is_empty() || self.want > 0 {
                self.buffer.clear();
                self.want = 0;
                self.incoming.clear();
                self.request(cx);
                self.done(DTOE, cx);
            }
        }
        if let Some(t) = self.tc_at
            && now >= t
        {
            self.tc_at = None;
            self.done(TC, cx);
        }
    }

    fn poll_counted(&self) -> bool {
        false
    }

    // guess: a gated eSDHC holds its data phase, its drain timeout and the card's busy time, and resumes them
    // where they stopped when the gate opens (not measured).
    fn gate(&mut self, closed: bool, now: u64) {
        if closed {
            self.gated_at = Some(now);
        } else if let Some(at) = self.gated_at.take() {
            let d = now - at;
            if let Some(p) = &mut self.pending {
                p.at += d;
            }
            self.drain_by = self.drain_by.map(|t| t + d);
            self.word_at = self.word_at.map(|t| t + d);
            self.tc_at = self.tc_at.map(|t| t + d);
            if self.busy_until > at {
                self.busy_until += d;
            }
        }
    }

    /// The eSDHC interrupt, while a data phase is pending or its buffer waits to be drained.
    fn wake_sources(&self, out: &mut Vec<(usize, u8)>) -> bool {
        if self.next_event().is_some() {
            out.push((self.config.controller, self.config.source));
        }
        true
    }
}
