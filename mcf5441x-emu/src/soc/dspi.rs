//! The DMA serial peripheral interfaces DSPI0-DSPI3 (RM ch. 40), as SPI masters with parts on their chip selects.

use std::any::Any;
use std::fmt;

use crate::device::{Device, DeviceCx};
use crate::mem::Size;
use crate::stop::Stop;

const MCR: u32 = 0x00;
const TCR: u32 = 0x08;
/// CTAR0-CTAR7 (RM Table 40-3).
const CTAR0: u32 = 0x0C;
const CTAR7: u32 = 0x28;
const SR: u32 = 0x2C;
const RSER: u32 = 0x30;
const PUSHR: u32 = 0x34;
const POPR: u32 = 0x38;
/// TXFR0-TXFR15 and RXFR0-RXFR15 (RM 40.3.8, RM 40.3.9).
const TXFR0: u32 = 0x3C;
const RXFR0: u32 = 0x7C;
const RXFR15: u32 = 0xB8;

/// MCR (RM Table 40-4).
const MSTR: u32 = 1 << 31;
const DCONF: u32 = 3 << 28;
const MTFE: u32 = 1 << 26;
const ROOE: u32 = 1 << 24;
const MDIS: u32 = 1 << 14;
const DIS_TXF: u32 = 1 << 13;
const DIS_RXF: u32 = 1 << 12;
const CLR_TXF: u32 = 1 << 11;
const CLR_RXF: u32 = 1 << 10;
const HALT: u32 = 1 << 0;

/// SR (RM Table 40-7).
const TCF: u32 = 1 << 31;
const TXRXS: u32 = 1 << 30;
const EOQF: u32 = 1 << 28;
const TFFF: u32 = 1 << 25;
const RFOF: u32 = 1 << 19;
const RFDF: u32 = 1 << 17;

/// RSER (RM Table 40-8): the enables and request selects of TFFF and RFDF, the only requests the model drives.
const TFFF_RE: u32 = 1 << 25;
const TFFF_DIRS: u32 = 1 << 24;
const RFDF_RE: u32 = 1 << 17;
const RFDF_DIRS: u32 = 1 << 16;
/// The interrupt-only enables: TCF_RE, EOQF_RE, TFUF_RE, RFOF_RE.
const IRQ_RE: u32 = 1 << 31 | 1 << 28 | 1 << 27 | 1 << 19;

/// PUSHR (RM Table 40-9).
const CONT: u32 = 1 << 31;
const EOQ: u32 = 1 << 27;
const CTCNT: u32 = 1 << 26;

/// CTAR\[LSBFE\] (RM Table 40-6).
const LSBFE: u32 = 1 << 24;

/// The depth of each FIFO (RM 40.4.2.4, RM 40.4.2.5).
const FIFO: usize = 16;
/// The chip selects PCS0-PCS7 a PUSHR command can assert (RM Table 40-9).
const SELECTS: usize = 8;

/// A part on one of a DSPI's chip selects, seen byte by byte. Every part is taken to be selected by a low chip
/// select.
pub trait SpiDevice: Any {
    /// The chip select is asserted at tick `now`: a command begins.
    fn select(&mut self, now: u64);
    /// Shift one byte out to the part, MSB first; returns the byte it shifts back.
    fn transfer(&mut self, byte: u8) -> u8;
    /// The chip select is negated at tick `now`: the command ends.
    fn deselect(&mut self, now: u64);
    /// The description of a feature the part does not model that the last call reached (e.g. an opcode), naming
    /// the part and the feature; cleared by this call. The DSPI refuses the access that reached it.
    fn take_unmodelled(&mut self) -> Option<String> {
        None
    }
}

/// One frame shifted out, as [`Dspi::frame_log`] records it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Frame {
    /// The tick it was shifted out.
    pub at: u64,
    /// TXDATA within the frame size.
    pub data: u32,
    /// PUSHR\[PCS\]: bit n asserts PCSn.
    pub pcs: u8,
    /// PUSHR\[CONT\].
    pub cont: bool,
    /// PUSHR\[CTAS\]: the CTAR that set its size.
    pub ctas: u8,
}

/// One DSPI (RM ch. 40) in master mode, with [`SpiDevice`] parts on its chip selects
/// ([`Dspi::attach`]). A frame takes no time: it is shifted out, and its answer is in the RX FIFO, when it is
/// pushed. Every frame shifted out is recorded ([`Dspi::frame_log`]) unless [`Dspi::record`] turned the log off,
/// so the DSPI also serves as a recorder for write-only parts with nothing behind their select.
///
/// Registers (32-bit accesses only: a 1- or 2-byte access is refused with [`Stop::Unmodelled`], though the chip
/// takes 8- and 16-bit PUSHR writes and POPR reads):
/// - MCR (+0x00, reset 0x0000_4001: HALT and MDIS, RM Table 40-3): stored; CLR_TXF and CLR_RXF read as zero (RM
///   Table 40-4) and empty the FIFOs only in a write with HALT clear: with HALT set they do nothing, MDIS set or
///   clear (measured on an MCF54415; guess: in a write with HALT clear and MDIS set they do nothing either, RM
///   40.4.7.1). PCSISn sets the level PCSn idles at.
/// - TCR (+0x08): SPI_TCNT (bits 31:16) counts frames and wraps; a write presets it, and a frame with CTCNT clears
///   it before it is counted (RM 40.3.2).
/// - CTAR0-CTAR7 (+0x0C..+0x28, reset 0x7800_0000: 16-bit frames): stored, all eight, each picked by a frame's
///   PUSHR\[CTAS\]. Only FMSZ (the frame size) and LSBFE act; the baud rate and the delays do not, as frames take
///   no time.
/// - SR (+0x2C): TCF, EOQF and RFOF are set by the frames and cleared by writing 1 (RM Table 40-7). TXRXS is set
///   while the DSPI runs: HALT and EOQF clear (RM 40.4.1). TFFF is set while the TX FIFO is not full and MDIS is
///   clear, RFDF while the RX FIFO is not empty. Writing 1 to TFFF changes nothing (measured on an MCF54415);
///   guess: RFDF likewise follows the FIFO. TXCTR, TXNXTPTR, RXCTR and POPNXTPTR count and point into the FIFOs (guess: the 4-bit counters read
///   15 with 16 entries held, as the older model did; the pointers wrap at 16). TFUF (slave mode) is never set.
/// - RSER (+0x30, reset 0): stored and read back. TFFF_RE with TFFF_DIRS drives the TX FIFO fill DMA request
///   while SR\[TFFF\] is set, RFDF_RE with RFDF_DIRS the RX FIFO drain DMA request while SR\[RFDF\] is set (RM
///   Table 40-8, RM 40.4.6.2, RM 40.4.6.5), on the lines [`Dspi::with_dma`] gives (RM Table 19-6). A DIRS bit
///   without its RE bit requests nothing and is only stored. A write that enables an interrupt request (TCF_RE,
///   EOQF_RE, TFUF_RE, RFOF_RE, or TFFF_RE or RFDF_RE without its DIRS), sets a reserved bit, or enables a DMA
///   request on a DSPI with no request lines is refused, and RSER keeps its value.
/// - PUSHR (+0x34): pushes a command and data into the TX FIFO (a push to a full FIFO is ignored, RM 40.4.2.4.1);
///   while the DSPI runs the FIFO drains at once, frame by frame, and while it is halted the frames wait there
///   (measured on an MCF54415: three pushes to a halted master read SR 0x0200_3000, TXFR0-TXFR2 holding them).
///   Reads return the last word pushed (measured on an MCF54415; 0 at reset, RM Table 40-3).
/// - POPR (+0x38): pops the RX FIFO (with MDIS set it reads the entry without popping it, RM 40.4.7.1).
/// - TXFR0-TXFR15 (+0x3C..+0x78) and RXFR0-RXFR15 (+0x7C..+0xB8): the FIFO entries, read-only.
///
/// DMA requests: each line follows its flag after every access, so an eDMA write to PUSHR or read of POPR moves the
/// line before the access returns. Frames take no time, so while the DSPI runs the TX FIFO never fills and the fill
/// request stays up: the eDMA serving it runs minor loops until its major loop ends (D_REQ clears ERQ) or an EOQ
/// frame stops the DSPI and the FIFO fills (the push that fills it drops the request). Writing 1 to TFFF or RFDF
/// does not lower a request (measured on an MCF54415 for TFFF; guess for RFDF, as above).
///
/// The rest of the window reads 0. A write anywhere but MCR, TCR, CTARn, SR, RSER and PUSHR is refused.
///
/// A frame (RM 40.4.2.1): PUSHR\[CTAS\] picks the CTAR whose FMSZ sets the frame size; TXDATA goes out MSB first,
/// a 16-bit frame as two bytes, high byte first, and the bytes shifted back make the RX FIFO entry. With the RX
/// FIFO full the frame's data is dropped and RFOF set (MCR\[ROOE\] = 0, RM 40.4.2.5.1). The frame sets TCF,
/// and EOQF when its EOQ is set, which stops the DSPI until EOQF is cleared (RM 40.4.1); frames pushed meanwhile
/// wait in the TX FIFO.
///
/// Chip selects: a part on PCSn is selected during a frame whose PUSHR\[PCSn\] is set, or whose MCR\[PCSISn\] is
/// clear: PCSn then idles low, the active-low part's selected level (guess: the frame asserting another PCS, or
/// none, still reaches it). A frame reaching no part reads all ones. A part is deselected at the end of a frame
/// without CONT (also when it was selected by an idle-low PCS, as in the older model; on the wire it stays
/// selected), and before a frame that reaches another part or none (RM 40.4.4.5). The RM brings out PCS0-PCS3 on
/// DSPI0, PCS0-PCS2 on DSPI1 and PCS0 alone on DSPI2 and DSPI3 (RM Table 40-4); the model takes the PUSHR field's
/// eight, and which of them hold parts is the board's business.
///
/// Refused with [`Stop::Unmodelled`] (the detail names the DSPI and the feature): a 1- or 2-byte access, a POPR
/// read of an empty RX FIFO, a push to a full TX FIFO (it fills only while HALT or EOQF stops the DSPI; the chip
/// ignores the push, RM 40.4.2.4.1), a frame pushed or going out while disabled
/// (MCR\[MDIS\]), in slave mode, with a DCONF other than SPI, with the modified transfer format (MCR\[MTFE\]) or
/// with a FIFO disabled, a frame size other than 8 or 16 bits, LSB first, a frame selecting more than one attached
/// part, an RX overflow with MCR\[ROOE\] set, the RSER writes above, and whatever an attached part reports with
/// [`SpiDevice::take_unmodelled`]. A refused push changes nothing, unless a part refused it: the part has then seen
/// the frame.
pub struct Dspi {
    mcr: u32,
    tcnt: u16,
    ctar: [u32; 8],
    /// TCF, EOQF and RFOF.
    flags: u32,
    tx: [u32; FIFO],
    /// TXNXTPTR and TXCTR.
    tx_next: usize,
    tx_count: usize,
    /// The last word pushed: what PUSHR reads.
    pushr: u32,
    rx: [u32; FIFO],
    /// POPNXTPTR and RXCTR.
    pop_next: usize,
    rx_count: usize,
    parts: [Option<Box<dyn SpiDevice>>; SELECTS],
    /// The part selected now.
    selected: Option<usize>,
    rser: u32,
    /// The receive (RFDF) and transmit (TFFF) DMA request lines.
    dma: Option<(u8, u8)>,
    frames: Vec<Frame>,
    record: bool,
}

impl Frame {
    fn tuple(&self) -> (u64, u32, u8, bool) {
        (self.at, self.data, self.pcs, self.cont)
    }
}

impl fmt::Debug for Dspi {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let attached: Vec<usize> = (0..SELECTS).filter(|&i| self.parts[i].is_some()).collect();
        f.debug_struct("Dspi")
            .field("mcr", &format_args!("{:#010x}", self.mcr))
            .field("attached", &attached)
            .field("selected", &self.selected)
            .field("tx_count", &self.tx_count)
            .field("rx_count", &self.rx_count)
            .field("frames", &self.frames.len())
            .finish_non_exhaustive()
    }
}

impl Default for Dspi {
    fn default() -> Dspi {
        Dspi::new()
    }
}

impl Dspi {
    /// A DSPI at reset (RM Table 40-3), with nothing on its chip selects and no DMA request lines.
    pub fn new() -> Dspi {
        Dspi {
            mcr: MDIS | HALT,
            tcnt: 0,
            ctar: [0x7800_0000; 8],
            flags: 0,
            tx: [0; FIFO],
            pushr: 0,
            tx_next: 0,
            tx_count: 0,
            rx: [0; FIFO],
            pop_next: 0,
            rx_count: 0,
            parts: Default::default(),
            selected: None,
            rser: 0,
            dma: None,
            frames: Vec::new(),
            record: true,
        }
    }

    /// A DSPI at reset whose RX FIFO drain request is DMA request line `rx` and TX FIFO fill request line `tx`
    /// (RM Table 19-6).
    pub fn with_dma(rx: u8, tx: u8) -> Dspi {
        Dspi {
            dma: Some((rx, tx)),
            ..Dspi::new()
        }
    }

    /// Put `dev` on chip select PCS`pcs` (0-7; which of them a DSPI brings out is the board's business, RM Table
    /// 40-4), in place of any part there.
    pub fn attach(&mut self, pcs: u8, dev: Box<dyn SpiDevice>) {
        assert!(usize::from(pcs) < SELECTS, "no PCS{pcs}: PCS0-PCS7 only");
        self.parts[usize::from(pcs)] = Some(dev);
    }

    /// Every frame shifted out, with its CTAS.
    pub fn frame_log(&self) -> &[Frame] {
        &self.frames
    }

    /// Every frame shifted out so far, emptying the log.
    pub fn take_frame_log(&mut self) -> Vec<Frame> {
        std::mem::take(&mut self.frames)
    }

    /// Every frame shifted out, without its CTAS: (tick, TXDATA within the frame size, PUSHR\[PCS\],
    /// PUSHR\[CONT\]). This builds a new vector; [`frame_log`](Self::frame_log) is the cheap accessor.
    pub fn frames(&self) -> Vec<(u64, u32, u8, bool)> {
        self.frames.iter().map(Frame::tuple).collect()
    }

    /// Every frame shifted out so far, without its CTAS, emptying the log.
    pub fn take_frames(&mut self) -> Vec<(u64, u32, u8, bool)> {
        self.take_frame_log().iter().map(Frame::tuple).collect()
    }

    /// Whether frames are logged (default on). Off changes only the log, never the transfers.
    pub fn record(&mut self, on: bool) {
        self.record = on;
    }

    /// The part on PCS`pcs`. Panics when there is none or it is not a `T`.
    pub fn device<T: SpiDevice>(&self, pcs: u8) -> &T {
        let dev: &dyn Any = self
            .parts
            .get(usize::from(pcs))
            .and_then(Option::as_deref)
            .unwrap_or_else(|| panic!("nothing on PCS{pcs}"));
        dev.downcast_ref().unwrap_or_else(|| {
            panic!(
                "the part on PCS{pcs} is not a {}",
                std::any::type_name::<T>()
            )
        })
    }

    /// The part on PCS`pcs`, mutably. Panics when there is none or it is not a `T`.
    pub fn device_mut<T: SpiDevice>(&mut self, pcs: u8) -> &mut T {
        let dev: &mut dyn Any = self
            .parts
            .get_mut(usize::from(pcs))
            .and_then(Option::as_deref_mut)
            .unwrap_or_else(|| panic!("nothing on PCS{pcs}"));
        dev.downcast_mut().unwrap_or_else(|| {
            panic!(
                "the part on PCS{pcs} is not a {}",
                std::any::type_name::<T>()
            )
        })
    }

    fn running(&self) -> bool {
        self.mcr & HALT == 0 && self.flags & EOQF == 0
    }

    fn sr(&self) -> u32 {
        let mut v = self.flags;
        if self.running() {
            v |= TXRXS;
        }
        if self.tx_count < FIFO && self.mcr & MDIS == 0 {
            v |= TFFF;
        }
        if self.rx_count > 0 {
            v |= RFDF;
        }
        let count = |n: usize| n.min(15) as u32;
        v | count(self.tx_count) << 12
            | (self.tx_next as u32) << 8
            | count(self.rx_count) << 4
            | self.pop_next as u32
    }

    /// Drive the DMA request lines from the flags.
    fn drive(&self, cx: &mut DeviceCx) {
        if let Some((rx, tx)) = self.dma {
            let sr = self.sr();
            let on = |re: u32, dirs: u32, flag: u32| {
                self.rser & (re | dirs) == re | dirs && sr & flag != 0
            };
            cx.dma.set(rx, on(RFDF_RE, RFDF_DIRS, RFDF));
            cx.dma.set(tx, on(TFFF_RE, TFFF_DIRS, TFFF));
        }
    }

    /// An RSER write: stored, or why it is refused.
    fn write_rser(&mut self, v: u32) -> Result<(), String> {
        let irq = IRQ_RE
            | if v & TFFF_DIRS == 0 { TFFF_RE } else { 0 }
            | if v & RFDF_DIRS == 0 { RFDF_RE } else { 0 };
        let known = IRQ_RE | TFFF_RE | TFFF_DIRS | RFDF_RE | RFDF_DIRS;
        if v & irq != 0 {
            return Err(format!(
                "DSPI: RSER {v:#010x}: interrupt requests are not modelled"
            ));
        }
        if v & !known != 0 {
            return Err(format!("DSPI: RSER {v:#010x}: reserved bits set"));
        }
        if v & (TFFF_RE | RFDF_RE) != 0 && self.dma.is_none() {
            return Err(format!(
                "DSPI: RSER {v:#010x}: a DMA request with no request line wired"
            ));
        }
        self.rser = v;
        Ok(())
    }

    /// The frame size of command `v`, and the part it reaches; or why it is refused.
    fn check(&self, v: u32) -> Result<(u32, Option<usize>), String> {
        let ctas = v >> 28 & 7;
        let ctar = self.ctar[ctas as usize];
        let bits = (ctar >> 27 & 0xF) + 1;
        if bits != 8 && bits != 16 {
            return Err(format!(
                "DSPI: {bits}-bit frames (CTAR{ctas} FMSZ) are not modelled"
            ));
        }
        if ctar & LSBFE != 0 {
            return Err(format!(
                "DSPI: LSB-first frames (CTAR{ctas} LSBFE) are not modelled"
            ));
        }
        let low = (v >> 16) as u8 | !(self.mcr >> 16) as u8;
        let reached: Vec<usize> = (0..SELECTS)
            .filter(|&i| low >> i & 1 != 0 && self.parts[i].is_some())
            .collect();
        match reached[..] {
            [] => Ok((bits, None)),
            [one] => Ok((bits, Some(one))),
            _ => {
                let names: Vec<String> = reached.iter().map(|i| format!("PCS{i}")).collect();
                Err(format!(
                    "DSPI: one frame selects the parts on {} at once",
                    names.join(" and ")
                ))
            }
        }
    }

    /// Why the MCR's configuration cannot shift frames, if it cannot.
    fn mode(&self) -> Option<String> {
        let what = if self.mcr & MDIS != 0 {
            "frames while disabled (MCR MDIS)"
        } else if self.mcr & MSTR == 0 {
            "slave mode (MCR MSTR clear) is not modelled"
        } else if self.mcr & DCONF != 0 {
            "MCR DCONF other than SPI is not modelled"
        } else if self.mcr & MTFE != 0 {
            "the modified transfer format (MCR MTFE) is not modelled"
        } else if self.mcr & (DIS_TXF | DIS_RXF) != 0 {
            "disabled FIFOs (MCR DIS_TXF, DIS_RXF) are not modelled"
        } else {
            return None;
        };
        Some(format!("DSPI: {what}"))
    }

    fn push(&mut self, v: u32, cx: &mut DeviceCx) {
        if let Some(d) = self.mode() {
            return cx.refuse(Stop::Unmodelled, d);
        }
        if let Err(d) = self.check(v) {
            return cx.refuse(Stop::Unmodelled, d);
        }
        if self.tx_count == FIFO {
            // the chip ignores the push (RM 40.4.2.4.1): the frame would be lost without a trace. The FIFO only
            // fills while the DSPI is stopped.
            let why = if self.mcr & HALT != 0 {
                "MCR HALT"
            } else {
                "EOQF set"
            };
            return cx.refuse(
                Stop::Unmodelled,
                format!("DSPI: PUSHR with the TX FIFO full (stopped: {why})"),
            );
        }
        self.pushr = v;
        self.tx[(self.tx_next + self.tx_count) % FIFO] = v;
        self.tx_count += 1;
        self.run(cx);
    }

    /// Shift out the TX FIFO's frames while the DSPI runs.
    fn run(&mut self, cx: &mut DeviceCx) {
        while self.running() && self.tx_count > 0 {
            if let Some(d) = self.mode() {
                return cx.refuse(Stop::Unmodelled, d);
            }
            let v = self.tx[self.tx_next];
            let (bits, part) = match self.check(v) {
                Ok(f) => f,
                Err(d) => return cx.refuse(Stop::Unmodelled, d),
            };
            self.tx_next = (self.tx_next + 1) % FIFO;
            self.tx_count -= 1;
            if !self.shift(v, bits, part, cx) {
                return;
            }
        }
    }

    /// Ask the part on `i` what it does not model; refuse the access with it. Returns whether all is well.
    fn part_ok(&mut self, i: usize, cx: &mut DeviceCx) -> bool {
        match self.parts[i].as_mut().and_then(|p| p.take_unmodelled()) {
            Some(d) => {
                cx.refuse(Stop::Unmodelled, d);
                false
            }
            None => true,
        }
    }

    fn deselect(&mut self, cx: &mut DeviceCx) -> bool {
        match self.selected.take() {
            Some(i) => {
                if let Some(p) = &mut self.parts[i] {
                    p.deselect(cx.now);
                }
                self.part_ok(i, cx)
            }
            None => true,
        }
    }

    /// One frame of `bits` bits to `part`. Returns false when it was refused.
    fn shift(&mut self, v: u32, bits: u32, part: Option<usize>, cx: &mut DeviceCx) -> bool {
        let mut ok = true;
        if self.selected != part {
            ok &= self.deselect(cx);
        }
        if v & CTCNT != 0 {
            self.tcnt = 0;
        }
        let mask = (1u32 << bits) - 1;
        let data = v & mask;
        let cont = v & CONT != 0;
        if self.record {
            self.frames.push(Frame {
                at: cx.now,
                data,
                pcs: (v >> 16) as u8,
                cont,
                ctas: (v >> 28 & 7) as u8,
            });
        }
        let r = match part {
            None => mask,
            Some(i) => {
                if self.selected.is_none() {
                    self.selected = Some(i);
                    if let Some(p) = &mut self.parts[i] {
                        p.select(cx.now);
                    }
                    ok &= self.part_ok(i, cx);
                }
                let mut r = 0;
                for k in (0..bits / 8).rev() {
                    let b = self.parts[i]
                        .as_mut()
                        .map_or(0xFF, |p| p.transfer((data >> (8 * k)) as u8));
                    r = r << 8 | u32::from(b);
                    ok &= self.part_ok(i, cx);
                }
                r
            }
        };
        self.tcnt = self.tcnt.wrapping_add(1);
        if self.rx_count < FIFO {
            self.rx[(self.pop_next + self.rx_count) % FIFO] = r;
            self.rx_count += 1;
        } else {
            self.flags |= RFOF;
            if self.mcr & ROOE != 0 {
                cx.refuse(
                    Stop::Unmodelled,
                    "DSPI: an RX FIFO overflow with MCR ROOE set is not modelled",
                );
                ok = false;
            }
        }
        self.flags |= TCF;
        if v & EOQ != 0 {
            self.flags |= EOQF;
        }
        if !cont && part.is_some() {
            // guess: a part reached only because its PCS idles low is deselected here too, as the older model did;
            // on the wire its select stays low
            ok &= self.deselect(cx);
        }
        ok
    }

    fn refuse_size(&self, offset: u32, size: Size, dir: &str, cx: &mut DeviceCx) {
        cx.refuse(
            Stop::Unmodelled,
            format!(
                "DSPI: {}-byte {dir} at +{offset:#x} (32-bit accesses only)",
                size.bytes()
            ),
        );
    }
}

impl Device for Dspi {
    fn read(&mut self, offset: u32, size: Size, cx: &mut DeviceCx) -> u32 {
        if size != Size::Long {
            self.refuse_size(offset, size, "read", cx);
            return 0;
        }
        match offset {
            MCR => self.mcr,
            TCR => u32::from(self.tcnt) << 16,
            CTAR0..=CTAR7 if offset % 4 == 0 => self.ctar[((offset - CTAR0) / 4) as usize],
            SR => self.sr(),
            RSER => self.rser,
            PUSHR => self.pushr,
            POPR => {
                if self.rx_count == 0 {
                    cx.refuse(Stop::Unmodelled, "DSPI: POPR read with the RX FIFO empty");
                    return 0;
                }
                let v = self.rx[self.pop_next];
                if self.mcr & MDIS == 0 {
                    self.pop_next = (self.pop_next + 1) % FIFO;
                    self.rx_count -= 1;
                }
                self.drive(cx);
                v
            }
            TXFR0..RXFR0 if offset % 4 == 0 => self.tx[((offset - TXFR0) / 4) as usize],
            RXFR0..=RXFR15 if offset % 4 == 0 => self.rx[((offset - RXFR0) / 4) as usize],
            _ => 0,
        }
    }

    fn write(&mut self, offset: u32, size: Size, value: u32, cx: &mut DeviceCx) {
        if size != Size::Long {
            return self.refuse_size(offset, size, "write", cx);
        }
        match offset {
            MCR => {
                // measured on an MCF54415: the clears act only in a write with HALT clear. guess: not with MDIS set
                // either (RM 40.4.7.1: not while disabled)
                if value & (HALT | MDIS) == 0 {
                    if value & CLR_TXF != 0 {
                        self.tx_count = 0;
                    }
                    if value & CLR_RXF != 0 {
                        self.rx_count = 0;
                    }
                }
                self.mcr = value & !(CLR_TXF | CLR_RXF);
                self.run(cx);
            }
            TCR => self.tcnt = (value >> 16) as u16,
            CTAR0..=CTAR7 if offset % 4 == 0 => self.ctar[((offset - CTAR0) / 4) as usize] = value,
            SR => {
                self.flags &= !(value & (TCF | EOQF | RFOF));
                self.run(cx);
            }
            RSER => {
                if let Err(d) = self.write_rser(value) {
                    cx.refuse(Stop::Unmodelled, d);
                }
            }
            PUSHR => self.push(value, cx),
            _ => cx.refuse(
                Stop::Unmodelled,
                format!("DSPI: write to +{offset:#x} (read-only or reserved)"),
            ),
        }
        self.drive(cx);
    }

    fn poll_counted(&self) -> bool {
        false
    }

    /// None: the DSPI raises no interrupts and has no events (the eDMA lists the routes of its DMA requests).
    fn wake_sources(&self, _out: &mut Vec<(usize, u8)>) -> bool {
        true
    }
}
