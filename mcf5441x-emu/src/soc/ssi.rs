//! The synchronous serial interfaces SSI0 and SSI1 (RM ch. 35), as an I2S transmitter and receiver.

use std::collections::VecDeque;
use std::fmt;

use crate::device::{Device, DeviceCx};
use crate::mem::Size;
use crate::stop::Stop;

const TX0: u32 = 0x00;
const TX1: u32 = 0x04;
const RX0: u32 = 0x08;
const RX1: u32 = 0x0C;
const CR: u32 = 0x10;
const ISR: u32 = 0x14;
const IER: u32 = 0x18;
const TCR: u32 = 0x1C;
const RCR: u32 = 0x20;
const CCR: u32 = 0x24;
const FCSR: u32 = 0x2C;
/// ACR, ACADD, ACDAT, ATAG, TMASK, RMASK, ACCSR, ACCEN, ACCDIS (RM Table 35-4).
const ACR: u32 = 0x38;
const TMASK: u32 = 0x48;
const RMASK: u32 = 0x4C;
const ACCDIS: u32 = 0x58;

/// CR (RM Table 35-7).
const SSI_EN: u32 = 0x01;
const TE: u32 = 0x02;
const RE: u32 = 0x04;
const NET: u32 = 0x08;
const SYN: u32 = 0x10;
const I2S: u32 = 0b11 << 5;

/// ISR (RM Table 35-8).
const TFE0: u32 = 0x0001;
const TFE1: u32 = 0x0002;
const RFF0: u32 = 0x0004;
const TUE0: u32 = 0x0100;
const ROE0: u32 = 0x0400;
const TDE0: u32 = 0x1000;
const TDE1: u32 = 0x2000;
const RDR0: u32 = 0x4000;
const TFRC: u32 = 0x0080_0000;

/// IER (RM Table 35-9).
const TDMAE: u32 = 0x0010_0000;
const RDMAE: u32 = 0x0040_0000;

/// TCR and RCR (RM Table 35-10, Table 35-11).
const TFEN0: u32 = 0x80;
const TFEN1: u32 = 0x100;
const TFDIR_TXDIR: u32 = 0x60;
const RFEN0: u32 = 0x80;
const RFEN1: u32 = 0x100;
const RXEXT: u32 = 0x400;

/// The FCSR fields that are not FIFO counts (RM 35.3.13).
const FCSR_WATERMARKS: u32 = 0x00FF_00FF;

/// Words per FIFO.
const DEPTH: usize = 15;

/// How an SSI is wired and clocked: board values.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SsiConfig {
    /// The DMA request line of transmit FIFO 0 (RM Table 19-6: SSI0 TX0 on 50, SSI1 TX0 on 54).
    pub tx_dma: u8,
    /// The DMA request line of receive FIFO 0 (RM Table 19-6: SSI0 RX0 on 48, SSI1 RX0 on 52).
    pub rx_dma: u8,
    /// The ticks one frame takes: the board's sample rate in ticks. At least 1.
    pub ticks_per_frame: u64,
}

/// What an [`Ssi`] has counted since it was made.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct SsiCounters {
    /// Transmit underruns (ISR\[TUE0\]): frames that found fewer words in transmit FIFO 0 than slots.
    pub underruns: u64,
    /// Those of the underruns that happened while the eDMA served the transmit request (its ERQ bit set): the
    /// stream not keeping up, as against clocks started ahead of it.
    pub dma_underruns: u64,
    /// Words written to TX0 while transmit FIFO 0 was full, and dropped.
    pub overruns: u64,
    /// Received frames that did not fit in receive FIFO 0, and were dropped (ISR\[ROE0\]).
    pub rx_overruns: u64,
    /// Frames since the SSI was made.
    pub frames: u64,
    /// Words written into a starved transmitter (one that has underrun since it was started): the FIFO has no slot
    /// tags, so such a word goes out in whichever slot comes next (RM 35.4.1), and the stream may be a slot out.
    pub unaligned_refills: u64,
}

/// One SSI (RM ch. 35) in synchronous mode generating its clocks, as an I2S master or in network mode, with FIFO
/// 0 each way, each fed by the eDMA.
///
/// While SSI_EN and TE are set, a frame ends every [`SsiConfig::ticks_per_frame`] ticks (the first one frame after
/// TE is set; the clock dividers in CCR are not used). A frame has CCR\[DC\] + 1 slots of 2 × (CCR\[WL\] + 1)-bit
/// words. Each frame pops a word per slot from transmit FIFO 0 and records the words sent, each its low word-length
/// bits sign-extended, with the tick ([`Ssi::capture`]): for 24-bit words bits 23:0 of a FIFO word are sent
/// (measured on an MCF54415, with TCR\[TXBIT0\] clear). A frame that finds fewer words than slots underruns: it
/// sends silence, sets ISR\[TUE0\] and empties the FIFO (the odd word goes with the frame). guess: this differs
/// from the RM, which re-transmits the previous data on an underrun (RM Table 35-8); not measured. While RE is set
/// too, the frame then pushes the words of the received line ([`Ssi::set_source`]: one per slot, cut or padded with
/// 0) into receive FIFO 0, each masked to the word length and sign-extended to 32 bits when RCR\[RXEXT\] is set; a
/// frame that does not fit is dropped, setting ISR\[ROE0\].
///
/// Clearing TE with SSI_EN kept stops the frames at once and sets ISR\[TFRC\] at the end of the frame in progress
/// (RM Table 35-8); setting TE again starts a frame period afresh and clears TFRC. Clearing SSI_EN resets the status
/// and empties both FIFOs (RM 35.3.7). guess: words written while SSI_EN is clear are kept (the RM says the FIFOs
/// are cleared when the SSI is disabled, not that writes are ignored).
///
/// DMA requests are levels (RM Table 35-9): the transmit request (line [`SsiConfig::tx_dma`]) is IER\[TDMAE\] with
/// at least FCSR\[TFWM0\] free words in transmit FIFO 0, the receive request (line [`SsiConfig::rx_dma`])
/// IER\[RDMAE\] with at least FCSR\[RFWM0\] words in receive FIFO 0; a watermark of 0 requests nothing. They are
/// driven after every register access and every frame. When several frames are due at once they run in one go,
/// without the eDMA in between.
///
/// Registers (32-bit; another access size is refused with [`Stop::Unmodelled`]). Reset values are the RM's (RM
/// Table 35-4; measured equal on an MCF54415):
/// - TX0 (+0x00 write): appends a word to transmit FIFO 0, or drops it when the FIFO is full.
/// - RX0 (+0x08 read): pops receive FIFO 0 (0 when empty).
/// - CR, IER, TCR, RCR, CCR (+0x10, +0x18 - +0x24) are stored; FCSR (+0x2C) stores its watermarks and reads TFCNT0
///   and RFCNT0 (the counts of FIFO 1 read 0).
/// - ISR (+0x14): TFE0 (at least TFWM0 free words), TDE0 (a free word), RFF0 (at least RFWM0 words; never with
///   RFWM0 0), RDR0 (a word), TUE0 and ROE0 (latched; write 1 to clear), TFRC; TFE1 and TDE1 are always set, FIFO 1
///   being always empty. The other bits read 0.
/// - ACR - ACCDIS (+0x38 - +0x58) read 0 and take 0.
///
/// Everything else is refused with [`Stop::Unmodelled`], the detail naming the SSI and the feature: reads of TX0,
/// writes of RX0, TX1 and RX1 (the second FIFO pair), nonzero writes of the AC97 registers and of the time slot
/// masks, and the rest of the window. A write of CR, IER, TCR, RCR, CCR or FCSR that leaves SSI_EN and TE set is
/// checked, and refused (not stored) as "configuration" when the result leaves the modelled subset:
/// - CR without SYN, or with a bit other than SYN, NET, I2S, SSI_EN, TE and RE; CR\[I2S\] neither I2S master nor
///   normal with NET (I2S slave and the clocks from outside are not modelled)
/// - TCR without TFEN0, with TFEN1, or without TFDIR and TXDIR; with RE, RCR without RFEN0 or with RFEN1
/// - IER bits other than TDMAE and RDMAE (the SSI's own interrupt request is not modelled; the reset IER, 0x3003,
///   enables some, so a driver writes IER before setting TE)
/// - FCSR\[TFWM0\] 0
pub struct Ssi {
    config: SsiConfig,
    cr: u32,
    ier: u32,
    tcr: u32,
    rcr: u32,
    ccr: u32,
    fcsr: u32,
    tx: VecDeque<u32>,
    rx: VecDeque<u32>,
    tue: bool,
    roe: bool,
    /// The end of the frame in progress, while the transmitter runs.
    next: Option<u64>,
    /// Underrun since the transmitter was started: the slot phase is lost.
    starved: bool,
    /// TE cleared: the frame end that sets TFRC.
    tfrc_at: Option<u64>,
    capture: Vec<(u64, Vec<i32>)>,
    record: bool,
    counters: SsiCounters,
    source: Option<Box<dyn FnMut(u64) -> Vec<i32>>>,
}

impl fmt::Debug for Ssi {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Ssi")
            .field("config", &self.config)
            .field("cr", &self.cr)
            .field("tx", &self.tx.len())
            .field("rx", &self.rx.len())
            .field("next", &self.next)
            .field("capture", &self.capture.len())
            .field("counters", &self.counters)
            .finish_non_exhaustive()
    }
}

fn running(cr: u32) -> bool {
    cr & (SSI_EN | TE) == SSI_EN | TE
}

impl Ssi {
    /// An SSI at reset, wired and clocked as `config` says. Panics when `config.ticks_per_frame` is 0.
    pub fn new(config: SsiConfig) -> Ssi {
        assert!(
            config.ticks_per_frame > 0,
            "SSI ticks_per_frame must be at least 1"
        );
        Ssi {
            config,
            cr: 0,
            ier: 0x3003,
            tcr: 0x200,
            rcr: 0x200,
            ccr: 0x0004_0000,
            fcsr: 0x0081_0081,
            tx: VecDeque::new(),
            rx: VecDeque::new(),
            tue: false,
            roe: false,
            next: None,
            starved: false,
            tfrc_at: None,
            capture: Vec::new(),
            record: true,
            counters: SsiCounters::default(),
            source: None,
        }
    }

    /// The received line: `f(n)` gives the words of frame `n` (counted from 0 over all frames, as
    /// [`SsiCounters::frames`]), one per slot. Without a source the line is silent.
    pub fn set_source(&mut self, f: Box<dyn FnMut(u64) -> Vec<i32>>) {
        self.source = Some(f);
    }

    /// The frames sent: (tick of the frame's end, the signed word of each slot).
    pub fn capture(&self) -> &[(u64, Vec<i32>)] {
        &self.capture
    }

    /// Every frame captured so far, emptying the log.
    pub fn take_capture(&mut self) -> Vec<(u64, Vec<i32>)> {
        std::mem::take(&mut self.capture)
    }

    /// Whether frames are captured (default on). Off changes only the log, never the transmitter.
    pub fn record(&mut self, on: bool) {
        self.record = on;
    }

    /// The counts of underruns, overruns, frames and unaligned refills.
    pub fn counters(&self) -> SsiCounters {
        self.counters
    }

    fn slots(&self) -> usize {
        (self.ccr >> 8 & 0x1F) as usize + 1
    }

    fn bits(&self) -> u32 {
        2 * ((self.ccr >> 13 & 0xF) + 1)
    }

    /// `w`'s low word-length bits, sign-extended.
    fn signed(&self, w: u32) -> i32 {
        let bits = self.bits();
        let w = i64::from(w) & ((1i64 << bits) - 1);
        let v = if w >> (bits - 1) & 1 != 0 {
            w - (1i64 << bits)
        } else {
            w
        };
        v as i32
    }

    fn tfwm0(&self) -> usize {
        (self.fcsr & 0xF) as usize
    }

    fn rfwm0(&self) -> usize {
        (self.fcsr >> 4 & 0xF) as usize
    }

    fn free(&self) -> usize {
        DEPTH - self.tx.len()
    }

    fn isr(&self, now: u64) -> u32 {
        let mut v = TFE1 | TDE1;
        if self.free() >= self.tfwm0() {
            v |= TFE0;
        }
        if self.free() > 0 {
            v |= TDE0;
        }
        if self.rfwm0() > 0 && self.rx.len() >= self.rfwm0() {
            v |= RFF0;
        }
        if !self.rx.is_empty() {
            v |= RDR0;
        }
        if self.tue {
            v |= TUE0;
        }
        if self.roe {
            v |= ROE0;
        }
        if self.tfrc_at.is_some_and(|t| now >= t) {
            v |= TFRC;
        }
        v
    }

    fn fcsr(&self) -> u32 {
        self.fcsr & FCSR_WATERMARKS | (self.tx.len() as u32) << 8 | (self.rx.len() as u32) << 12
    }

    fn drive(&self, cx: &mut DeviceCx) {
        let tx = self.ier & TDMAE != 0 && self.tfwm0() > 0 && self.free() >= self.tfwm0();
        let rx = self.ier & RDMAE != 0 && self.rfwm0() > 0 && self.rx.len() >= self.rfwm0();
        cx.dma.set(self.config.tx_dma, tx);
        cx.dma.set(self.config.rx_dma, rx);
    }

    /// What the configuration `regs` (CR, IER, TCR, RCR, FCSR) has outside the modelled subset.
    fn problems(cr: u32, ier: u32, tcr: u32, rcr: u32, fcsr: u32) -> Vec<String> {
        let mut p = Vec::new();
        if cr & SYN == 0 || cr & !(SYN | NET | I2S | SSI_EN | TE | RE) != 0 {
            p.push(format!("CR {cr:#x}"));
        }
        let i2s = cr >> 5 & 0b11;
        if !(i2s == 0b01 || i2s == 0 && cr & NET != 0) {
            p.push(format!("CR {cr:#x} (only I2S master or network mode)"));
        }
        if tcr & TFEN0 == 0 || tcr & TFEN1 != 0 || tcr & TFDIR_TXDIR != TFDIR_TXDIR {
            p.push(format!("TCR {tcr:#x}"));
        }
        if cr & RE != 0 && (rcr & RFEN0 == 0 || rcr & RFEN1 != 0) {
            p.push(format!("RCR {rcr:#x}"));
        }
        if ier & !(TDMAE | RDMAE) != 0 {
            p.push(format!("IER {ier:#x}"));
        }
        if fcsr & 0xF == 0 {
            p.push(format!("FCSR {fcsr:#x}"));
        }
        p
    }

    /// A write of CR, IER, TCR, RCR, CCR or FCSR.
    fn configure(&mut self, offset: u32, value: u32, cx: &mut DeviceCx) {
        let (mut cr, mut ier, mut tcr, mut rcr, mut fcsr) =
            (self.cr, self.ier, self.tcr, self.rcr, self.fcsr);
        match offset {
            CR => cr = value,
            IER => ier = value,
            TCR => tcr = value,
            RCR => rcr = value,
            FCSR => fcsr = value & FCSR_WATERMARKS,
            _ => {}
        }
        if running(cr) {
            let p = Ssi::problems(cr, ier, tcr, rcr, fcsr);
            if !p.is_empty() {
                cx.refuse(
                    Stop::Unmodelled,
                    format!("SSI configuration {} is not modelled", p.join(", ")),
                );
                return;
            }
        }
        let was = self.cr;
        if offset == CCR {
            self.ccr = value;
        }
        (self.cr, self.ier, self.tcr, self.rcr, self.fcsr) = (cr, ier, tcr, rcr, fcsr);
        if running(cr) {
            if !running(was) {
                self.next = Some(cx.now + self.config.ticks_per_frame);
                self.starved = false;
                self.tfrc_at = None;
            }
        } else if offset == CR {
            if running(was) && cr & SSI_EN != 0 {
                // the transmitter stops at the end of this frame
                self.tfrc_at = self.next;
            } else if cr & SSI_EN == 0 {
                self.tfrc_at = None;
            }
            self.next = None;
            self.starved = false;
        }
        if was & SSI_EN != 0 && cr & SSI_EN == 0 {
            // SSI reset (RM 35.3.7): the status as at reset, both FIFOs empty
            self.tx.clear();
            self.rx.clear();
            self.tue = false;
            self.roe = false;
        }
    }

    fn send(&mut self, w: u32) {
        if self.starved && self.next.is_some() {
            self.counters.unaligned_refills += 1;
            self.starved = false;
        }
        if self.tx.len() >= DEPTH {
            self.counters.overruns += 1;
        } else {
            self.tx.push_back(w);
        }
    }

    /// The frame ending at `t`.
    fn frame(&mut self, t: u64, cx: &mut DeviceCx) {
        let n = self.slots();
        if self.tx.len() >= n {
            let words = self.tx.drain(..n).collect::<Vec<_>>();
            let words = words.into_iter().map(|w| self.signed(w)).collect();
            if self.record {
                self.capture.push((t, words));
            }
        } else {
            // guess: silence and an emptied FIFO; RM Table 35-8 says the previous data is re-transmitted on an
            // underrun. Not measured.
            self.tue = true;
            self.starved = true;
            self.counters.underruns += 1;
            if cx.dma.enabled() & 1 << self.config.tx_dma != 0 {
                self.counters.dma_underruns += 1;
            }
            self.tx.clear();
            if self.record {
                self.capture.push((t, vec![0; n]));
            }
        }
        if self.cr & RE != 0 {
            let frame = self.counters.frames;
            let mut words = self.source.as_mut().map_or_else(Vec::new, |f| f(frame));
            words.resize(n, 0);
            if self.rx.len() + n > DEPTH {
                self.counters.rx_overruns += 1;
                self.roe = true;
            } else {
                let mask = ((1u64 << self.bits()) - 1) as u32;
                for w in words {
                    let w = w as u32 & mask;
                    let w = if self.rcr & RXEXT != 0 {
                        self.signed(w) as u32
                    } else {
                        w
                    };
                    self.rx.push_back(w);
                }
            }
        }
        self.counters.frames += 1;
    }

    /// The name and feature of a register in ACR - ACCDIS.
    fn ac97(offset: u32) -> (&'static str, &'static str) {
        match offset {
            TMASK => ("TMASK", "time slot masks are"),
            RMASK => ("RMASK", "time slot masks are"),
            _ => {
                const NAMES: [&str; 9] = [
                    "ACR", "ACADD", "ACDAT", "ATAG", "TMASK", "RMASK", "ACCSR", "ACCEN", "ACCDIS",
                ];
                (NAMES[((offset - ACR) / 4) as usize], "AC97 mode is")
            }
        }
    }
}

impl Device for Ssi {
    fn read(&mut self, offset: u32, size: Size, cx: &mut DeviceCx) -> u32 {
        if size != Size::Long {
            cx.refuse(
                Stop::Unmodelled,
                format!("SSI {size:?} read at +{offset:#x} is not modelled (32-bit registers)"),
            );
            return 0;
        }
        match offset {
            RX0 => {
                let v = self.rx.pop_front().unwrap_or(0);
                self.drive(cx);
                v
            }
            CR => self.cr,
            ISR => self.isr(cx.now),
            IER => self.ier,
            TCR => self.tcr,
            RCR => self.rcr,
            CCR => self.ccr,
            FCSR => self.fcsr(),
            ACR..=ACCDIS if offset % 4 == 0 => 0,
            _ => {
                let what = match offset {
                    TX0 => "TX0 read".to_string(),
                    TX1 => "TX1 (the second FIFO pair)".to_string(),
                    RX1 => "RX1 (the second FIFO pair)".to_string(),
                    _ => format!("read +{offset:#x}"),
                };
                cx.refuse(Stop::Unmodelled, format!("SSI {what} is not modelled"));
                0
            }
        }
    }

    fn write(&mut self, offset: u32, size: Size, value: u32, cx: &mut DeviceCx) {
        if size != Size::Long {
            cx.refuse(
                Stop::Unmodelled,
                format!("SSI {size:?} write at +{offset:#x} is not modelled (32-bit registers)"),
            );
            return;
        }
        match offset {
            TX0 => self.send(value),
            ISR => {
                if value & TUE0 != 0 {
                    self.tue = false;
                }
                if value & ROE0 != 0 {
                    self.roe = false;
                }
            }
            CR | IER | TCR | RCR | CCR | FCSR => self.configure(offset, value, cx),
            ACR..=ACCDIS if offset % 4 == 0 && value == 0 => {}
            ACR..=ACCDIS if offset % 4 == 0 => {
                let (name, what) = Ssi::ac97(offset);
                cx.refuse(
                    Stop::Unmodelled,
                    format!("SSI {name} = {value:#x}: {what} not modelled"),
                );
            }
            _ => {
                let what = match offset {
                    RX0 => "RX0 write".to_string(),
                    TX1 => "TX1 (the second FIFO pair)".to_string(),
                    RX1 => "RX1 (the second FIFO pair)".to_string(),
                    _ => format!("write +{offset:#x} = {value:#x}"),
                };
                cx.refuse(Stop::Unmodelled, format!("SSI {what} is not modelled"));
            }
        }
        self.drive(cx);
    }

    fn next_event(&self) -> Option<u64> {
        self.next
    }

    fn advance(&mut self, now: u64, cx: &mut DeviceCx) {
        while let Some(t) = self.next
            && t <= now
        {
            self.next = Some(t + self.config.ticks_per_frame);
            self.frame(t, cx);
        }
        self.drive(cx);
    }

    fn poll_counted(&self) -> bool {
        false
    }

    /// No sources: the SSI's own interrupt request is not modelled, and the eDMA lists the routes of the requests
    /// it serves.
    fn wake_sources(&self, _out: &mut Vec<(usize, u8)>) -> bool {
        true
    }
}
