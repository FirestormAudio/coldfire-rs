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

/// One DSPI (RM ch. 40) in master mode, with [`SpiDevice`] parts on its chip selects
/// ([`Dspi::attach`]). A frame takes no time: it is shifted out, and its answer is in the RX FIFO, when it is
/// pushed. Every frame shifted out is recorded ([`Dspi::frames`]), so the DSPI also serves as a recorder for
/// write-only parts with nothing behind their select.
///
/// Registers (32-bit accesses only: a 1- or 2-byte access is refused with [`Stop::Unmodelled`], though the chip
/// takes 8- and 16-bit PUSHR writes and POPR reads):
/// - MCR (+0x00, reset 0x0000_4001: HALT and MDIS, RM Table 40-3): stored; CLR_TXF and CLR_RXF empty the FIFOs and
///   read as zero (RM Table 40-4). PCSISn sets the level PCSn idles at.
/// - TCR (+0x08): SPI_TCNT (bits 31:16) counts frames and wraps; a write presets it, and a frame with CTCNT clears
///   it before it is counted (RM 40.3.2).
/// - CTAR0-CTAR7 (+0x0C..+0x28, reset 0x7800_0000: 16-bit frames): stored. Only FMSZ (the frame size) and LSBFE
///   act; the baud rate and the delays do not, as frames take no time.
/// - SR (+0x2C): TCF, EOQF and RFOF are set by the frames and cleared by writing 1 (RM Table 40-7). TXRXS is set
///   while the DSPI runs: HALT and EOQF clear (RM 40.4.1). TFFF is set while the TX FIFO is not full and MDIS is
///   clear, RFDF while the RX FIFO is not empty (guess: both follow the FIFO, and writing 1 to them has no lasting
///   effect). TXCTR, TXNXTPTR, RXCTR and POPNXTPTR count and point into the FIFOs (guess: the 4-bit counters read
///   15 with 16 entries held, as the older model did; the pointers wrap at 16). TFUF (slave mode) is never set.
/// - RSER (+0x30, reset 0): DSPI interrupts and DMA requests are not modelled; a write of anything but 0 is
///   refused.
/// - PUSHR (+0x34): pushes a command and data into the TX FIFO (a push to a full FIFO is ignored, RM 40.4.2.4.1);
///   while the DSPI runs the FIFO drains at once, frame by frame. Reads return 0 (guess).
/// - POPR (+0x38): pops the RX FIFO (with MDIS set it reads the entry without popping it, RM 40.4.7.1).
/// - TXFR0-TXFR15 (+0x3C..+0x78) and RXFR0-RXFR15 (+0x7C..+0xB8): the FIFO entries, read-only.
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
/// read of an empty RX FIFO, a push while halted (MCR\[HALT\]), a push to a full TX FIFO (it
/// fills only while EOQF stops the DSPI; the chip ignores the push, RM 40.4.2.4.1), a frame pushed or going out
/// while disabled (MCR\[MDIS\]), in slave mode, with a DCONF other than SPI, with the modified transfer format
/// (MCR\[MTFE\]) or with a FIFO disabled, a frame through CTAR2-CTAR7 (PUSHR\[CTAS\] of 2 or more), a frame size other than 8 or 16 bits,
/// LSB first, a frame selecting more than one attached part, an RX overflow with MCR\[ROOE\] set, and whatever
/// an attached part reports with [`SpiDevice::take_unmodelled`]. A refused push changes nothing, unless a part
/// refused it: the part has then seen the frame.
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
    rx: [u32; FIFO],
    /// POPNXTPTR and RXCTR.
    pop_next: usize,
    rx_count: usize,
    parts: [Option<Box<dyn SpiDevice>>; SELECTS],
    /// The part selected now.
    selected: Option<usize>,
    frames: Vec<(u64, u32, u8, bool)>,
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
    /// A DSPI at reset (RM Table 40-3), with nothing on its chip selects.
    pub fn new() -> Dspi {
        Dspi {
            mcr: MDIS | HALT,
            tcnt: 0,
            ctar: [0x7800_0000; 8],
            flags: 0,
            tx: [0; FIFO],
            tx_next: 0,
            tx_count: 0,
            rx: [0; FIFO],
            pop_next: 0,
            rx_count: 0,
            parts: Default::default(),
            selected: None,
            frames: Vec::new(),
        }
    }

    /// Put `dev` on chip select PCS`pcs` (0-7; which of them a DSPI brings out is the board's business, RM Table
    /// 40-4), in place of any part there.
    pub fn attach(&mut self, pcs: u8, dev: Box<dyn SpiDevice>) {
        assert!(usize::from(pcs) < SELECTS, "no PCS{pcs}: PCS0-PCS7 only");
        self.parts[usize::from(pcs)] = Some(dev);
    }

    /// Every frame shifted out: (tick, TXDATA within the frame size, PUSHR\[PCS\], PUSHR\[CONT\]).
    pub fn frames(&self) -> &[(u64, u32, u8, bool)] {
        &self.frames
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

    /// The frame size of command `v`, and the part it reaches; or why it is refused.
    fn check(&self, v: u32) -> Result<(u32, Option<usize>), String> {
        let ctas = v >> 28 & 7;
        if ctas >= 2 {
            return Err(format!(
                "DSPI: PUSHR CTAS {ctas} (CTAR2-CTAR7) is not modelled"
            ));
        }
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
        if self.mcr & HALT != 0 {
            return cx.refuse(Stop::Unmodelled, "DSPI: PUSHR while halted (MCR HALT)");
        }
        if let Some(d) = self.mode() {
            return cx.refuse(Stop::Unmodelled, d);
        }
        if let Err(d) = self.check(v) {
            return cx.refuse(Stop::Unmodelled, d);
        }
        if self.tx_count == FIFO {
            // the chip ignores the push (RM 40.4.2.4.1): the frame would be lost without a trace. The FIFO only
            // fills while the DSPI is stopped, and a push while halted is refused above, so EOQF stopped it.
            return cx.refuse(
                Stop::Unmodelled,
                "DSPI: PUSHR with the TX FIFO full (stopped: EOQF set)",
            );
        }
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
        self.frames.push((cx.now, data, (v >> 16) as u8, cont));
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
                // guess: the written MDIS decides whether the clears act (RM 40.4.7.1: not while disabled)
                if value & MDIS == 0 {
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
            RSER if value != 0 => cx.refuse(
                Stop::Unmodelled,
                format!("DSPI: RSER {value:#010x}: interrupt and DMA requests are not modelled"),
            ),
            RSER => {}
            PUSHR => self.push(value, cx),
            _ => cx.refuse(
                Stop::Unmodelled,
                format!("DSPI: write to +{offset:#x} (read-only or reserved)"),
            ),
        }
    }

    fn poll_counted(&self) -> bool {
        false
    }

    /// None: the DSPI raises no interrupts and has no events.
    fn wake_sources(&self, _out: &mut Vec<(usize, u8)>) -> bool {
        true
    }
}
