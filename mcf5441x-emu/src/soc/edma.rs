//! The enhanced DMA controller, eDMA (RM ch. 19): 64 channels, the chip's first bus master.

use crate::device::{Device, DeviceCx};
use crate::mem::Size;
use crate::stop::Stop;

/// The TCDs: 64 of 32 bytes at +0x1000 (RM 19.4.17).
const TCD0: u32 = 0x1000;
const TCD_END: u32 = TCD0 + 64 * 32;

const ES: u32 = 0x04;
const EEIH: u32 = 0x10;
const EEIL: u32 = 0x14;
const SERQ: u32 = 0x18;
const CERQ: u32 = 0x19;
const SEEI: u32 = 0x1A;
const CEEI: u32 = 0x1B;
const CINT: u32 = 0x1C;
const CERR: u32 = 0x1D;
const SSRT: u32 = 0x1E;
const CDNE: u32 = 0x1F;
const ERRH: u32 = 0x28;
const ERRL: u32 = 0x2C;

/// The command registers' NOP bit and their "all channels" bit (RM 19.4.5 - RM 19.4.12).
const NOP: u32 = 0x80;
const ALL: u32 = 0x40;

/// ES (RM Table 19-4).
const VLD: u32 = 1 << 31;
const SAE: u32 = 0x80;
const DAE: u32 = 0x20;

/// TCD field offsets (RM 19.4.17).
const SADDR: usize = 0x00;
const ATTR: usize = 0x04;
const SOFF: usize = 0x06;
const NBYTES: usize = 0x08;
const SLAST: usize = 0x0C;
const DADDR: usize = 0x10;
const CITER: usize = 0x14;
const DOFF: usize = 0x16;
const DLAST: usize = 0x18;
const BITER: usize = 0x1C;
const CSR: usize = 0x1E;

/// TCD CSR bits (RM Table 19-31).
const START: u16 = 0x01;
const INTMAJOR: u16 = 0x02;
const INTHALF: u16 = 0x04;
const DREQ: u16 = 0x08;
const ESG: u16 = 0x10;
const MAJORELINK: u16 = 0x20;
const DONE: u16 = 0x80;
/// CITER and BITER E_LINK.
const ELINK: u16 = 0x8000;

/// Minor loops one asserted request may run before the model gives up on it ever dropping (a backstop: a minor
/// loop that touches no device cannot lower the request, and stops the serving at once).
const MAX_LOOPS: u32 = 1 << 20;

/// What one activation of a channel did.
enum Activation {
    /// A minor loop ran; whether any of its accesses reached a device.
    Moved { device: bool },
    /// A configuration error was recorded; nothing moved.
    Error,
}

/// The eDMA, mapped at 0xFC04_4000 with a 0x2000 window as a bus master
/// ([`Machine::add_bus_master`](crate::Machine::add_bus_master)). Transfers are instant: a minor loop runs whole
/// in the call that activates it and takes no ticks.
///
/// A channel is activated by its peripheral request (DMA request line `n`, [`DmaLines`](crate::DmaLines)) while
/// its ERQ bit is set, or by software through TCD CSR\[START\] or SSRT. Each activation runs one minor loop
/// (RM 19.6.4): `NBYTES / size` reads at SADDR and writes at DADDR through the bus, each followed by
/// `SADDR += SOFF` and `DADDR += DOFF` within their SMOD/DMOD rings (RM 19.6.4.3); then CITER counts down, an
/// INT_HALF interrupt is raised when CITER reaches BITER / 2, and when it reaches 0 CITER reloads from BITER,
/// SLAST and DLAST are added, DONE is set, an INT_MAJOR interrupt is raised and D_REQ clears the ERQ bit. An
/// activation clears DONE and START.
///
/// A peripheral request is served until its line drops (each minor loop reads or writes the peripheral, which
/// lowers its line when it has nothing more), at SERQ and at every instruction boundary
/// ([`Device::service`]); channels are served from 63 down to 0, the order the reset group and channel priorities
/// give (RM 19.4.1, RM 19.4.16). A peripheral may hold any number of minor loops' worth behind one request. A
/// minor loop whose accesses all hit RAM cannot lower the request: if the request is still asserted after one,
/// the serving is refused (at SERQ) or halts the run with [`Stop::Unmodelled`] "channel N request never drops".
/// As a backstop, a request still asserted after 2^20 minor loops is refused or halts the same way.
///
/// A source or destination address that is not a multiple of the transfer size is a configuration error (RM
/// 19.4.2): nothing moves, CITER and the TCD are unchanged but for START, DONE is not set, the channel's ERR bit is
/// set and ES records the channel with SAE or DAE. ES\[VLD\] is the OR of the ERR bits (RM Table 19-4), so CERR
/// clears it and ES keeps the channel and SAE/DAE (measured on an MCF54415). A request left asserted after an error
/// is not served again until the next boundary.
///
/// Interrupt routes (RM Table 17-15, Table 17-16, Table 17-17): channels 0-15 drive INTC0 sources 8-23, channels
/// 16-55 INTC1 sources 8-47, and channels 56-63 share INTC2 source 0. The interrupt of a channel holds until CINT.
///
/// Registers: ES, ERRH and ERRL (read, 32-bit); EEIH and EEIL read 0 and take 0; the byte commands SERQ, CERQ,
/// CINT, CERR, SSRT and CDNE for one channel, CEEI, and any command with NOP set (ignored); the TCDs (any size,
/// raw). Everything this model leaves out is refused with [`Stop::Unmodelled`], the detail naming the eDMA and the
/// feature:
/// - EDMA_CR (arbitration, minor loop mapping, cancel, halt), ERQH/ERQL, INTH/INTL, the hardware request status
///   registers, DCHPRIn, writes to ES and ERRH/ERRL, reads of the commands and commands at other sizes
/// - the "all channels" commands (SAER, CAER, CAIR, CAEI, SAST, CADN)
/// - error interrupts: SEEI, or a nonzero write to EEIH/EEIL (INTC0 source 24 is not driven)
/// - writing TCD CSR E_SG or MAJOR_E_LINK (scatter/gather, channel linking; with DONE set they are forced to 0,
///   RM Table 19-31)
///
/// and, when a channel is activated (refused when the activation comes from a register write, a halt when it
/// comes from a request served at the boundary):
/// - transfer sizes other than 8-bit or 32-bit on both sides, NBYTES not a multiple of the size, or NBYTES 0
/// - minor-loop channel linking (CITER or BITER E_LINK), CITER 0, SOFF or DOFF not a multiple of the size (the
///   NCE, SOE and DOE errors)
/// - a bus error on a read or write (SBE, DBE); the earlier units of that minor loop are already written, and
///   the TCD is left as before it
///
/// BWC is ignored: it only spaces the accesses out in time.
#[derive(Debug)]
pub struct Edma {
    tcd: Vec<u8>,
    /// ERQH:ERQL.
    erq: u64,
    /// INTH:INTL: channels with an interrupt not cleared.
    int: u64,
    /// ERRH:ERRL.
    err: u64,
    /// ES without VLD: ERRCHN and the error bits of the last error.
    es: u32,
}

impl Default for Edma {
    fn default() -> Edma {
        Edma::new()
    }
}

/// The interrupt (controller, source) of channel `ch` (RM Table 17-15, Table 17-16, Table 17-17).
fn route(ch: usize) -> (usize, u8) {
    match ch {
        0..16 => (0, 8 + ch as u8),
        16..56 => (1, ch as u8 - 8),
        _ => (2, 0),
    }
}

/// `addr + off`, with the bits above the low `modulo` held (RM 19.6.4.3); a plain add when `modulo` is 0.
fn mod_add(addr: u32, off: i32, modulo: u32) -> u32 {
    let sum = addr.wrapping_add(off as u32);
    if modulo == 0 {
        return sum;
    }
    let mask = ((1u64 << modulo) - 1) as u32;
    (addr & !mask) | (sum & mask)
}

impl Edma {
    /// An eDMA in its reset state: no request enabled, no interrupt or error, TCDs zero (the RM leaves most of the
    /// TCD undefined at reset).
    pub fn new() -> Edma {
        Edma {
            tcd: vec![0; 64 * 32],
            erq: 0,
            int: 0,
            err: 0,
            es: 0,
        }
    }

    fn field(&self, ch: usize, off: usize, n: usize) -> u32 {
        let at = ch * 32 + off;
        self.tcd[at..at + n]
            .iter()
            .fold(0, |v, &b| v << 8 | u32::from(b))
    }

    fn set_field(&mut self, ch: usize, off: usize, n: usize, v: u32) {
        let at = ch * 32 + off;
        for i in 0..n {
            self.tcd[at + i] = (v >> (8 * (n - 1 - i))) as u8;
        }
    }

    fn csr(&self, ch: usize) -> u16 {
        self.field(ch, CSR, 2) as u16
    }

    fn set_csr(&mut self, ch: usize, csr: u16) {
        self.set_field(ch, CSR, 2, u32::from(csr));
    }

    /// Drive the interrupt source of `ch`'s route: asserted while any channel on it has its INT bit set.
    fn drive(&self, ch: usize, cx: &mut DeviceCx) {
        let r = route(ch);
        let on = (0..64).any(|c| self.int & 1 << c != 0 && route(c) == r);
        cx.irq.set(r.0, r.1, on);
    }

    fn raise(&mut self, ch: usize, cx: &mut DeviceCx) {
        self.int |= 1 << ch;
        let (c, s) = route(ch);
        cx.irq.set(c, s, true);
    }

    /// Why channel `ch`'s TCD cannot be run by this model, if it cannot.
    fn unmodelled(&self, ch: usize) -> Option<String> {
        let attr = self.field(ch, ATTR, 2);
        let nbytes = self.field(ch, NBYTES, 4);
        let (ssize, dsize) = (attr >> 8 & 7, attr & 7);
        if nbytes == 0 {
            return Some(format!(
                "eDMA channel {ch} NBYTES 0 (a 4 GiB minor loop, RM 19.4.17) is not modelled"
            ));
        }
        if ssize != dsize || !(ssize == 0 || ssize == 2) || !nbytes.is_multiple_of(1 << ssize) {
            return Some(format!(
                "eDMA channel {ch} transfer size (ATTR {attr:#06x}, NBYTES {nbytes}): only 8-bit or 32-bit on \
                 both sides is modelled"
            ));
        }
        let csr = self.csr(ch);
        if csr & (ESG | MAJORELINK) != 0 {
            return Some(format!(
                "eDMA channel {ch} CSR {csr:#06x}: scatter/gather and major-loop channel linking are not modelled"
            ));
        }
        let (citer, biter) = (self.field(ch, CITER, 2), self.field(ch, BITER, 2));
        if (citer | biter) & u32::from(ELINK) != 0 {
            return Some(format!("eDMA channel {ch} minor-loop channel link"));
        }
        if citer == 0 {
            return Some(format!(
                "eDMA channel {ch} CITER 0 (an NBYTES/CITER configuration error, not modelled)"
            ));
        }
        let size = 1 << ssize;
        let (soff, doff) = (self.field(ch, SOFF, 2), self.field(ch, DOFF, 2));
        if !soff.is_multiple_of(size) || !doff.is_multiple_of(size) {
            return Some(format!(
                "eDMA channel {ch} offset not a multiple of the transfer size (SOFF {soff:#06x}, DOFF \
                 {doff:#06x}; offset configuration errors are not modelled)"
            ));
        }
        None
    }

    /// Activate channel `ch` once: one minor loop (RM 19.6.4), or a configuration error. `Err` is the detail of a
    /// feature this model leaves out.
    fn activate(&mut self, ch: usize, cx: &mut DeviceCx) -> Result<Activation, String> {
        if let Some(why) = self.unmodelled(ch) {
            return Err(why);
        }
        let attr = self.field(ch, ATTR, 2);
        let ssize = attr >> 8 & 7;
        let size = if ssize == 2 { Size::Long } else { Size::Byte };
        let bytes = size.bytes();
        let mut csr = self.csr(ch) & !START;
        let mut saddr = self.field(ch, SADDR, 4);
        let mut daddr = self.field(ch, DADDR, 4);
        if !saddr.is_multiple_of(bytes) || !daddr.is_multiple_of(bytes) {
            // Measured on an MCF54415: nothing moves, START clears, DONE is not set, CITER is unchanged.
            // guess: an earlier DONE is kept (the older model). RM 19.6.4.1 step 3 clears DONE as the channel
            // activates; whether that comes before the configuration check is not measured.
            self.es = (ch as u32) << 8
                | if !saddr.is_multiple_of(bytes) {
                    SAE
                } else {
                    DAE
                };
            self.err |= 1 << ch;
            self.set_csr(ch, csr);
            return Ok(Activation::Error);
        }
        csr &= !DONE;
        let (smod, dmod) = (attr >> 11, attr >> 3 & 0x1F);
        let soff = i32::from(self.field(ch, SOFF, 2) as u16 as i16);
        let doff = i32::from(self.field(ch, DOFF, 2) as u16 as i16);
        let mut bus = cx.bus().expect("the eDMA is a bus master");
        let mut device = false;
        for _ in 0..self.field(ch, NBYTES, 4) / bytes {
            device |= !bus.is_ram(saddr, size) || !bus.is_ram(daddr, size);
            let v = bus.read(saddr, size).map_err(|f| {
                format!("eDMA channel {ch} bus error reading {saddr:#010x} ({f:?}): not modelled")
            })?;
            bus.write(daddr, size, v).map_err(|f| {
                format!("eDMA channel {ch} bus error writing {daddr:#010x} ({f:?}): not modelled")
            })?;
            saddr = mod_add(saddr, soff, smod);
            daddr = mod_add(daddr, doff, dmod);
        }
        let biter = self.field(ch, BITER, 2);
        let mut citer = (self.field(ch, CITER, 2) - 1) & 0x7FFF;
        if csr & INTHALF != 0 && citer == biter >> 1 {
            self.raise(ch, cx);
        }
        if citer == 0 {
            citer = biter;
            saddr = saddr.wrapping_add(self.field(ch, SLAST, 4));
            daddr = daddr.wrapping_add(self.field(ch, DLAST, 4));
            csr |= DONE;
            if csr & INTMAJOR != 0 {
                self.raise(ch, cx);
            }
            if csr & DREQ != 0 {
                self.erq &= !(1 << ch);
            }
        }
        self.set_field(ch, SADDR, 4, saddr);
        self.set_field(ch, DADDR, 4, daddr);
        self.set_field(ch, CITER, 2, citer);
        self.set_csr(ch, csr);
        Ok(Activation::Moved { device })
    }

    /// Serve channel `ch`'s peripheral request: minor loops while it is enabled and its line is asserted.
    fn serve(&mut self, ch: usize, cx: &mut DeviceCx) -> Result<(), String> {
        let mut loops = 0;
        let asserted = |cx: &DeviceCx| cx.dma.asserted() & 1 << ch != 0;
        while self.erq & 1 << ch != 0 && asserted(cx) {
            if loops == MAX_LOOPS {
                return Err(format!(
                    "eDMA channel {ch} served {MAX_LOOPS} minor loops without the request dropping"
                ));
            }
            match self.activate(ch, cx)? {
                Activation::Moved { device } => {
                    loops += 1;
                    if !device && self.erq & 1 << ch != 0 && asserted(cx) {
                        return Err(format!(
                            "eDMA channel {ch} request never drops: a minor loop that reached no device left \
                             it asserted"
                        ));
                    }
                }
                Activation::Error => break,
            }
        }
        Ok(())
    }

    /// A write into the TCDs.
    fn write_tcd(&mut self, offset: u32, size: Size, value: u32, cx: &mut DeviceCx) {
        let at = (offset - TCD0) as usize;
        let n = size.bytes() as usize;
        let (first, last) = (at / 32, (at + n - 1) / 32);
        let old = self.tcd[at..at + n].to_vec();
        let done: Vec<bool> = (first..=last).map(|ch| self.csr(ch) & DONE != 0).collect();
        for i in 0..n {
            self.tcd[at + i] = (value >> (8 * (n - 1 - i))) as u8;
        }
        let mut started = Vec::new();
        for (ch, done) in (first..=last).zip(done) {
            let mut csr = self.csr(ch);
            if done {
                // RM Table 19-31: E_SG and MAJOR_E_LINK are forced to zero when written while DONE is set.
                csr &= !(ESG | MAJORELINK);
                self.set_csr(ch, csr);
            }
            if csr & (ESG | MAJORELINK) != 0 {
                self.tcd[at..at + n].copy_from_slice(&old);
                cx.refuse(
                    Stop::Unmodelled,
                    format!(
                        "eDMA channel {ch} CSR {csr:#06x}: scatter/gather and major-loop channel linking are not \
                         modelled"
                    ),
                );
                return;
            }
            if csr & START != 0 {
                started.push(ch);
            }
        }
        for ch in started {
            if let Err(why) = self.activate(ch, cx) {
                // A write covers at most one CSR: no other channel has run.
                self.tcd[at..at + n].copy_from_slice(&old);
                cx.refuse(Stop::Unmodelled, why);
                return;
            }
        }
    }

    /// A byte command (SERQ ... CDNE) for channel `value & 0x3F`.
    fn command(&mut self, offset: u32, value: u32, cx: &mut DeviceCx) {
        if value & NOP != 0 {
            return;
        }
        let ch = (value & 0x3F) as usize;
        if offset == CEEI {
            return; // no error interrupt is ever enabled
        }
        if offset == SEEI {
            cx.refuse(
                Stop::Unmodelled,
                format!(
                    "eDMA SEEI {value:#04x}: error interrupts (INTC0 source 24) are not modelled"
                ),
            );
            return;
        }
        if value & ALL != 0 {
            cx.refuse(
                Stop::Unmodelled,
                format!(
                    "eDMA command +{offset:#x} = {value:#04x}: \"all channels\" is not modelled"
                ),
            );
            return;
        }
        match offset {
            SERQ => {
                self.erq |= 1 << ch;
                if let Err(why) = self.serve(ch, cx) {
                    cx.refuse(Stop::Unmodelled, why);
                }
            }
            CERQ => self.erq &= !(1 << ch),
            CINT => {
                self.int &= !(1 << ch);
                self.drive(ch, cx);
            }
            CERR => self.err &= !(1 << ch),
            SSRT => {
                let csr = self.csr(ch);
                self.set_csr(ch, csr | START);
                if let Err(why) = self.activate(ch, cx) {
                    self.set_csr(ch, csr);
                    cx.refuse(Stop::Unmodelled, why);
                }
            }
            _ => self.set_csr(ch, self.csr(ch) & !DONE), // CDNE
        }
    }
}

impl Device for Edma {
    fn read(&mut self, offset: u32, size: Size, cx: &mut DeviceCx) -> u32 {
        if (TCD0..TCD_END).contains(&offset) && offset + size.bytes() <= TCD_END {
            let at = (offset - TCD0) as usize;
            return self.tcd[at..at + size.bytes() as usize]
                .iter()
                .fold(0, |v, &b| v << 8 | u32::from(b));
        }
        match (offset, size) {
            (ES, Size::Long) => self.es | if self.err != 0 { VLD } else { 0 },
            (ERRH, Size::Long) => (self.err >> 32) as u32,
            (ERRL, Size::Long) => self.err as u32,
            (EEIH | EEIL, Size::Long) => 0,
            _ => {
                cx.refuse(
                    Stop::Unmodelled,
                    format!("eDMA read +{offset:#x} ({size:?}) is not modelled"),
                );
                0
            }
        }
    }

    fn write(&mut self, offset: u32, size: Size, value: u32, cx: &mut DeviceCx) {
        if (TCD0..TCD_END).contains(&offset) && offset + size.bytes() <= TCD_END {
            self.write_tcd(offset, size, value, cx);
            return;
        }
        match (offset, size) {
            (SERQ..=CDNE, Size::Byte) => self.command(offset, value, cx),
            (EEIH | EEIL, Size::Long) if value == 0 => {}
            (EEIH | EEIL, Size::Long) => cx.refuse(
                Stop::Unmodelled,
                format!(
                    "eDMA EEI +{offset:#x} = {value:#010x}: error interrupts (INTC0 source 24) are not modelled"
                ),
            ),
            _ => cx.refuse(
                Stop::Unmodelled,
                format!("eDMA write +{offset:#x} ({size:?}) = {value:#x} is not modelled"),
            ),
        }
    }

    fn service(&mut self, cx: &mut DeviceCx) {
        if self.erq & cx.dma.asserted() == 0 {
            return;
        }
        for ch in (0..64).rev() {
            if self.erq & cx.dma.asserted() & 1 << ch != 0
                && let Err(why) = self.serve(ch, cx)
            {
                cx.halt(Stop::Unmodelled, why);
                return;
            }
        }
    }

    fn poll_counted(&self) -> bool {
        false
    }

    fn wake_sources(&self, out: &mut Vec<(usize, u8)>) -> bool {
        for ch in 0..64 {
            let r = route(ch);
            if self.erq & 1 << ch != 0
                && self.csr(ch) & (INTMAJOR | INTHALF) != 0
                && !out.contains(&r)
            {
                out.push(r);
            }
        }
        true
    }
}
