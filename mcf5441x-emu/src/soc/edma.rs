//! The enhanced DMA controller, eDMA (RM ch. 19): 64 channels, the chip's first bus master.

use crate::device::{Device, DeviceCx};
use crate::mem::Size;
use crate::stop::Stop;

/// The TCDs: 64 of 32 bytes at +0x1000 (RM 19.4.17).
const TCD0: u32 = 0x1000;
const TCD_END: u32 = TCD0 + 64 * 32;

const CR: u32 = 0x00;
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

/// EDMA_CR (RM Figure 19-3, Table 19-3): its reset value (GRP3PRI 3, GRP2PRI 2, GRP1PRI 1, GRP0PRI 0), the group
/// priority fields, the bits this model stores and the bits it leaves out.
const CR_RESET: u32 = 0xE400;
const CR_GRPPRI: u32 = 0xFF00;
const CR_EDBG: u32 = 0x02;
const CR_ERCA: u32 = 0x04;
const CR_ERGA: u32 = 0x08;
const CR_STORED: u32 = CR_GRPPRI | CR_ERGA | CR_ERCA | CR_EDBG;
const CR_UNMODELLED: [(u32, &str); 6] = [
    (1 << 17, "CX (cancel transfer)"),
    (1 << 16, "ECX (error cancel transfer)"),
    (0x80, "EMLM (minor loop mapping)"),
    (0x40, "CLM (continuous link mode)"),
    (0x20, "HALT"),
    (0x10, "HOE (halt on error)"),
];

/// The command registers' NOP bit and their "all channels" bit (RM 19.4.5 - RM 19.4.12).
const NOP: u32 = 0x80;
const ALL: u32 = 0x40;

/// ES (RM Table 19-4).
const VLD: u32 = 1 << 31;
const SAE: u32 = 0x80;
const DAE: u32 = 0x20;
const SGE: u32 = 0x04;
/// What ES records for a reserved transfer size, besides VLD and ERRCHN: SAE, SOE, DAE and DOE together (measured
/// on an MCF54415).
const SIZE_ERROR: u32 = 0xF0;

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

/// Minor loops one asserted request may run, or one start may set off through its links, before the model gives
/// up on it ever ending (a backstop: a minor loop that touches no device cannot lower the request, and stops the
/// serving at once).
const MAX_LOOPS: u32 = 1 << 20;

/// What one activation of a channel did.
enum Activation {
    /// A minor loop ran; whether any of its accesses reached a device.
    Moved { device: bool },
    /// A configuration error was recorded.
    Error,
}

/// The bytes of an ATTR SSIZE or DSIZE encoding (RM Table 19-21), unless it is reserved.
fn size_bytes(code: u32) -> Option<u32> {
    match code {
        0 => Some(1),
        1 => Some(2),
        2 => Some(4),
        4 => Some(16),
        // 011, 101, 110 and 111 are reserved. 101 is not a 32-byte size on this chip (measured on an MCF54415).
        _ => None,
    }
}

/// The bus access one transfer of `bytes` is made of: a byte, a word, or longwords (a 16-byte transfer moves as a
/// burst of 4 longwords).
fn unit(bytes: u32) -> Size {
    match bytes {
        1 => Size::Byte,
        2 => Size::Word,
        _ => Size::Long,
    }
}

/// The eDMA, mapped at 0xFC04_4000 with a 0x2000 window as a bus master
/// ([`Machine::add_bus_master`](crate::Machine::add_bus_master)). Transfers are instant: a minor loop runs whole
/// in the call that activates it and takes no ticks.
///
/// A channel is activated by its peripheral request (DMA request line `n`, [`DmaLines`](crate::DmaLines)) while
/// its ERQ bit is set, or by software through TCD CSR\[START\] or SSRT, or by a channel link. Each activation runs
/// one minor loop (RM 19.6.4): NBYTES in transfers of the source size (ATTR SSIZE: 8-bit, 16-bit, 32-bit or
/// 16-byte, RM Table 19-21) read at SADDR and of the destination size written at DADDR through the
/// bus, each followed by `SADDR += SOFF` or `DADDR += DOFF` within their SMOD/DMOD rings (RM 19.6.4.3). The two
/// sizes may differ: reads fill a buffer of the larger size, which the writes then empty (RM Example 19-1). A
/// 16-byte transfer is a burst of 4 longword accesses at consecutive addresses. Then CITER counts
/// down (9 bits wide with E_LINK, 15 without), an INT_HALF interrupt is raised when it reaches BITER / 2 (for
/// BITER 2 and up) and when it reaches 0 the major loop completes:
/// - CITER reloads from BITER, SLAST is added to SADDR, and DLAST to DADDR unless E_SG loads a TCD
/// - DONE is set, INT_MAJOR raises the interrupt and D_REQ clears the ERQ bit
/// - with MAJOR_E_LINK, the channel MAJOR_LINKCH gets START
/// - then, with E_SG (scatter/gather), the 32-byte TCD at DLAST_SGA is read through the bus into the channel's TCD
///   and DONE is set onto its CSR. The load overwrites a START the link gave the channel itself; a START in the
///   loaded CSR is honoured. That order and DONE are measured on an MCF54415.
///
/// With CITER E_LINK, every minor loop but the last gives the channel CITER LINKCH START (RM 19.6.6). An activation
/// clears DONE and START. A start set by a link or a loaded TCD runs within the same call, before the call
/// returns: the started channels run one minor loop per START, highest channel first, until none is left (guess:
/// the order the reset channel priorities give, RM 19.4.16, stands in for arbitration). A start that sets off
/// more than 2^20 minor loops, or a started channel this model cannot run, is refused or halts the run with
/// [`Stop::Unmodelled`], and START is cleared on the channels still queued.
///
/// A peripheral request is served until its line drops (each minor loop reads or writes the peripheral, which
/// lowers its line when it has nothing more), at SERQ and at every instruction boundary
/// ([`Device::service`]); channels are served from 63 down to 0, the order the reset group and channel priorities
/// give (RM 19.4.1, RM 19.4.16). The ERQ bits are published as [`DmaLines::enabled`](crate::DmaLines::enabled)
/// for the peripherals' host-side statistics. A peripheral may hold any number of minor loops' worth behind one
/// request. A minor loop (with what its links started) whose accesses all hit RAM cannot lower the request: if the
/// request is still asserted after one, the serving is refused (at SERQ) or halts the run with [`Stop::Unmodelled`]
/// "channel N request never drops". As a backstop, a request still asserted after 2^20 minor loops is refused or
/// halts the same way; the likely cause is a peripheral request that cannot drop: a DSPI TX-fill request on a
/// running queue (DSPI frames take no time), or a UART or DSPI whose clock gate is closed (a gated module keeps its
/// request lines as it left them and drops the writes that would lower them; guess: not measured).
///
/// A source or destination address that is not a multiple of its transfer size is a configuration error (RM
/// 19.4.2): nothing moves, CITER and the TCD are unchanged but for START, DONE is not set, the channel's ERR bit is
/// set and ES records the channel with SAE or DAE. ES\[VLD\] is the OR of the ERR bits (RM Table 19-4), so CERR
/// clears it and ES keeps the channel and SAE/DAE (measured on an MCF54415). A request left asserted after an error
/// is not served again until the next boundary. A reserved SSIZE or DSIZE encoding (011, 101, 110, 111; there
/// is no 32-byte size) is a configuration error the same way, with SAE, SOE, DAE and DOE all set in ES (measured
/// on an MCF54415 for 011 and 101 on both sides; guess: 110, 111 and a reserved size on one side alike).
///
/// A DLAST_SGA that is not a multiple of 32 when a major loop with E_SG completes is the SGE configuration error,
/// found after the completion (measured on an MCF54415): the minor loop has moved its data, CITER is reloaded,
/// SLAST is added to SADDR and DLAST_SGA to DADDR, DONE is set and E_SG kept; ES records the channel with SGE and
/// the ERR bit is set; no TCD is loaded. The INT_MAJOR interrupt, D_REQ and the major-loop link come before it
/// (guess: RM's order, not measured).
///
/// Interrupt routes (RM Table 17-15, Table 17-16, Table 17-17): channels 0-15 drive INTC0 sources 8-23, channels
/// 16-55 INTC1 sources 8-47, and channels 56-63 share INTC2 source 0. The interrupt of a channel holds until CINT.
///
/// EDMA_CR is stored and reads back (32-bit; reset 0x0000_E400). EDBG, ERCA and ERGA change nothing (guess:
/// transfers take no time, so debug stalls and the arbitration order inside one call do not show); its reserved
/// bits are dropped.
///
/// Registers: CR (32-bit); ES, ERRH and ERRL (read, 32-bit); EEIH and EEIL read 0 and take 0; the byte commands
/// SERQ, CERQ, CINT, CERR, SSRT and CDNE for one channel, CEEI, and any command with NOP set (ignored); the TCDs
/// (any size, raw). Everything this model leaves out is refused with [`Stop::Unmodelled`], the detail naming the
/// eDMA and the feature:
/// - CR\[CX\], CR\[ECX\], CR\[EMLM\], CR\[CLM\], CR\[HALT\], CR\[HOE\] and group priorities other than the reset
///   ones (the detail names the bit); ERQH/ERQL, INTH/INTL, the hardware request status registers, DCHPRIn,
///   writes to ES and ERRH/ERRL, reads of the commands and commands at other sizes
/// - the "all channels" commands (SAER, CAER, CAIR, CAEI, SAST, CADN)
/// - error interrupts: SEEI, or a nonzero write to EEIH/EEIL (INTC0 source 24 is not driven)
/// - SERQ on a channel whose request line nothing drives ([`Edma::with_driven`]; [`Edma::new`] takes every line
///   as driven)
///
/// and, when a channel is activated (refused when the activation comes from a register write, a halt when it
/// comes from a request served at the boundary):
/// - NBYTES not a multiple of both sizes, or NBYTES 0
/// - CITER E_LINK not equal to BITER E_LINK, CITER 0, SOFF or DOFF not a multiple of its size (the NCE, SOE and
///   DOE errors)
/// - a bus error on a read or write (SBE, DBE): the earlier units of that minor loop are already written, and
///   the TCD is left as before it
/// - a bus error on the scatter/gather load: by then the minor loop has moved its data and the completion is done
///   (SADDR, DADDR and CITER written back, DONE set, the interrupt raised, D_REQ applied); the major-loop link's
///   START is cleared again and no TCD is loaded. That is the path of a channel served at its peripheral's
///   request; a channel started by CSR or SSRT restores the bytes of the starting write instead
///
/// With DONE set, a TCD write forces E_SG and MAJOR_E_LINK to 0 (RM Table 19-31). BWC is ignored: it only spaces
/// the accesses out in time.
#[derive(Debug)]
pub struct Edma {
    tcd: Vec<u8>,
    /// EDMA_CR: the stored bits.
    cr: u32,
    /// ERQH:ERQL.
    erq: u64,
    /// INTH:INTL: channels with an interrupt not cleared.
    int: u64,
    /// ERRH:ERRL.
    err: u64,
    /// ES without VLD: ERRCHN and the error bits of the last error.
    es: u32,
    /// The channels whose request lines something drives: SERQ is refused on the others.
    driven: u64,
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
    /// TCD undefined at reset). Every channel's request line counts as driven (see [`Edma::with_driven`]).
    pub fn new() -> Edma {
        Edma {
            driven: u64::MAX,
            tcd: vec![0; 64 * 32],
            cr: CR_RESET,
            erq: 0,
            int: 0,
            err: 0,
            es: 0,
        }
    }

    /// An eDMA in its reset state whose peripheral requests are modelled only on the channels in `driven` (bit
    /// `n` is channel `n`): SERQ on any other channel is refused with [`Stop::Unmodelled`], since its request
    /// would never come. A software start (TCD CSR\[START\], SSRT) needs no request and works on every channel.
    pub fn with_driven(driven: u64) -> Edma {
        Edma {
            driven,
            ..Edma::new()
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

    /// Give channel `ch` START, as a link does, and queue it in `starts`.
    fn link(&mut self, ch: usize, starts: &mut u64) {
        self.set_csr(ch, self.csr(ch) | START);
        *starts |= 1 << ch;
    }

    /// Record a configuration error of channel `ch`.
    fn error(&mut self, ch: usize, bits: u32) {
        self.es = (ch as u32) << 8 | bits;
        self.err |= 1 << ch;
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
        let (Some(ssize), Some(dsize)) = (size_bytes(attr >> 8 & 7), size_bytes(attr & 7)) else {
            return None; // a configuration error, recorded by `activate` before this check
        };
        if nbytes == 0 {
            return Some(format!(
                "eDMA channel {ch} NBYTES 0 (a 4 GiB minor loop, RM 19.4.17) is not modelled"
            ));
        }
        if !nbytes.is_multiple_of(ssize.max(dsize)) {
            return Some(format!(
                "eDMA channel {ch} transfer size (ATTR {attr:#06x}, NBYTES {nbytes}): NBYTES not a multiple of \
                 the sizes (an NBYTES/CITER configuration error, not modelled)"
            ));
        }
        let (citer, biter) = (self.field(ch, CITER, 2), self.field(ch, BITER, 2));
        if (citer ^ biter) & u32::from(ELINK) != 0 {
            return Some(format!(
                "eDMA channel {ch} minor-loop channel link: CITER {citer:#06x} and BITER {biter:#06x} differ in \
                 E_LINK (an NBYTES/CITER configuration error, not modelled)"
            ));
        }
        if citer & count_mask(citer) == 0 {
            return Some(format!(
                "eDMA channel {ch} CITER 0 (an NBYTES/CITER configuration error, not modelled)"
            ));
        }
        let (soff, doff) = (self.field(ch, SOFF, 2), self.field(ch, DOFF, 2));
        if !soff.is_multiple_of(ssize) || !doff.is_multiple_of(dsize) {
            return Some(format!(
                "eDMA channel {ch} offset not a multiple of the transfer size (SOFF {soff:#06x}, DOFF \
                 {doff:#06x}; offset configuration errors are not modelled)"
            ));
        }
        None
    }

    /// Activate channel `ch` once: one minor loop (RM 19.6.4), or a configuration error. The channels its links
    /// or a loaded TCD start get START and are added to `starts`. `Err` is the detail of a feature this model
    /// leaves out.
    fn activate(
        &mut self,
        ch: usize,
        cx: &mut DeviceCx,
        starts: &mut u64,
    ) -> Result<Activation, String> {
        let attr = self.field(ch, ATTR, 2);
        let mut csr = self.csr(ch) & !START;
        let (Some(ssize), Some(dsize)) = (size_bytes(attr >> 8 & 7), size_bytes(attr & 7)) else {
            // Measured on an MCF54415 (011 and 101 on both sides): nothing moves, START clears, DONE is not set,
            // SADDR and DADDR are unchanged, ES has SAE, SOE, DAE and DOE. guess: 110, 111 and a reserved size on
            // one side only alike; an earlier DONE kept, as for a misaligned address.
            self.error(ch, SIZE_ERROR);
            self.set_csr(ch, csr);
            return Ok(Activation::Error);
        };
        if let Some(why) = self.unmodelled(ch) {
            return Err(why);
        }
        let mut saddr = self.field(ch, SADDR, 4);
        let mut daddr = self.field(ch, DADDR, 4);
        if !saddr.is_multiple_of(ssize) || !daddr.is_multiple_of(dsize) {
            // Measured on an MCF54415: nothing moves, START clears, DONE is not set, CITER is unchanged.
            // guess: an earlier DONE is kept (the older model). RM 19.6.4.1 step 3 clears DONE as the channel
            // activates; whether that comes before the configuration check is not measured.
            self.error(
                ch,
                if !saddr.is_multiple_of(ssize) {
                    SAE
                } else {
                    DAE
                },
            );
            self.set_csr(ch, csr);
            return Ok(Activation::Error);
        }
        csr &= !DONE;
        let (smod, dmod) = (attr >> 11, attr >> 3 & 0x1F);
        let soff = i32::from(self.field(ch, SOFF, 2) as u16 as i16);
        let doff = i32::from(self.field(ch, DOFF, 2) as u16 as i16);
        let (sunit, dunit) = (unit(ssize), unit(dsize));
        let chunk = ssize.max(dsize);
        let mut buf = [0u8; 16];
        let mut bus = cx.bus().expect("the eDMA is a bus master");
        let mut device = false;
        for _ in 0..self.field(ch, NBYTES, 4) / chunk {
            for at in (0..chunk).step_by(sunit.bytes() as usize) {
                let a = saddr.wrapping_add(at % ssize);
                device |= !bus.is_ram(a, sunit);
                let v = bus.read(a, sunit).map_err(|f| {
                    format!("eDMA channel {ch} bus error reading {a:#010x} ({f:?}): not modelled")
                })?;
                let n = sunit.bytes() as usize;
                buf[at as usize..at as usize + n].copy_from_slice(&v.to_be_bytes()[4 - n..]);
                if (at + sunit.bytes()).is_multiple_of(ssize) {
                    saddr = mod_add(saddr, soff, smod);
                }
            }
            for at in (0..chunk).step_by(dunit.bytes() as usize) {
                let a = daddr.wrapping_add(at % dsize);
                device |= !bus.is_ram(a, dunit);
                let n = dunit.bytes() as usize;
                let v = buf[at as usize..at as usize + n]
                    .iter()
                    .fold(0, |v, &b| v << 8 | u32::from(b));
                bus.write(a, dunit, v).map_err(|f| {
                    format!("eDMA channel {ch} bus error writing {a:#010x} ({f:?}): not modelled")
                })?;
                if (at + dunit.bytes()).is_multiple_of(dsize) {
                    daddr = mod_add(daddr, doff, dmod);
                }
            }
        }
        let biter = self.field(ch, BITER, 2);
        let citer = self.field(ch, CITER, 2);
        let mask = count_mask(citer);
        let count = (citer & mask) - 1;
        if csr & INTHALF != 0 && biter & mask >= 2 && count == (biter & mask) >> 1 {
            self.raise(ch, cx);
        }
        if count != 0 {
            self.set_field(ch, SADDR, 4, saddr);
            self.set_field(ch, DADDR, 4, daddr);
            self.set_field(ch, CITER, 2, citer & !mask | count);
            self.set_csr(ch, csr);
            if citer & u32::from(ELINK) != 0 {
                self.link((citer >> 9 & 0x3F) as usize, starts);
            }
            return Ok(Activation::Moved { device });
        }
        // The major loop is complete, in RM's order (measured on an MCF54415): the final adjustments and DONE,
        // the interrupt and D_REQ, the major-loop link, then scatter/gather. A minor-loop link gives way to the
        // major-loop link (RM 19.6.6).
        let dlast = self.field(ch, DLAST, 4);
        let load = csr & ESG != 0 && dlast.is_multiple_of(32);
        saddr = saddr.wrapping_add(self.field(ch, SLAST, 4));
        if !load {
            // Measured on an MCF54415: a misaligned DLAST_SGA is added to DADDR, as DLAST.
            daddr = daddr.wrapping_add(dlast);
        }
        self.set_field(ch, SADDR, 4, saddr);
        self.set_field(ch, DADDR, 4, daddr);
        self.set_field(ch, CITER, 2, biter);
        csr |= DONE;
        self.set_csr(ch, csr);
        if csr & INTMAJOR != 0 {
            self.raise(ch, cx);
        }
        if csr & DREQ != 0 {
            self.erq &= !(1 << ch);
            cx.dma.set_enabled(self.erq);
        }
        if csr & MAJORELINK != 0 {
            self.link((csr >> 8 & 0x3F) as usize, starts);
        }
        if csr & ESG == 0 {
            return Ok(Activation::Moved { device });
        }
        if !load {
            // Measured on an MCF54415: the SGE error comes after the completion (data moved, DONE set, E_SG kept,
            // CITER reloaded, DLAST_SGA added to DADDR) and nothing is loaded. guess: the INT_MAJOR interrupt,
            // D_REQ and the major-loop link still happen, as they come before scatter/gather in RM's order.
            self.error(ch, SGE);
            return Ok(Activation::Error);
        }
        let mut bus = cx.bus().expect("the eDMA is a bus master");
        let mut next = [0u8; 32];
        for (i, word) in next.chunks_mut(4).enumerate() {
            let a = dlast.wrapping_add(4 * i as u32);
            device |= !bus.is_ram(a, Size::Long);
            let v = bus.read(a, Size::Long).map_err(|f| {
                format!(
                    "eDMA channel {ch} bus error loading the scatter/gather TCD at {a:#010x} ({f:?}): not modelled"
                )
            })?;
            word.copy_from_slice(&v.to_be_bytes());
        }
        // Measured on an MCF54415: the loaded TCD replaces the whole TCD, so a START the major-loop link gave this
        // channel is overwritten, and DONE is set onto the loaded CSR. A START in the loaded CSR is honoured.
        self.tcd[ch * 32..ch * 32 + 32].copy_from_slice(&next);
        self.set_csr(ch, self.csr(ch) | DONE);
        if self.csr(ch) & START != 0 {
            *starts |= 1 << ch;
        }
        Ok(Activation::Moved { device })
    }

    /// Clear START on the channels in `starts`: queued starts a refused chain leaves unrun.
    fn abandon(&mut self, starts: u64) {
        for ch in (0..64).filter(|ch| starts & 1 << ch != 0) {
            self.set_csr(ch, self.csr(ch) & !START);
        }
    }

    /// Run the channels in `starts` and every channel they start in turn, one minor loop per START, highest
    /// channel first (guess: the order the reset channel priorities give, RM 19.4.16), until none is left. A
    /// queued channel whose START is gone (overwritten by a scatter/gather load) does not run. Returns whether any
    /// access reached a device. On an error, START is cleared on the failing channel and every one still queued.
    fn run_started(&mut self, mut starts: u64, cx: &mut DeviceCx) -> Result<bool, String> {
        let mut device = false;
        let mut loops = 0;
        while starts != 0 {
            let ch = 63 - starts.leading_zeros() as usize;
            starts &= !(1 << ch);
            if self.csr(ch) & START == 0 {
                continue;
            }
            if loops == MAX_LOOPS {
                self.abandon(starts | 1 << ch);
                return Err(format!(
                    "eDMA channel {ch}: linked starts ran {MAX_LOOPS} minor loops without ending"
                ));
            }
            loops += 1;
            match self.activate(ch, cx, &mut starts) {
                Ok(Activation::Moved { device: d }) => device |= d,
                Ok(Activation::Error) => {}
                Err(why) => {
                    self.abandon(starts | 1 << ch);
                    return Err(why);
                }
            }
        }
        Ok(device)
    }

    /// A software start of channel `ch` (START is set): one minor loop and what it starts. When the channel itself
    /// cannot be run, the TCD bytes at `restore.0` get back `restore.1` (what the starting access wrote over); when
    /// a channel it started cannot, the channels that ran keep their state.
    fn start(&mut self, ch: usize, restore: (usize, &[u8]), cx: &mut DeviceCx) {
        let mut starts = 0;
        if let Err(why) = self.activate(ch, cx, &mut starts) {
            let (at, old) = restore;
            self.tcd[at..at + old.len()].copy_from_slice(old);
            self.abandon(starts);
            cx.refuse(Stop::Unmodelled, why);
            return;
        }
        if let Err(why) = self.run_started(starts, cx) {
            cx.refuse(Stop::Unmodelled, why);
        }
    }

    /// Serve channel `ch`'s peripheral request: minor loops while it is enabled and its line is asserted.
    fn serve(&mut self, ch: usize, cx: &mut DeviceCx) -> Result<(), String> {
        let mut loops = 0;
        let asserted = |cx: &DeviceCx| cx.dma.asserted() & 1 << ch != 0;
        while self.erq & 1 << ch != 0 && asserted(cx) {
            if loops == MAX_LOOPS {
                return Err(format!(
                    "eDMA channel {ch} served {MAX_LOOPS} minor loops without the request dropping: a peripheral request \
                     that cannot drop (a DSPI TX-fill request on a running queue: DSPI frames take no time; \
                     a peripheral whose clock gate is closed)"
                ));
            }
            let mut starts = 0;
            let activation = self
                .activate(ch, cx, &mut starts)
                .inspect_err(|_| self.abandon(starts))?;
            let linked = self.run_started(starts, cx)?;
            match activation {
                Activation::Moved { device } => {
                    loops += 1;
                    if !device && !linked && self.erq & 1 << ch != 0 && asserted(cx) {
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
        let mut started = None;
        for (ch, done) in (first..=last).zip(done) {
            let mut csr = self.csr(ch);
            if done {
                // RM Table 19-31: E_SG and MAJOR_E_LINK are forced to zero when written while DONE is set.
                csr &= !(ESG | MAJORELINK);
                self.set_csr(ch, csr);
            }
            if csr & START != 0 {
                started = Some(ch);
            }
        }
        // A write covers at most one CSR.
        if let Some(ch) = started {
            self.start(ch, (at, &old), cx);
        }
    }

    /// A write to EDMA_CR.
    fn write_cr(&mut self, value: u32, cx: &mut DeviceCx) {
        if let Some((_, name)) = CR_UNMODELLED.iter().find(|(bit, _)| value & bit != 0) {
            cx.refuse(
                Stop::Unmodelled,
                format!("eDMA CR = {value:#010x}: {name} is not modelled"),
            );
            return;
        }
        if value & CR_GRPPRI != CR_RESET {
            cx.refuse(
                Stop::Unmodelled,
                format!(
                    "eDMA CR = {value:#010x}: group priorities (GRPnPRI) other than the reset ones are not \
                     modelled"
                ),
            );
            return;
        }
        self.cr = value & CR_STORED;
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
            SERQ if self.driven & 1 << ch == 0 => cx.refuse(
                Stop::Unmodelled,
                format!("eDMA: channel {ch}: its peripheral request is not modelled"),
            ),
            SERQ => {
                self.erq |= 1 << ch;
                cx.dma.set_enabled(self.erq);
                if let Err(why) = self.serve(ch, cx) {
                    cx.refuse(Stop::Unmodelled, why);
                }
            }
            CERQ => {
                self.erq &= !(1 << ch);
                cx.dma.set_enabled(self.erq);
            }
            CINT => {
                self.int &= !(1 << ch);
                self.drive(ch, cx);
            }
            CERR => self.err &= !(1 << ch),
            SSRT => {
                let at = ch * 32 + CSR;
                let old = [self.tcd[at], self.tcd[at + 1]];
                self.set_csr(ch, self.csr(ch) | START);
                self.start(ch, (at, &old), cx);
            }
            _ => self.set_csr(ch, self.csr(ch) & !DONE), // CDNE
        }
    }
}

/// The CITER/BITER bits that count: 9 with E_LINK, 15 without (RM Table 19-27).
fn count_mask(iter: u32) -> u32 {
    if iter & u32::from(ELINK) != 0 {
        0x1FF
    } else {
        0x7FFF
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
            (CR, Size::Long) => self.cr,
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
            (CR, Size::Long) => self.write_cr(value, cx),
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
