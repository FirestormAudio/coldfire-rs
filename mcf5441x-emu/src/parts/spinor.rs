//! A serial NOR flash: the subset of the Spansion S25FL-S command set a boot loader and a flash writer use
//! (S25FL-S datasheet).

use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

use crate::soc::dspi::SpiDevice;

/// Status register 1 (S25FL-S datasheet).
const WIP: u8 = 0x01;
const WEL: u8 = 0x02;
const E_ERR: u8 = 0x20;
const P_ERR: u8 = 0x40;

const RDID: u8 = 0x9F;
const RDSR1: u8 = 0x05;
const WREN: u8 = 0x06;
const WRDI: u8 = 0x04;
const CLSR: u8 = 0x30;
const READ: u8 = 0x03;
const PP: u8 = 0x02;
const PP4: u8 = 0x12;
const SE: u8 = 0xD8;
const SE4: u8 = 0xDC;
const DYBRD: u8 = 0xE0;
const DYBWR: u8 = 0xE1;
const PPBRD: u8 = 0xE2;

/// The address bytes of a modelled opcode (`Some(0)` for none); `None` for an opcode not modelled.
fn address_bytes(op: u8) -> Option<u32> {
    match op {
        RDID | RDSR1 | WREN | WRDI | CLSR => Some(0),
        READ | PP | SE => Some(3),
        PP4 | SE4 | DYBRD | DYBWR | PPBRD => Some(4),
        _ => None,
    }
}

/// Faults a test injects into a [`SpiNor`], and the knobs of its timing.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NorFaults {
    /// Page bases whose every program fails with P_ERR, writing nothing. Entries are page bases (multiples of
    /// [`SpiNor::page_size`]): any other address never matches.
    pub fail_program: BTreeSet<u32>,
    /// Page bases whose next program fails with P_ERR, writing nothing; the entry is then removed. Page bases only,
    /// as for [`NorFaults::fail_program`].
    pub fail_program_once: BTreeSet<u32>,
    /// Bytes with bits stuck at 0: address → mask of the bits that still work. Applied by programs and erases.
    pub stuck: BTreeMap<u32, u8>,
    /// WIP never clears after a program or erase.
    pub hang: bool,
    /// The status reads that see WIP after a successful program or erase (default 2).
    pub busy_polls: u32,
    /// Sector base → DYB byte: 0x00 protects the sector, 0xFF (the default) leaves it unprotected. DYBWR sets it.
    pub dyb: BTreeMap<u32, u8>,
    /// Sector bases protected by their PPB (there is no PPB program command: only a test sets them).
    pub ppb: BTreeSet<u32>,
}

impl Default for NorFaults {
    fn default() -> NorFaults {
        NorFaults {
            fail_program: BTreeSet::new(),
            fail_program_once: BTreeSet::new(),
            stuck: BTreeMap::new(),
            hang: false,
            busy_polls: 2,
            dyb: BTreeMap::new(),
            ppb: BTreeSet::new(),
        }
    }
}

/// A serial NOR flash part with an S25FL-S command subset, as an [`SpiDevice`] on a DSPI chip select.
///
/// The array is `size` bytes (addresses wrap at it). The sector and program page sizes come from byte 4 of the
/// JEDEC ID, the sector architecture: 0x00 is uniform 256 KiB sectors with 512-byte pages, anything else 64 KiB
/// sectors with 256-byte pages (S25FL-S datasheet; the 4 KiB parameter sectors of those parts are not modelled).
///
/// Commands (opcode, address bytes): RDID 0x9F (the six ID bytes, then 0xFF), RDSR1 0x05 (status register 1:
/// WIP, WEL, E_ERR, P_ERR), WREN 0x06, WRDI 0x04, CLSR 0x30 (clears E_ERR, P_ERR and WIP), READ 0x03 (3; reads on
/// and on), PP 0x02 (3) and 4PP 0x12 (4) page programs, SE 0xD8 (3) and 4SE 0xDC (4) sector erases, DYBRD 0xE0
/// (4), DYBWR 0xE1 (4) and PPBRD 0xE2 (4). Opcode and address bytes shift back 0xFF. Any other opcode is reported
/// through [`SpiDevice::take_unmodelled`].
///
/// Programs, erases and DYBWR act when the select is negated, and only with WEL set; WEL clears after every such
/// command, whether it acted, failed or was ignored (guess: an S25FL-S keeps WEL after a failed program or erase
/// until CLSR or WRDI; unchecked on a part). A program ANDs its bytes into the page, wrapping within it; an
/// erase sets its sector to 0xFF. Either into a protected sector (its DYB 0x00 or its PPB set) or a program into a
/// page set to fail ([`NorFaults`]) changes nothing and sets E_ERR or P_ERR with WIP, which stay until CLSR.
/// After a successful program or erase, WIP reads set for [`NorFaults::busy_polls`] status reads (guess: a real
/// part times them; the count keeps the older model's timing, which code that polls RDSR cannot tell apart).
///
/// Logs: every command at its deselect ([`SpiNor::log`]) and every program, erase and DYBWR that acted
/// ([`SpiNor::writes`]).
pub struct SpiNor {
    mem: Vec<u8>,
    jedec: [u8; 6],
    sector: u32,
    page: u32,
    log: Vec<(u64, u8, Option<u32>)>,
    writes: Vec<(u64, u8, u32)>,
    wel: bool,
    /// E_ERR, P_ERR and WIP latched until CLSR.
    status: u8,
    /// Status reads left that see WIP.
    busy: u32,
    faults: NorFaults,
    /// The command in progress: its opcode, the bytes shifted so far, the address and the data.
    op: Option<u8>,
    n: u32,
    addr: u32,
    data: Vec<u8>,
    unmodelled: Option<String>,
}

impl fmt::Debug for SpiNor {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SpiNor")
            .field("size", &self.mem.len())
            .field("jedec", &self.jedec)
            .field("wel", &self.wel)
            .field("status", &self.status)
            .field("log", &self.log.len())
            .field("writes", &self.writes.len())
            .finish_non_exhaustive()
    }
}

impl SpiNor {
    /// A part of `size` bytes reporting `jedec`, holding `contents` from address 0 and erased (0xFF) beyond.
    /// Panics when `size` is not a whole number of sectors or `contents` does not fit.
    pub fn new(size: u32, jedec: [u8; 6], contents: &[u8]) -> SpiNor {
        let (sector, page) = if jedec[4] == 0x00 {
            (0x4_0000, 512)
        } else {
            (0x1_0000, 256)
        };
        assert!(
            size > 0 && size.is_multiple_of(sector),
            "{size:#x} bytes is not a whole number of {sector:#x}-byte sectors"
        );
        assert!(
            contents.len() <= size as usize,
            "{} bytes of contents in a {size:#x}-byte part",
            contents.len()
        );
        let mut mem = vec![0xFF; size as usize];
        mem[..contents.len()].copy_from_slice(contents);
        SpiNor {
            mem,
            jedec,
            sector,
            page,
            log: Vec::new(),
            writes: Vec::new(),
            wel: false,
            status: 0,
            busy: 0,
            faults: NorFaults::default(),
            op: None,
            n: 0,
            addr: 0,
            data: Vec::new(),
            unmodelled: None,
        }
    }

    /// The array.
    pub fn memory(&self) -> &[u8] {
        &self.mem
    }

    /// The sector size: 256 KiB or 64 KiB, from JEDEC ID byte 4.
    pub fn sector_size(&self) -> u32 {
        self.sector
    }

    /// The program page size: 512 or 256 bytes, from JEDEC ID byte 4.
    pub fn page_size(&self) -> u32 {
        self.page
    }

    /// Every command, at the tick its select was negated: (tick, opcode, address for the commands taking one).
    pub fn log(&self) -> &[(u64, u8, Option<u32>)] {
        &self.log
    }

    /// Every program, erase and DYBWR that acted: (tick, opcode, address).
    pub fn writes(&self) -> &[(u64, u8, u32)] {
        &self.writes
    }

    /// The injected faults and the WIP timing.
    pub fn faults(&mut self) -> &mut NorFaults {
        &mut self.faults
    }

    fn size(&self) -> u32 {
        self.mem.len() as u32
    }

    fn sector_base(&self, addr: u32) -> u32 {
        (addr % self.size()) & !(self.sector - 1)
    }

    fn protected(&self, addr: u32) -> bool {
        let base = self.sector_base(addr);
        self.faults.dyb.get(&base) == Some(&0x00) || self.faults.ppb.contains(&base)
    }

    fn end(&mut self, now: u64) {
        let Some(op) = self.op.take() else { return };
        let addr = (address_bytes(op).unwrap_or(0) > 0).then_some(self.addr);
        self.log.push((now, op, addr));
        match op {
            WREN => self.wel = true,
            WRDI => self.wel = false,
            CLSR => {
                self.status = 0;
                self.busy = 0;
            }
            PP | PP4 | SE | SE4 | DYBWR => {
                if self.wel {
                    self.writes.push((now, op, self.addr));
                    self.execute(op);
                }
                // guess: WEL clears after every attempt, failed or not
                self.wel = false;
            }
            _ => {}
        }
    }

    fn execute(&mut self, op: u8) {
        let addr = self.addr % self.size();
        if op == DYBWR {
            let v = self.data.first().copied().unwrap_or(0xFF);
            self.faults.dyb.insert(self.sector_base(addr), v);
            // guess: DYBWR leaves WIP clear, as in the older model (the S25FL-S datasheet facts at hand do not say
            // how long a DYB write keeps WIP set)
            return;
        }
        let erase = op == SE || op == SE4;
        if self.protected(addr) {
            self.status |= if erase { E_ERR } else { P_ERR } | WIP;
            return;
        }
        if erase {
            let base = self.sector_base(addr);
            let end = base + self.sector;
            self.mem[base as usize..end as usize].fill(0xFF);
            for (&a, &mask) in self.faults.stuck.range(base..end) {
                self.mem[a as usize] &= mask;
            }
        } else {
            // S25FL-S datasheet: the page is 512 bytes on the uniform 256 KiB sector parts (the older model always
            // wrapped at 256)
            let page = addr & !(self.page - 1);
            debug_assert!(
                self.faults
                    .fail_program
                    .iter()
                    .chain(&self.faults.fail_program_once)
                    .all(|p| p % self.page == 0),
                "NorFaults page entries must be multiples of the {}-byte page",
                self.page
            );
            if self.faults.fail_program.contains(&page)
                || self.faults.fail_program_once.remove(&page)
            {
                self.status |= P_ERR | WIP;
                return;
            }
            for (i, &v) in self.data.iter().enumerate() {
                let a = page | (addr.wrapping_add(i as u32) & (self.page - 1));
                let mask = self.faults.stuck.get(&a).copied().unwrap_or(0xFF);
                self.mem[a as usize] &= v & mask;
            }
        }
        self.busy = self.faults.busy_polls;
    }
}

impl SpiDevice for SpiNor {
    fn select(&mut self, _now: u64) {
        self.op = None;
        self.n = 0;
        self.addr = 0;
        self.data.clear();
    }

    fn transfer(&mut self, byte: u8) -> u8 {
        let n = self.n;
        self.n += 1;
        if n == 0 {
            if address_bytes(byte).is_none() {
                self.unmodelled = Some(format!("SPI NOR: opcode {byte:#04x} is not modelled"));
            }
            self.op = Some(byte);
            return 0xFF;
        }
        let Some(op) = self.op else { return 0xFF };
        let na = address_bytes(op).unwrap_or(0);
        if n <= na {
            self.addr = self.addr << 8 | u32::from(byte);
            return 0xFF;
        }
        let k = n - 1 - na;
        match op {
            RDID => self.jedec.get(k as usize).copied().unwrap_or(0xFF),
            RDSR1 => {
                let mut sr = self.status | if self.wel { WEL } else { 0 };
                if self.faults.hang || self.busy > 0 {
                    sr |= WIP;
                    self.busy = self.busy.saturating_sub(1);
                }
                sr
            }
            READ => self.mem[(self.addr.wrapping_add(k) % self.size()) as usize],
            DYBRD => {
                let base = self.sector_base(self.addr);
                self.faults.dyb.get(&base).copied().unwrap_or(0xFF)
            }
            PPBRD => {
                if self.faults.ppb.contains(&self.sector_base(self.addr)) {
                    0x00
                } else {
                    0xFF
                }
            }
            _ => {
                self.data.push(byte);
                0xFF
            }
        }
    }

    fn deselect(&mut self, now: u64) {
        self.end(now);
    }

    fn take_unmodelled(&mut self) -> Option<String> {
        self.unmodelled.take()
    }
}
