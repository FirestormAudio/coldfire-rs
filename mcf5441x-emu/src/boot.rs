//! Starting a program from reset as the chip's boot hardware does, per boot mode: the serial boot facility (RM ch.
//! 11), FlexBus CS0 (RM ch. 20) and the NAND flash controller (RM ch. 22).
//!
//! [`boot`] does what the hardware has done by the time the core fetches its reset vector, then loads SP and PC from
//! the first two longwords the core sees: it is a host helper, and the hardware's own steps take no time. Where the
//! board's SRAM sits and what the PLL runs at are the board's: the SRAM is RAM the board maps
//! ([`Memory::map_ram`](crate::Memory::map_ram)) and passes as `sram_base`, the PLL values come from
//! [`ChipOptions`](crate::soc::ChipOptions).
//!
//! [`sbf_image`], [`nand_pages`] and [`nand_boot_view`] lay out a program image the way each medium holds it.

use std::fmt;

use crate::cpu::Cpu;
use crate::machine::Machine;
use crate::mem::Size;
use crate::parts::SpiNor;
use crate::soc::{Chip, Dspi, NandArray, Nfc, StubBlock};

/// The serial boot facility reads the flash on DSPI0's PCS1 (the SBF's pins, RM 11.2 and Table 15-1).
const SBF_DSPI: usize = 0;
const SBF_PCS: u8 = 1;
/// SBFSR and SBFCR in the chip configuration module (RM Table 11-2).
const SBFSR: u32 = 0x20;
const SBFCR: u32 = 0x22;
/// The SPI memory's header: {0000, BLDIV}, BLL (2 bytes), RCON (4 bytes), little-endian (RM Table 11-5).
const SBF_HEADER: usize = 7;
/// The BLDIV [`sbf_image`] writes: divisor 2 (RM Table 11-4). The model has no shift clock, so any value boots.
const SBF_BLDIV: u8 = 1;
/// FlexBus CS0's flash as [`boot`] maps it: the image, rounded up to this with erased bytes.
const NOR_ROUND: usize = 0x1_0000;
/// The NAND boot blocks' first rows (RM 22.4.4): four pages are read from the first block whose reads all pass ECC.
const NAND_BOOT_BLOCKS: [u32; 4] = [0, 256, 512, 768];
const NAND_BOOT_PAGES: usize = 4;
/// The NFC's boot view at address 0 runs to 0xF8F (RM 22.4.4); the RAM [`boot`] maps there.
const NAND_VIEW_RAM: u32 = 0x1000;
/// The boot view's interleave unit (RM Figure 22-25).
const NAND_CHUNK: usize = 8;
/// NFC_CFG and its BTMD bit (RM Table 22-17): set when the NFC booted (RM 22.4.4).
const NFC_CFG: u32 = 0xFC0F_FF30;
const NFC_BTMD: u32 = 1 << 6;

/// What the chip boots from, with the medium's contents.
pub enum BootSource {
    /// Serial boot: the serial boot facility reads the SPI flash on DSPI0 PCS1 (header, BLL, RCON, RM Table 11-5),
    /// copies the boot image into the SRAM at `sram_base`, and the flash stays on DSPI0 PCS1.
    Serial { flash: SpiNor, sram_base: u32 },
    /// NOR boot: FlexBus CS0, the global chip select, decodes the flash at address 0 (RM 20.4.1.3).
    Nor { image: Vec<u8> },
    /// NAND boot: the NFC's boot view of the first four pages at address 0 (RM 22.4.4); the array goes behind the
    /// NFC.
    Nand { array: NandArray },
}

/// Why [`boot`] could not start the program.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BootError {
    /// Serial: no byte of the flash has bits 7:4 clear, so the serial boot facility never synchronises (RM 11.4.1).
    NoSbfHeader,
    /// Serial: the header's BLL is 0, so nothing is loaded and the reset vector comes from FlexBus or the NFC
    /// (RM 11.4.3), which a serial boot here does not model.
    NoBootLoad,
    /// The boot image does not fit in RAM at this address (the SRAM is not mapped there, or is too small).
    Unmapped { addr: u32 },
    /// The image is shorter than its reset vector (SP and PC, 8 bytes).
    ShortImage,
    /// NAND: the chip was built without a NAND array, so it has no NFC model ([`ChipOptions::nand`](crate::soc::ChipOptions::nand)).
    NoNfc,
    /// NAND: a page of every boot block fails ECC, so the NFC boot is unsuccessful (RM 22.4.4).
    NandBootFailed,
}

impl fmt::Display for BootError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            BootError::NoSbfHeader => write!(
                f,
                "no serial boot header in the flash (no byte with bits 7:4 clear)"
            ),
            BootError::NoBootLoad => write!(f, "the serial boot header loads nothing (BLL 0)"),
            BootError::Unmapped { addr } => {
                write!(f, "the boot image does not fit in RAM at {addr:#010x}")
            }
            BootError::ShortImage => write!(f, "the boot image is shorter than its reset vector"),
            BootError::NoNfc => write!(f, "NAND boot on a chip built without a NAND array"),
            BootError::NandBootFailed => write!(f, "every NAND boot block fails ECC"),
        }
    }
}

impl std::error::Error for BootError {}

fn long(b: &[u8], at: usize) -> Result<u32, BootError> {
    let s = b.get(at..at + 4).ok_or(BootError::ShortImage)?;
    Ok(u32::from_be_bytes(s.try_into().unwrap()))
}

/// Boot `m` from `source` as the chip leaves reset: put the core in its reset state, do what the boot hardware does
/// before the reset vector fetch, then load SP and PC from the first two longwords of the boot image. Returns the
/// entry PC; a [`Machine::run`] then starts there.
///
/// - [`BootSource::Serial`]: the serial boot facility (RM 11.4) finds the header (the first byte with bits 7:4
///   clear, RM 11.4.1; guess: the 1-to-0 transition it also needs is taken as given), reads BLL and RCON, and
///   copies 4 × (BLL + 1) bytes from just after the header into the SRAM at `sram_base` (bytes past the flash's
///   end read 0xFF). SBFSR reads BLL and SBFCR the header's BLDIV with FR clear (RM 11.3; set in the CCM stub).
///   The flash goes on DSPI0 PCS1, in place of any part there. RCON drives nothing: what it configures (the PLL's
///   values among them) is the board's [`ChipOptions`](crate::soc::ChipOptions). RM 11.4.3 has the SBF map the
///   SRAM "to address 0 via the RAMBAR", which RAMBAR's base field cannot hold; the SRAM is wherever the board
///   mapped it, RAMBAR is left at its reset value, and the reset vector is read from the SRAM's first bytes.
/// - [`BootSource::Nor`]: the image is mapped as RAM at address 0, rounded up to 64 KB with erased bytes (0xFF)
///   (guess: CS0's flash takes the CPU's writes like RAM; a flash would take them as commands).
/// - [`BootSource::Nand`]: of the boot blocks at rows 0, 256, 512 and 768, the first whose four pages all pass ECC
///   is read (RM 22.4.4); its [`nand_boot_view`] is mapped as RAM at 0..0x1000, the bytes past the view 0xFF.
///   NFC_CFG\[BTMD\] is set (RM Table 22-17) and the array goes behind the NFC.
///
/// # Panics
/// When the RAM mapped at address 0 (NOR, NAND) overlaps RAM or a device `m` already has there.
pub fn boot(m: &mut Machine, chip: &Chip, source: BootSource) -> Result<u32, BootError> {
    m.cpu = Cpu::new();
    let vector = match source {
        BootSource::Serial { flash, sram_base } => {
            let code = sbf_load(m, chip, flash.memory())?;
            m.mem
                .load(sram_base, &code)
                .map_err(|_| BootError::Unmapped { addr: sram_base })?;
            m.device_mut::<Dspi>(chip.dspi[SBF_DSPI])
                .attach(SBF_PCS, Box::new(flash));
            code
        }
        BootSource::Nor { image } => {
            if image.len() < 8 {
                return Err(BootError::ShortImage);
            }
            let mut flash = image;
            flash.resize(flash.len().div_ceil(NOR_ROUND) * NOR_ROUND, 0xFF);
            m.mem.map_ram(0, flash.len() as u32);
            m.mem
                .load(0, &flash)
                .map_err(|_| BootError::Unmapped { addr: 0 })?;
            flash
        }
        BootSource::Nand { array } => {
            let nfc = chip.nfc.ok_or(BootError::NoNfc)?;
            let first = NAND_BOOT_BLOCKS
                .into_iter()
                .find(|&b| (b..b + NAND_BOOT_PAGES as u32).all(|row| !array.bad.contains(&row)))
                .ok_or(BootError::NandBootFailed)?;
            let pages: Vec<Vec<u8>> = (first..first + NAND_BOOT_PAGES as u32)
                .map(|row| {
                    let mut p = array.pages.get(row as usize).cloned().unwrap_or_default();
                    p.resize(array.payload, 0xFF);
                    p
                })
                .collect();
            let mut view = nand_boot_view(&pages);
            view.resize(NAND_VIEW_RAM as usize, 0xFF);
            m.mem.map_ram(0, NAND_VIEW_RAM);
            m.mem
                .load(0, &view)
                .map_err(|_| BootError::Unmapped { addr: 0 })?;
            m.device_mut::<Nfc>(nfc).set_array(array);
            let cfg = m.access(NFC_CFG, Size::Long, None).expect("NFC_CFG reads");
            m.access(NFC_CFG, Size::Long, Some(cfg | NFC_BTMD))
                .expect("NFC_CFG takes a write");
            view
        }
    };
    let (sp, pc) = (long(&vector, 0)?, long(&vector, 4)?);
    m.cpu.a[7] = sp;
    m.cpu.pc = pc;
    Ok(pc)
}

/// The serial boot facility's read of `flash`: the boot image it copies into the SRAM. Sets SBFSR and SBFCR.
fn sbf_load(m: &mut Machine, chip: &Chip, flash: &[u8]) -> Result<Vec<u8>, BootError> {
    let h = flash
        .iter()
        .position(|&b| b >> 4 == 0)
        .ok_or(BootError::NoSbfHeader)?;
    let header = flash.get(h..h + SBF_HEADER).ok_or(BootError::NoSbfHeader)?;
    let bldiv = u32::from(header[0] & 0xF);
    let bll = u16::from_le_bytes([header[1], header[2]]);
    if let Some(&(_, ccm)) = chip.stubs.iter().find(|(name, _)| *name == "CCM") {
        let ccm = m.device_mut::<StubBlock>(ccm);
        ccm.set(SBFSR, u32::from(bll));
        ccm.set(SBFCR, bldiv);
    }
    if bll == 0 {
        return Err(BootError::NoBootLoad);
    }
    let start = h + SBF_HEADER;
    let len = 4 * (usize::from(bll) + 1);
    let mut code: Vec<u8> = flash.iter().skip(start).take(len).copied().collect();
    code.resize(len, 0xFF);
    Ok(code)
}

/// The SPI flash contents the serial boot facility boots `image` from (RM Table 11-5): {0000, BLDIV} with BLDIV 1
/// (divisor 2, RM Table 11-4), BLL and `rcon` little-endian, then `image` from flash byte 7. BLL makes the SBF
/// copy the first `sbf_len` bytes of `image`, rounded up to whole longwords: BLL = ⌈`sbf_len` / 4⌉ - 1. An
/// `sbf_len` of 0 gives BLL 0: nothing is copied (RM Table 11-3).
///
/// # Panics
/// When `sbf_len` is 1 to 4 (BLL 0 means no load, so the SBF cannot copy a single longword) or above 0x40000 (BLL
/// is 16 bits, RM 11.4.2).
pub fn sbf_image(image: &[u8], sbf_len: usize, rcon: u32) -> Vec<u8> {
    assert!(
        sbf_len == 0 || (5..=0x4_0000).contains(&sbf_len),
        "sbf_image: the SBF copies 2 to 65536 longwords (BLL 1 to 0xFFFF) or none, not {sbf_len} bytes"
    );
    let bll = sbf_len.div_ceil(4).saturating_sub(1) as u16;
    let mut out = Vec::with_capacity(SBF_HEADER + image.len());
    out.push(SBF_BLDIV);
    out.extend_from_slice(&bll.to_le_bytes());
    out.extend_from_slice(&rcon.to_le_bytes());
    out.extend_from_slice(image);
    out
}

/// What the CPU reads from address 0 in NFC boot mode, given the four boot pages' payloads (RM 22.4.4 and Figure
/// 22-25): 8-byte units taken from each page in turn, page 0's first, then page 1's, page 2's, page 3's, then each
/// page's second. The view holds the whole 8-byte units of the shortest page: 3968 bytes for 996-byte payloads.
/// guess: the RM's view runs to 0xF8F (3984 bytes, 996 per page) and leaves the last half unit of each page open,
/// so the view stops before them.
///
/// # Panics
/// When `pages` is not four pages.
pub fn nand_boot_view(pages: &[Vec<u8>]) -> Vec<u8> {
    assert_eq!(
        pages.len(),
        NAND_BOOT_PAGES,
        "nand_boot_view: the NFC boots from four pages"
    );
    let units = pages.iter().map(Vec::len).min().unwrap_or(0) / NAND_CHUNK;
    let mut view = Vec::with_capacity(NAND_BOOT_PAGES * units * NAND_CHUNK);
    for unit in 0..units {
        for page in pages {
            view.extend_from_slice(&page[unit * NAND_CHUNK..][..NAND_CHUNK]);
        }
    }
    view
}

/// The four boot pages' payloads (`payload` bytes each) whose [`nand_boot_view`] starts with `view`: the inverse
/// of the interleave. Bytes `view` does not reach are erased (0xFF).
///
/// # Panics
/// When `view` is longer than four pages' whole 8-byte units (3968 bytes for a 996-byte payload).
pub fn nand_pages(view: &[u8], payload: usize) -> Vec<Vec<u8>> {
    let units = payload / NAND_CHUNK;
    let room = NAND_BOOT_PAGES * units * NAND_CHUNK;
    assert!(
        view.len() <= room,
        "nand_pages: a boot view of {} bytes does not fit four {payload}-byte pages ({room} bytes)",
        view.len()
    );
    let mut pages = vec![vec![0xFF; payload]; NAND_BOOT_PAGES];
    for (i, unit) in view.chunks(NAND_CHUNK).enumerate() {
        let at = i / NAND_BOOT_PAGES * NAND_CHUNK;
        pages[i % NAND_BOOT_PAGES][at..at + unit.len()].copy_from_slice(unit);
    }
    pages
}
