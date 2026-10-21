//! Loading a linked program: [`load_elf`] reads a 32-bit big-endian m68k ELF executable into a [`Machine`].

use std::fmt;

use crate::machine::Machine;
use crate::mem::Fault;

const EM_68K: u16 = 4;
const PT_LOAD: u32 = 1;
const EHDR_LEN: usize = 52;
const PHDR_LEN: usize = 32;
const ZEROS: [u8; 4096] = [0; 4096];

/// Why [`load_elf`] refused a file.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ElfError {
    /// The file does not start with the ELF magic.
    NotElf,
    /// The file is not a 32-bit ELF (class 1).
    NotElf32,
    /// The file is not big-endian (data encoding 2).
    NotBigEndian,
    /// The file is for a machine other than the 68K family (`EM_68K`, 4); holds `e_machine`.
    WrongMachine(u16),
    /// A header, the program header table or a segment's file range lies outside the file.
    Truncated,
    /// A program header is malformed: the program header entries are too small, or a segment is longer in the
    /// file than in memory.
    BadHeader,
    /// A segment does not fit in mapped RAM; holds the first address that is not covered.
    Unmapped { addr: u32 },
}

impl fmt::Display for ElfError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ElfError::NotElf => write!(f, "not an ELF file"),
            ElfError::NotElf32 => write!(f, "not a 32-bit ELF file"),
            ElfError::NotBigEndian => write!(f, "not a big-endian ELF file"),
            ElfError::WrongMachine(m) => write!(f, "ELF machine {m} is not the 68K family"),
            ElfError::Truncated => write!(f, "ELF file is truncated"),
            ElfError::BadHeader => write!(f, "malformed ELF program header"),
            ElfError::Unmapped { addr } => write!(f, "segment does not fit in RAM at {addr:#010x}"),
        }
    }
}

impl std::error::Error for ElfError {}

fn u16_at(b: &[u8], off: usize) -> Result<u16, ElfError> {
    let s = b.get(off..off.checked_add(2).ok_or(ElfError::Truncated)?);
    Ok(u16::from_be_bytes(
        s.ok_or(ElfError::Truncated)?.try_into().unwrap(),
    ))
}

fn u32_at(b: &[u8], off: usize) -> Result<u32, ElfError> {
    let s = b.get(off..off.checked_add(4).ok_or(ElfError::Truncated)?);
    Ok(u32::from_be_bytes(
        s.ok_or(ElfError::Truncated)?.try_into().unwrap(),
    ))
}

fn unmapped(f: Fault, fallback: u32) -> ElfError {
    match f {
        Fault::Unmapped { addr, .. } => ElfError::Unmapped { addr },
        Fault::Device(_) => ElfError::Unmapped { addr: fallback },
    }
}

/// Load every `PT_LOAD` segment of the ELF executable `bytes` into `m`'s RAM and return its entry point.
///
/// Each segment with a non-zero memory size has its file bytes copied to its physical address (`p_paddr`) and
/// the rest of its memory size zeroed, so `.bss` reads as zero whatever the RAM held before, also when the
/// linker put it in a segment of its own with no file data. The file must be 32-bit,
/// big-endian and for `EM_68K`. The loads go through [`Memory::load`](crate::Memory::load), so decoded code
/// over the loaded range is dropped. The program counter is not touched.
///
/// Every offset is checked against `bytes`; a malformed file gives an [`ElfError`], never a panic. A segment
/// that does not fit in RAM fails with [`ElfError::Unmapped`]; segments before it stay loaded.
pub fn load_elf(m: &mut Machine, bytes: &[u8]) -> Result<u32, ElfError> {
    if bytes.get(..4) != Some(b"\x7fELF") {
        return Err(if bytes.len() < 4 {
            ElfError::Truncated
        } else {
            ElfError::NotElf
        });
    }
    if bytes.len() < EHDR_LEN {
        return Err(ElfError::Truncated);
    }
    if bytes[4] != 1 {
        return Err(ElfError::NotElf32);
    }
    if bytes[5] != 2 {
        return Err(ElfError::NotBigEndian);
    }
    let machine = u16_at(bytes, 18)?;
    if machine != EM_68K {
        return Err(ElfError::WrongMachine(machine));
    }
    let entry = u32_at(bytes, 24)?;
    let phoff = u32_at(bytes, 28)? as usize;
    let phentsize = u16_at(bytes, 42)? as usize;
    let phnum = u16_at(bytes, 44)? as usize;
    if phnum > 0 && phentsize < PHDR_LEN {
        return Err(ElfError::BadHeader);
    }
    for i in 0..phnum {
        let ph = i
            .checked_mul(phentsize)
            .and_then(|o| o.checked_add(phoff))
            .ok_or(ElfError::Truncated)?;
        if ph.checked_add(PHDR_LEN).is_none_or(|end| end > bytes.len()) {
            return Err(ElfError::Truncated);
        }
        if u32_at(bytes, ph)? != PT_LOAD {
            continue;
        }
        let offset = u32_at(bytes, ph + 4)? as usize;
        let paddr = u32_at(bytes, ph + 12)?;
        let filesz = u32_at(bytes, ph + 16)?;
        let memsz = u32_at(bytes, ph + 20)?;
        if memsz == 0 {
            continue;
        }
        if filesz > memsz {
            return Err(ElfError::BadHeader);
        }
        let data = offset
            .checked_add(filesz as usize)
            .and_then(|end| bytes.get(offset..end))
            .ok_or(ElfError::Truncated)?;
        m.mem.load(paddr, data).map_err(|f| unmapped(f, paddr))?;
        let mut at = paddr.checked_add(filesz);
        let end = paddr.checked_add(memsz);
        let (Some(mut a), Some(end)) = (at.take(), end) else {
            return Err(ElfError::Unmapped { addr: paddr });
        };
        while a < end {
            let n = (end - a).min(ZEROS.len() as u32);
            m.mem
                .load(a, &ZEROS[..n as usize])
                .map_err(|f| unmapped(f, a))?;
            a += n;
        }
    }
    Ok(entry)
}
