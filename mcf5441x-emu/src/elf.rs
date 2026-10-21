//! Loading a linked program: [`load_elf`] reads a 32-bit big-endian m68k ELF executable into a [`Machine`];
//! [`sections`] and [`symbols`] read its sections and symbol table, for building the flat images a boot medium holds
//! (see [`boot`](crate::boot)).

use std::collections::BTreeMap;
use std::fmt;

use crate::machine::Machine;
use crate::mem::Fault;

const EM_68K: u16 = 4;
const PT_LOAD: u32 = 1;
const EHDR_LEN: usize = 52;
const PHDR_LEN: usize = 32;
const SHDR_LEN: usize = 40;
const SHT_PROGBITS: u32 = 1;
const SHT_SYMTAB: u32 = 2;
const SHF_ALLOC: u32 = 2;
const SYM_LEN: usize = 16;
const STB_GLOBAL: u8 = 1;
const STT_SECTION: u8 = 3;
const STT_FILE: u8 = 4;
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
    /// A program or section header is malformed: the header entries are too small, a segment is longer in the
    /// file than in memory, or a symbol table names a string table that is not there.
    BadHeader,
    /// A segment does not fit in mapped RAM; holds the start address of the segment (or zero-fill chunk) that could not be placed.
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
            ElfError::BadHeader => write!(f, "malformed ELF program or section header"),
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

/// Check the ELF identification and machine: 32-bit, big-endian, `EM_68K`.
fn check_header(bytes: &[u8]) -> Result<(), ElfError> {
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
    Ok(())
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
    check_header(bytes)?;
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

/// A section with contents in the file that the program occupies at run time (`SHT_PROGBITS` with `SHF_ALLOC`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Section {
    /// The section's name.
    pub name: String,
    /// Where it runs (`sh_addr`).
    pub addr: u32,
    /// Where it loads: the physical address its bytes have in the `PT_LOAD` segment holding them (`p_paddr` plus
    /// the section's offset into the segment), or `addr` when no segment holds them. This is where a boot medium
    /// keeps the section when it runs elsewhere.
    pub load: u32,
    /// Its bytes.
    pub data: Vec<u8>,
}

/// One section header's fields: name offset, type, flags, address, file offset, size, link.
struct Shdr {
    name: u32,
    kind: u32,
    flags: u32,
    addr: u32,
    offset: usize,
    size: usize,
    link: usize,
}

fn section_headers(bytes: &[u8]) -> Result<Vec<Shdr>, ElfError> {
    check_header(bytes)?;
    let shoff = u32_at(bytes, 32)? as usize;
    let shentsize = u16_at(bytes, 46)? as usize;
    let shnum = u16_at(bytes, 48)? as usize;
    if shnum > 0 && shentsize < SHDR_LEN {
        return Err(ElfError::BadHeader);
    }
    (0..shnum)
        .map(|i| {
            let sh = i
                .checked_mul(shentsize)
                .and_then(|o| o.checked_add(shoff))
                .ok_or(ElfError::Truncated)?;
            if sh.checked_add(SHDR_LEN).is_none_or(|end| end > bytes.len()) {
                return Err(ElfError::Truncated);
            }
            Ok(Shdr {
                name: u32_at(bytes, sh)?,
                kind: u32_at(bytes, sh + 4)?,
                flags: u32_at(bytes, sh + 8)?,
                addr: u32_at(bytes, sh + 12)?,
                offset: u32_at(bytes, sh + 16)? as usize,
                size: u32_at(bytes, sh + 20)? as usize,
                link: u32_at(bytes, sh + 24)? as usize,
            })
        })
        .collect()
}

/// The bytes of a section header's file range.
fn contents<'b>(bytes: &'b [u8], sh: &Shdr) -> Result<&'b [u8], ElfError> {
    sh.offset
        .checked_add(sh.size)
        .and_then(|end| bytes.get(sh.offset..end))
        .ok_or(ElfError::Truncated)
}

/// The NUL-terminated string at `at` in `table`.
fn string(table: &[u8], at: usize) -> Result<String, ElfError> {
    let s = table.get(at..).ok_or(ElfError::Truncated)?;
    let len = s.iter().position(|&b| b == 0).ok_or(ElfError::Truncated)?;
    Ok(String::from_utf8_lossy(&s[..len]).into_owned())
}

/// The `PT_LOAD` segments' (file offset, file size, physical address).
fn load_segments(bytes: &[u8]) -> Result<Vec<(usize, usize, u32)>, ElfError> {
    let phoff = u32_at(bytes, 28)? as usize;
    let phentsize = u16_at(bytes, 42)? as usize;
    let phnum = u16_at(bytes, 44)? as usize;
    if phnum > 0 && phentsize < PHDR_LEN {
        return Err(ElfError::BadHeader);
    }
    let mut out = Vec::new();
    for i in 0..phnum {
        let ph = i
            .checked_mul(phentsize)
            .and_then(|o| o.checked_add(phoff))
            .ok_or(ElfError::Truncated)?;
        if ph.checked_add(PHDR_LEN).is_none_or(|end| end > bytes.len()) {
            return Err(ElfError::Truncated);
        }
        if u32_at(bytes, ph)? == PT_LOAD {
            out.push((
                u32_at(bytes, ph + 4)? as usize,
                u32_at(bytes, ph + 16)? as usize,
                u32_at(bytes, ph + 12)?,
            ));
        }
    }
    Ok(out)
}

/// The sections of the ELF executable `bytes` that the program occupies with contents from the file
/// (`SHT_PROGBITS` with `SHF_ALLOC`; `.bss` and debug sections are left out), in section-table order, each with
/// its run and load address. Every offset is checked against `bytes`; a malformed file gives an [`ElfError`].
pub fn sections(bytes: &[u8]) -> Result<Vec<Section>, ElfError> {
    let headers = section_headers(bytes)?;
    let shstrndx = u16_at(bytes, 50)? as usize;
    let names = match headers.get(shstrndx) {
        Some(sh) => contents(bytes, sh)?,
        None => &[][..],
    };
    let segments = load_segments(bytes)?;
    let mut out = Vec::new();
    for sh in headers
        .iter()
        .filter(|sh| sh.kind == SHT_PROGBITS && sh.flags & SHF_ALLOC != 0)
    {
        let data = contents(bytes, sh)?.to_vec();
        let load = segments
            .iter()
            .find(|&&(off, len, _)| {
                sh.size > 0 && off <= sh.offset && sh.offset + sh.size <= off + len
            })
            .map_or(sh.addr, |&(off, _, paddr)| {
                paddr.wrapping_add((sh.offset - off) as u32)
            });
        out.push(Section {
            name: string(names, sh.name as usize)?,
            addr: sh.addr,
            load,
            data,
        });
    }
    Ok(out)
}

/// The defined symbols of the ELF executable `bytes` (every `SHT_SYMTAB`), by name, with their values. Section and
/// file symbols and undefined ones are left out. When a name occurs more than once, a global symbol wins over a
/// local one, and otherwise the first in the table.
pub fn symbols(bytes: &[u8]) -> Result<BTreeMap<String, u32>, ElfError> {
    let headers = section_headers(bytes)?;
    let mut out: BTreeMap<String, (u32, bool)> = BTreeMap::new();
    for sh in headers.iter().filter(|sh| sh.kind == SHT_SYMTAB) {
        let table = contents(bytes, sh)?;
        let strings = contents(bytes, headers.get(sh.link).ok_or(ElfError::BadHeader)?)?;
        for sym in table.chunks_exact(SYM_LEN) {
            let (name, value) = (u32_at(sym, 0)? as usize, u32_at(sym, 4)?);
            let (info, shndx) = (sym[12], u16_at(sym, 14)?);
            if name == 0 || shndx == 0 || matches!(info & 0xF, STT_SECTION | STT_FILE) {
                continue;
            }
            let global = info >> 4 == STB_GLOBAL;
            let name = string(strings, name)?;
            match out.get(&name) {
                Some(&(_, was_global)) if was_global || !global => {}
                _ => {
                    out.insert(name, (value, global));
                }
            }
        }
    }
    Ok(out.into_iter().map(|(k, (v, _))| (k, v)).collect())
}
