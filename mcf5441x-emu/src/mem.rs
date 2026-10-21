//! Big-endian physical memory: sorted RAM regions behind a faulting access interface.

use crate::stop::Stop;

/// Access width.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Size {
    Byte,
    Word,
    Long,
}

impl Size {
    /// Width in bytes.
    pub fn bytes(self) -> u32 {
        match self {
            Size::Byte => 1,
            Size::Word => 2,
            Size::Long => 4,
        }
    }
}

/// A failed access.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fault {
    /// No memory covers the whole access.
    Unmapped { addr: u32, size: Size },
    /// A device stopped the machine.
    Device(Stop),
}

/// Big-endian byte storage at a base address.
#[derive(Debug, Clone)]
pub struct Ram {
    base: u32,
    data: Vec<u8>,
}

impl Ram {
    fn new(base: u32, size: u32) -> Ram {
        Ram {
            base,
            data: vec![0; size as usize],
        }
    }

    /// First address of the region.
    pub fn base(&self) -> u32 {
        self.base
    }

    /// Size in bytes.
    pub fn len(&self) -> u32 {
        self.data.len() as u32
    }

    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    /// Offset of `[addr, addr + len)` when it lies wholly inside the region.
    fn span(&self, addr: u32, len: u32) -> Option<usize> {
        let off = addr.checked_sub(self.base)? as usize;
        let end = off.checked_add(len as usize)?;
        (end <= self.data.len()).then_some(off)
    }
}

/// The physical address space.
#[derive(Debug, Clone, Default)]
pub struct Memory {
    /// Sorted by base, non-overlapping.
    ram: Vec<Ram>,
    /// Index of the region that served the last access.
    last: usize,
}

impl Memory {
    /// Map zeroed RAM at `base`. Panics when the region is empty, wraps the address space or overlaps another.
    pub fn map_ram(&mut self, base: u32, size: u32) {
        assert!(
            size > 0 && base.checked_add(size - 1).is_some(),
            "bad RAM region {base:#x}+{size:#x}"
        );
        let end = base as u64 + size as u64;
        let at = self.ram.partition_point(|r| r.base < base);
        if let Some(prev) = at.checked_sub(1).map(|i| &self.ram[i]) {
            assert!(
                prev.base as u64 + prev.data.len() as u64 <= base as u64,
                "RAM overlaps at {base:#x}"
            );
        }
        if let Some(next) = self.ram.get(at) {
            assert!(end <= next.base as u64, "RAM overlaps at {base:#x}");
        }
        self.ram.insert(at, Ram::new(base, size));
        self.last = 0;
    }

    /// Find the region holding `[addr, addr + len)`; returns its index and the offset.
    fn find(&mut self, addr: u32, len: u32) -> Option<(usize, usize)> {
        if let Some(off) = self.ram.get(self.last).and_then(|r| r.span(addr, len)) {
            return Some((self.last, off));
        }
        let i = self
            .ram
            .partition_point(|r| r.base <= addr)
            .checked_sub(1)?;
        let off = self.ram[i].span(addr, len)?;
        self.last = i;
        Some((i, off))
    }

    /// Read a big-endian value, zero-extended.
    pub fn read(&mut self, addr: u32, size: Size) -> Result<u32, Fault> {
        let n = size.bytes();
        let (i, off) = self.find(addr, n).ok_or(Fault::Unmapped { addr, size })?;
        let b = &self.ram[i].data[off..off + n as usize];
        Ok(b.iter().fold(0u32, |v, &x| (v << 8) | x as u32))
    }

    /// Write the low bytes of `value`, big-endian.
    pub fn write(&mut self, addr: u32, size: Size, value: u32) -> Result<(), Fault> {
        let n = size.bytes() as usize;
        let (i, off) = self
            .find(addr, n as u32)
            .ok_or(Fault::Unmapped { addr, size })?;
        self.ram[i].data[off..off + n].copy_from_slice(&value.to_be_bytes()[4 - n..]);
        Ok(())
    }

    /// Copy `bytes` into memory; fails with the first address that is not covered by a single region.
    pub fn load(&mut self, addr: u32, bytes: &[u8]) -> Result<(), Fault> {
        if bytes.is_empty() {
            return Ok(());
        }
        let (i, off) = self.find(addr, bytes.len() as u32).ok_or(Fault::Unmapped {
            addr,
            size: Size::Byte,
        })?;
        self.ram[i].data[off..off + bytes.len()].copy_from_slice(bytes);
        Ok(())
    }

    /// Copy `len` bytes out without side effects; `None` unless one region covers all of them.
    pub fn peek(&self, addr: u32, len: u32) -> Option<Vec<u8>> {
        let i = self
            .ram
            .partition_point(|r| r.base <= addr)
            .checked_sub(1)?;
        let off = self.ram[i].span(addr, len)?;
        Some(self.ram[i].data[off..off + len as usize].to_vec())
    }
}
