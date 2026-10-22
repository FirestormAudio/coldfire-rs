//! Big-endian physical memory: sorted RAM regions and device windows behind a faulting access interface.
//!
//! RAM is the fast path. An access that no RAM region covers goes to the device whose window holds all of it,
//! with a [`DeviceCx`] for the clock, the interrupt and DMA request lines and halting (and, for a bus master, a
//! [`Bus`](crate::bus::Bus)); every such access also ends the current instruction's run of uninterrupted
//! execution, so the machine looks at interrupts and events before the next instruction.
//!
//! An access breakpoint ([`Limit::until_access`](crate::Limit::until_access)) takes the bytes it watches out of the
//! regions the core's operand accesses look in, so that an access to them misses RAM and reaches the path RAM does not
//! cover, where the breakpoint is checked; every other path finds them as RAM. Without one, the core looks in the
//! regions as mapped.
//!
//! A mirror ([`Memory::map_mirror`]) is a second address range for RAM already mapped: its lookup entry points at
//! the same backing store, so every path that finds RAM (core accesses, instruction fetch, bus masters and the host)
//! sees the same bytes through either range, at no cost to the lookup.
//!
//! Memory also keeps one bit per 4 KiB page for the pages holding decoded instructions. A write into such a page,
//! by an instruction, a bus master or [`Memory::load`], clears the bit, reports the page to the machine and ends the run of
//! uninterrupted execution, so the page's decoded blocks are dropped before the next instruction.

use std::collections::HashMap;

use crate::bus::DmaLines;
use crate::device::{Access, Device, DeviceCx, DeviceId, Refusal, halt_on};
use crate::irq::{InterruptController, Irq, IrqLines};
use crate::sched::{Devices, Kind};
use crate::stop::{AccessKind, Stop, StopReason};

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
    /// The access cannot complete and the run stops with this kind: a device refused it
    /// ([`DeviceCx::refuse`]), or a fault during exception processing.
    Device(Stop),
}

/// A RAM range in the address space: `len` bytes at `base`, held by a backing store from offset `at`. A region
/// mapped by [`Memory::map_ram`] has a store of its own; a mirror points into the store of the RAM it mirrors.
#[derive(Debug, Clone, Copy)]
pub struct Ram {
    base: u32,
    len: u32,
    /// Index of the backing store.
    store: usize,
    /// Offset of `base` in the store.
    at: usize,
}

impl Ram {
    /// First address of the region.
    pub fn base(&self) -> u32 {
        self.base
    }

    /// Size in bytes.
    pub fn len(&self) -> u32 {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Offset in the backing store of `[addr, addr + len)` when it lies wholly inside the region.
    fn span(&self, addr: u32, len: u32) -> Option<usize> {
        let off = addr.checked_sub(self.base)? as usize;
        let end = off.checked_add(len as usize)?;
        (end <= self.len as usize).then_some(self.at + off)
    }

    /// Whether the region overlaps `[base, last]`.
    fn overlaps(&self, base: u32, last: u32) -> bool {
        base as u64 <= self.base as u64 + self.len as u64 - 1 && self.base <= last
    }
}

/// An access breakpoint: the bytes `first..=last` and the accesses it stops at.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Watch {
    first: u32,
    last: u32,
    kind: AccessKind,
}

impl Watch {
    /// The watch for `(base, len, kind)`; `None` when `len` is 0.
    fn new((base, len, kind): (u32, u32, AccessKind)) -> Option<Watch> {
        let last = (base as u64 + len.checked_sub(1)? as u64).min(u32::MAX as u64) as u32;
        Some(Watch {
            first: base,
            last,
            kind,
        })
    }

    /// Whether an access of `size` at `addr` (a write when `write`) stops here.
    fn hits(&self, addr: u32, size: Size, write: bool) -> bool {
        let end = addr as u64 + (size.bytes() - 1) as u64;
        self.kind.matches(write) && addr <= self.last && end >= self.first as u64
    }
}

/// The region of `ram` holding `[addr, addr + len)`, trying `last` first and caching the one found there: its
/// backing store and the offset in it.
#[inline]
fn lookup(ram: &[Ram], last: &mut Ram, addr: u32, len: u32) -> Option<(usize, usize)> {
    if let Some(off) = last.span(addr, len) {
        return Some((last.store, off));
    }
    let i = ram.partition_point(|r| r.base <= addr).checked_sub(1)?;
    let r = ram[i];
    let off = r.span(addr, len)?;
    *last = r;
    Some((r.store, off))
}

/// A region no access of a byte or more lies in.
const NO_RAM: Ram = Ram {
    base: 0,
    len: 0,
    store: 0,
    at: 0,
};

/// The default for [`Memory::set_poll_limit`].
pub const DEFAULT_POLL_LIMIT: u32 = 10_000;

/// The most rounds of bus master servicing in one go; a round after that which still changes the DMA request lines
/// stops the run.
const SERVICE_ROUNDS: u32 = 64;

/// The address space proper: RAM, the device table and the record of decoded code. This is what a bus master's
/// [`Bus`](crate::bus::Bus) reaches; the lines, the clock and the bookkeeping of the core's accesses stay in
/// [`Memory`].
pub(crate) struct Space {
    /// Sorted by base, non-overlapping.
    ram: Vec<Ram>,
    /// The bytes the regions hold, one store per [`Memory::map_ram`].
    stores: Vec<Vec<u8>>,
    /// Whether any region is a mirror, so that a page of code may have other addresses.
    mirrored: bool,
    /// A copy of the region that served the last access, so that the next access to it needs no other load to
    /// find its store.
    last: Ram,
    /// The regions the core's operand accesses look in: `ram` without the bytes `watch` covers, so that an
    /// access to them takes the path RAM does not cover, where the watch is checked. The same as `ram` without a
    /// watch.
    core: Vec<Ram>,
    /// As `last`, for `core`.
    core_last: Ram,
    /// The access breakpoint of the run ([`Limit::until_access`](crate::Limit::until_access)).
    watch: Option<Watch>,
    pub(crate) devices: Devices,
    /// One bit per 4 KiB page of the address space: the page holds bytes of a decoded block.
    code: Box<[u64]>,
    /// Pages whose code was written since the machine last dropped their blocks.
    pub(crate) written: Vec<u32>,
}

/// The physical address space: RAM, devices, the interrupt and DMA request lines and the clock the devices see.
pub struct Memory {
    pub(crate) space: Space,
    irq: IrqLines,
    dma: DmaLines,
    /// The clock, in ticks.
    pub(crate) now: u64,
    /// The machine looks at events, limits and interrupts once `now` reaches this; a device access sets it to 0.
    pub(crate) horizon: u64,
    /// The instruction being executed, for the trace and the poll detector.
    pub(crate) pc: u32,
    /// A halt a device asked for, reported at the next instruction boundary.
    pub(crate) stop: Option<StopReason>,
    /// The detail of the last access a device refused ([`DeviceCx::refuse`]), for the stop its
    /// [`Fault::Device`] ends in.
    pub(crate) refused: Option<String>,
    /// The last access a host-side call refused, for [`Machine::take_refusal`](crate::Machine::take_refusal).
    pub(crate) host_refusal: Option<StopReason>,
    trace: Option<Vec<Access>>,
    /// Device reads per (pc, addr) since the last device write. Only counted, never iterated.
    polls: HashMap<(u32, u32), u32>,
    poll_limit: u32,
    /// The interrupt request lines at the last [`Memory::mark`].
    marked_irq: IrqLines,
}

/// What [`Memory::mark`] notes, for [`Memory::unchanged_since`].
#[derive(Debug, Clone, Copy)]
pub(crate) struct Mark {
    touched: u64,
    dma: DmaLines,
    masters_event: Option<u64>,
}

/// Log2 of the page size the block cache tracks writes in.
pub(crate) const PAGE_SHIFT: u32 = 12;

impl Default for Memory {
    fn default() -> Self {
        Memory {
            space: Space {
                ram: Vec::new(),
                stores: Vec::new(),
                mirrored: false,
                last: NO_RAM,
                core: Vec::new(),
                core_last: NO_RAM,
                watch: None,
                devices: Devices::default(),
                code: vec![0; 1 << (32 - PAGE_SHIFT - 6)].into_boxed_slice(),
                written: Vec::new(),
            },
            irq: IrqLines::default(),
            dma: DmaLines::default(),
            now: 0,
            horizon: 0,
            pc: 0,
            stop: None,
            refused: None,
            host_refusal: None,
            trace: None,
            polls: HashMap::new(),
            poll_limit: DEFAULT_POLL_LIMIT,
            marked_irq: IrqLines::default(),
        }
    }
}

impl std::fmt::Debug for Memory {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Memory")
            .field(
                "ram",
                &self
                    .space
                    .ram
                    .iter()
                    .map(|r| (r.base, r.len()))
                    .collect::<Vec<_>>(),
            )
            .field("devices", &self.space.devices.len())
            .field("irq", &self.irq)
            .field("dma", &self.dma)
            .field("now", &self.now)
            .finish_non_exhaustive()
    }
}

/// `value` truncated to `size`.
#[inline(always)]
pub(crate) fn truncate(value: u32, size: Size) -> u32 {
    match size {
        Size::Byte => value & 0xFF,
        Size::Word => value & 0xFFFF,
        Size::Long => value,
    }
}

/// The aligned accesses RM Table 3-12 splits a misaligned operand access into, in ascending address order, and
/// how many there are: a word at an odd address is byte, byte; a long at address\[1:0\] = 01 or 11 is byte, word,
/// byte; a long at 10 is word, word. An aligned access is itself.
fn pieces(addr: u32, size: Size) -> ([(u32, Size); 3], usize) {
    let at = |n: u32| addr.wrapping_add(n);
    match (size, addr & 3) {
        (Size::Word, 1 | 3) => (
            [(addr, Size::Byte), (at(1), Size::Byte), (0, Size::Byte)],
            2,
        ),
        (Size::Long, 1 | 3) => (
            [(addr, Size::Byte), (at(1), Size::Word), (at(3), Size::Byte)],
            3,
        ),
        (Size::Long, 2) => (
            [(addr, Size::Word), (at(2), Size::Word), (0, Size::Byte)],
            2,
        ),
        _ => ([(addr, size), (0, Size::Byte), (0, Size::Byte)], 1),
    }
}

/// `base..=last` for a window of `size` bytes; panics when it is empty or wraps the address space.
fn window(what: &str, base: u32, size: u32) -> (u32, u32) {
    let last = size
        .checked_sub(1)
        .and_then(|n| base.checked_add(n))
        .unwrap_or_else(|| panic!("bad {what} region {base:#x}+{size:#x}"));
    (base, last)
}

impl Space {
    /// Find the region holding `[addr, addr + len)`; returns its backing store and the offset in it.
    fn find(&mut self, addr: u32, len: u32) -> Option<(usize, usize)> {
        lookup(&self.ram, &mut self.last, addr, len)
    }

    /// As [`Space::find`], for the core's operand accesses: the bytes the watch covers are not found.
    fn find_core(&mut self, addr: u32, len: u32) -> Option<(usize, usize)> {
        lookup(&self.core, &mut self.core_last, addr, len)
    }

    /// Set the core's regions: the RAM regions less the watched bytes.
    fn carve(&mut self) {
        self.core_last = NO_RAM;
        self.core.clear();
        for &r in &self.ram {
            let Some(w) = self.watch.filter(|w| r.overlaps(w.first, w.last)) else {
                self.core.push(r);
                continue;
            };
            let end = r.base as u64 + r.len as u64;
            if w.first > r.base {
                self.core.push(Ram {
                    len: w.first - r.base,
                    ..r
                });
            }
            if (w.last as u64) + 1 < end {
                let base = w.last + 1;
                self.core.push(Ram {
                    base,
                    len: (end - base as u64) as u32,
                    store: r.store,
                    at: r.at + (base - r.base) as usize,
                });
            }
        }
    }

    /// Append to `pages` the pages holding the same bytes as some byte of `page`, `page` among them when it is
    /// RAM.
    fn aliases(&self, page: u32, pages: &mut Vec<u32>) {
        let (first, last) = (
            page << PAGE_SHIFT,
            page << PAGE_SHIFT | ((1 << PAGE_SHIFT) - 1),
        );
        for r in self.ram.iter().filter(|r| r.overlaps(first, last)) {
            // The part of the page in `r`, as offsets in its store.
            let lo = r.at + (first.max(r.base) - r.base) as usize;
            let hi = r.at + (last.min(r.base + (r.len - 1)) - r.base) as usize;
            for o in self.ram.iter().filter(|o| o.store == r.store) {
                let (o_lo, o_hi) = (o.at, o.at + (o.len as usize - 1));
                if lo <= o_hi && o_lo <= hi {
                    let a = o.base + (lo.max(o_lo) - o.at) as u32;
                    let b = o.base + (hi.min(o_hi) - o.at) as u32;
                    pages.extend(a >> PAGE_SHIFT..=b >> PAGE_SHIFT);
                }
            }
        }
    }

    /// Whether one RAM region covers `[addr, addr + len)`.
    pub(crate) fn is_ram(&mut self, addr: u32, len: u32) -> bool {
        self.find(addr, len).is_some()
    }

    /// A RAM read: `None` unless one region covers the access.
    #[inline(always)]
    pub(crate) fn ram_read(&mut self, addr: u32, size: Size) -> Option<u32> {
        let n = size.bytes();
        let (i, off) = self.find(addr, n)?;
        let b = &self.stores[i][off..off + n as usize];
        Some(b.iter().fold(0u32, |v, &x| (v << 8) | x as u32))
    }

    /// A RAM write: `None` unless one region covers the access, else whether it wrote into a page holding
    /// decoded code (reported in `written`).
    #[inline(always)]
    pub(crate) fn ram_write(&mut self, addr: u32, size: Size, value: u32) -> Option<bool> {
        let (i, off) = self.find(addr, size.bytes())?;
        Some(self.store_write(i, off, addr, size, value))
    }

    /// As [`Space::ram_write`], for the core's operand accesses ([`Space::find_core`]).
    #[inline(always)]
    fn core_write(&mut self, addr: u32, size: Size, value: u32) -> Option<bool> {
        let (i, off) = self.find_core(addr, size.bytes())?;
        Some(self.store_write(i, off, addr, size, value))
    }

    /// Write `value` at offset `off` of store `i`, which holds `addr`; whether it wrote into a page holding
    /// decoded code (reported in `written`).
    #[inline(always)]
    fn store_write(&mut self, i: usize, off: usize, addr: u32, size: Size, value: u32) -> bool {
        let n = size.bytes() as usize;
        self.stores[i][off..off + n].copy_from_slice(&value.to_be_bytes()[4 - n..]);
        let (first, last) = (addr >> PAGE_SHIFT, (addr + (n as u32 - 1)) >> PAGE_SHIFT);
        let code = self.has_code(first) || self.has_code(last);
        if code {
            self.code_written(first, last);
        }
        code
    }

    /// Whether `page` holds bytes of a decoded block.
    #[inline(always)]
    fn has_code(&self, page: u32) -> bool {
        self.code[(page >> 6) as usize & (self.code.len() - 1)] >> (page & 63) & 1 != 0
    }

    /// RAM in pages `first..=last` was written: report the ones holding code in `written`, so that their blocks
    /// are dropped before the next instruction. With mirrors, a page holding code has its bit set at every address
    /// of its bytes ([`Memory::mark_code`]), and a page reported takes with it every page holding code that shares
    /// a byte with it, and so on: an alias page can hold bytes of several code pages, and once its bit is clear a
    /// later write through it would reach none of them.
    #[cold]
    #[inline(never)]
    fn code_written(&mut self, first: u32, last: u32) {
        let mut todo = Vec::new();
        for page in first..=last {
            if !self.has_code(page) {
                continue;
            }
            todo.push(page);
            while let Some(p) = todo.pop() {
                if self.has_code(p) {
                    self.code[(p >> 6) as usize] &= !(1 << (p & 63));
                    self.written.push(p);
                    if self.mirrored {
                        self.aliases(p, &mut todo);
                    }
                }
            }
        }
    }

    /// Copy `bytes` into RAM; `Unmapped` at `addr` unless one region covers them all.
    pub(crate) fn load(&mut self, addr: u32, bytes: &[u8]) -> Result<(), Fault> {
        if bytes.is_empty() {
            return Ok(());
        }
        let (i, off) = self.find(addr, bytes.len() as u32).ok_or(Fault::Unmapped {
            addr,
            size: Size::Byte,
        })?;
        self.stores[i][off..off + bytes.len()].copy_from_slice(bytes);
        let last = addr + (bytes.len() as u32 - 1);
        self.code_written(addr >> PAGE_SHIFT, last >> PAGE_SHIFT);
        Ok(())
    }

    /// Copy RAM at `addr` into `out`; `None` unless one region covers it all.
    pub(crate) fn copy_out(&mut self, addr: u32, out: &mut [u8]) -> Option<()> {
        if out.is_empty() {
            return Some(());
        }
        let (i, off) = self.find(addr, out.len() as u32)?;
        out.copy_from_slice(&self.stores[i][off..off + out.len()]);
        Some(())
    }

    /// Call device `i` with a context at `now`: a bus master is taken out of its slot and gets the rest of the
    /// address space as its bus. Returns what `f` returned and the refusal the device made, if any; `None`,
    /// without a call, when `i` is a bus master already being called. Clock gates the device set are applied
    /// when it returns.
    pub(crate) fn call<R>(
        &mut self,
        i: usize,
        now: u64,
        irq: &mut IrqLines,
        dma: &mut DmaLines,
        stop: &mut Option<StopReason>,
        f: impl FnOnce(&mut dyn Device, &mut DeviceCx) -> R,
    ) -> Option<(R, Option<Refusal>)> {
        let gates = self.devices.closed();
        let (r, refused, set) = match self.devices.kind(i) {
            Kind::Out => return None,
            Kind::Plain => {
                let mut cx = DeviceCx {
                    now,
                    irq,
                    dma,
                    stop,
                    space: None,
                    refused: None,
                    gates,
                };
                let r = f(self.devices.get_mut(i), &mut cx);
                (r, cx.refused, cx.gates)
            }
            Kind::Master => {
                let mut dev = self.devices.take(i);
                let mut cx = DeviceCx {
                    now,
                    irq,
                    dma,
                    stop,
                    space: Some(self),
                    refused: None,
                    gates,
                };
                let r = f(&mut *dev, &mut cx);
                let (refused, set) = (cx.refused, cx.gates);
                self.devices.restore(i, dev);
                // Gates closed through its bus while it was out of its slot.
                self.devices.tell(i, now);
                (r, refused, set)
            }
        };
        if set != gates {
            self.devices.set_closed(set, now);
        }
        Some((r, refused))
    }

    /// An access to device `i` at `off`, as [`Space::call`]: the value read (0 for a write) and whether the
    /// device's reads are poll-counted. A gated device is not called: a read returns all ones at every size and a
    /// write is dropped (both measured on an MCF54415); the read is poll-counted.
    pub(crate) fn access(
        &mut self,
        i: usize,
        off: u32,
        size: Size,
        write: Option<u32>,
        now: u64,
        irq: &mut IrqLines,
        dma: &mut DmaLines,
        stop: &mut Option<StopReason>,
    ) -> Option<((u32, bool), Option<Refusal>)> {
        self.devices.touched = self.devices.touched.wrapping_add(1);
        if self.devices.gated(i) && self.devices.kind(i) != Kind::Out {
            let value = if write.is_some() {
                0
            } else {
                truncate(u32::MAX, size)
            };
            return Some(((value, true), None));
        }
        self.call(i, now, irq, dma, stop, |d, cx| match write {
            Some(value) => {
                d.write(off, size, truncate(value, size), cx);
                (0, d.poll_counted())
            }
            None => (truncate(d.read(off, size, cx), size), d.poll_counted()),
        })
    }

    /// Call device `i` outside an access: a refusal acts as a halt. `None` as for [`Space::call`].
    pub(crate) fn call_halting<R>(
        &mut self,
        i: usize,
        now: u64,
        irq: &mut IrqLines,
        dma: &mut DmaLines,
        stop: &mut Option<StopReason>,
        f: impl FnOnce(&mut dyn Device, &mut DeviceCx) -> R,
    ) -> Option<R> {
        let (r, refused) = self.call(i, now, irq, dma, &mut *stop, f)?;
        halt_on(stop, refused);
        Some(r)
    }
}

impl Memory {
    /// Map zeroed RAM at `base`. Panics when the region is empty, wraps the address space or overlaps another
    /// region (a mirror among them) or a device.
    pub fn map_ram(&mut self, base: u32, size: u32) {
        let store = self.space.stores.len();
        self.insert_ram("RAM", base, size, store, 0);
        self.space.stores.push(vec![0; size as usize]);
    }

    /// Map `[base, base + size)` as a mirror of the RAM at `[of, of + size)`: one backing store seen at both
    /// ranges, by every access, instruction fetch, bus master and host call, with cached code dropped whichever
    /// range writes over it. A mirror can itself be mirrored. Panics when the range is empty, wraps the address
    /// space or overlaps RAM, a mirror or a device, or when no single RAM region (or mirror) covers `[of, of + size)`.
    pub fn map_mirror(&mut self, base: u32, size: u32, of: u32) {
        let target = self
            .space
            .ram
            .iter()
            .find_map(|r| r.span(of, size).map(|at| (r.store, at)));
        let Some((store, at)) = target else {
            panic!("mirror at {base:#x}: no RAM region covers {of:#x}+{size:#x}");
        };
        self.insert_ram("mirror", base, size, store, at);
        self.space.mirrored = true;
        // Code already decoded from the mirrored bytes is now at a second address as well.
        let code = &self.space.code;
        let pages: Vec<u32> = (0..code.len())
            .filter(|&w| code[w] != 0)
            .flat_map(|w| {
                (0..64)
                    .filter(move |b| code[w] >> b & 1 != 0)
                    .map(move |b| (w * 64 + b) as u32)
            })
            .collect();
        for page in pages {
            self.mark_code(page, page);
        }
    }

    /// Insert a region of `size` bytes at `base`, held by `store` from offset `at`, as [`Memory::map_ram`]
    /// describes.
    fn insert_ram(&mut self, what: &str, base: u32, size: u32, store: usize, at: usize) {
        let (_, last) = window(what, base, size);
        let space = &mut self.space;
        assert!(
            !space.devices.overlaps(base, last),
            "{what} overlaps a device at {base:#x}"
        );
        assert!(
            !space.ram.iter().any(|r| r.overlaps(base, last)),
            "{what} overlaps RAM at {base:#x}"
        );
        let i = space.ram.partition_point(|r| r.base < base);
        let r = Ram {
            base,
            len: size,
            store,
            at,
        };
        space.ram.insert(i, r);
        space.last = NO_RAM;
        space.carve();
    }

    /// Set the access breakpoint of the core's accesses ([`Limit::until_access`](crate::Limit::until_access)).
    pub(crate) fn set_watch(&mut self, watch: Option<(u32, u32, AccessKind)>) {
        let watch = watch.and_then(Watch::new);
        if watch != self.space.watch {
            self.space.watch = watch;
            self.space.carve();
        }
    }

    /// Read a big-endian value, zero-extended.
    #[inline]
    pub fn read(&mut self, addr: u32, size: Size) -> Result<u32, Fault> {
        let n = size.bytes();
        let Some((i, off)) = self.space.find_core(addr, n) else {
            return self.device_read(addr, size);
        };
        let b = &self.space.stores[i][off..off + n as usize];
        Ok(b.iter().fold(0u32, |v, &x| (v << 8) | x as u32))
    }

    /// Write the low bytes of `value`, big-endian.
    #[inline]
    pub fn write(&mut self, addr: u32, size: Size, value: u32) -> Result<(), Fault> {
        match self.space.core_write(addr, size, value) {
            None => self.device_write(addr, size, value),
            Some(code) => {
                if code {
                    self.horizon = 0;
                }
                Ok(())
            }
        }
    }

    /// Mark pages `first..=last` as holding bytes of a decoded block, and with mirrors every other page holding
    /// the same bytes, so that a write through any address of them drops the block.
    pub(crate) fn mark_code(&mut self, first: u32, last: u32) {
        for page in first..=last {
            self.space.code[(page >> 6) as usize] |= 1 << (page & 63);
            if self.space.mirrored {
                let mut pages = Vec::new();
                self.space.aliases(page, &mut pages);
                for p in pages {
                    self.space.code[(p >> 6) as usize] |= 1 << (p & 63);
                }
            }
        }
    }

    /// Whether one RAM region holds all of `[addr, addr + len)`.
    pub(crate) fn in_ram(&mut self, addr: u32, len: u32) -> bool {
        self.space.find(addr, len).is_some()
    }

    /// Look at interrupts before the next instruction: the interrupt mask may have changed. The horizon is brought
    /// forward to the end of the instruction, not to 0, so the machine can tell that only SR was written since it
    /// last set the horizon (see [`Machine`](crate::Machine)'s boundary).
    #[inline(always)]
    pub(crate) fn resample(&mut self) {
        self.horizon = self.horizon.min(self.now + 1);
    }

    /// An instruction-stream word: RAM, or a read of the device whose window holds it that is neither traced nor
    /// counted by the poll detector.
    #[inline]
    pub(crate) fn fetch(&mut self, addr: u32) -> Result<u32, Fault> {
        match self.space.ram_read(addr, Size::Word) {
            Some(v) => Ok(v),
            None => self.device_fetch(addr),
        }
    }

    #[cold]
    #[inline(never)]
    fn device_fetch(&mut self, addr: u32) -> Result<u32, Fault> {
        self.device_access(addr, Size::Word, None, false)
            .map(|(v, _)| v)
    }

    /// A read RAM does not cover: the devices, or unmapped. More than the poll limit of reads of one register by
    /// one instruction, with no device write in between, asks the run to stop.
    #[cold]
    #[inline(never)]
    fn device_read(&mut self, addr: u32, size: Size) -> Result<u32, Fault> {
        if let Some(w) = self.space.watch {
            if w.hits(addr, size, false) {
                return Err(self.watched(addr, size, None));
            }
            // RAM next to the watched bytes, or in them for an access the watch does not stop.
            if let Some(v) = self.space.ram_read(addr, size) {
                return Ok(v);
            }
        }
        self.device_op(addr, size, None, true)
    }

    /// A write RAM does not cover: the devices, or unmapped.
    #[cold]
    #[inline(never)]
    fn device_write(&mut self, addr: u32, size: Size, value: u32) -> Result<(), Fault> {
        if let Some(w) = self.space.watch {
            if w.hits(addr, size, true) {
                return Err(self.watched(addr, size, Some(value)));
            }
            if let Some(code) = self.space.ram_write(addr, size, value) {
                if code {
                    self.horizon = 0;
                }
                return Ok(());
            }
        }
        self.polls.clear();
        self.device_op(addr, size, Some(value), false).map(|_| ())
    }

    /// The core's access of `size` at `addr` hit the watch: it is not made, and the run stops before the
    /// instruction (or exception) making it.
    #[cold]
    fn watched(&mut self, addr: u32, size: Size, write: Option<u32>) -> Fault {
        self.refused = Some(match write {
            Some(v) => format!("{size:?} write of {:#x} at {addr:#010x}", truncate(v, size)),
            None => format!("{size:?} read at {addr:#010x}"),
        });
        Fault::Device(Stop::UntilAccess)
    }

    /// An operand access to the devices: one access when it is aligned to its size, else the aligned pieces of
    /// [`pieces`], in order, each a traced device access of its own. A misaligned access is unmapped as a whole,
    /// with no piece made, unless every piece falls in a device window; a piece a device refuses ends it there,
    /// after the pieces before it were made. Reads are poll-counted when `counted`. The value read, or 0 for a
    /// write.
    fn device_op(
        &mut self,
        addr: u32,
        size: Size,
        write: Option<u32>,
        counted: bool,
    ) -> Result<u32, Fault> {
        let (list, n) = pieces(addr, size);
        if n == 1 {
            return self.piece(addr, size, write, counted);
        }
        if list[..n]
            .iter()
            .any(|&(a, s)| self.space.devices.find(a, s.bytes()).is_none())
        {
            return Err(Fault::Unmapped { addr, size });
        }
        let mut value = 0;
        let mut left = size.bytes();
        for &(a, s) in &list[..n] {
            left -= s.bytes();
            let part = write.map(|w| truncate(w >> (8 * left), s));
            value = value << (8 * s.bytes()) | self.piece(a, s, part, counted)?;
        }
        Ok(value)
    }

    /// One aligned device access, as [`Memory::device_op`] makes them.
    fn piece(
        &mut self,
        addr: u32,
        size: Size,
        write: Option<u32>,
        counted: bool,
    ) -> Result<u32, Fault> {
        let (value, poll) = self.device_access(addr, size, write, true)?;
        if counted && poll && write.is_none() {
            let pc = self.pc;
            let n = self.polls.entry((pc, addr)).or_insert(0);
            *n += 1;
            if *n > self.poll_limit && self.stop.is_none() {
                self.stop = Some(StopReason {
                    kind: Stop::Poll,
                    pc,
                    detail: format!("spinning at {pc:#x} reading {addr:#x}"),
                });
            }
        }
        Ok(value)
    }

    /// An access to the device whose window holds all of it, traced when `traced`: the value read (0 for a
    /// write) and whether the device's reads are poll-counted. Ends the run of uninterrupted execution. A refused
    /// access is traced like any other, fails with [`Fault::Device`] and leaves its detail in `refused`.
    fn device_access(
        &mut self,
        addr: u32,
        size: Size,
        write: Option<u32>,
        traced: bool,
    ) -> Result<(u32, bool), Fault> {
        let unmapped = Fault::Unmapped { addr, size };
        let (i, off) = self
            .space
            .devices
            .find(addr, size.bytes())
            .ok_or(unmapped)?;
        if let Some(value) = write
            && traced
        {
            self.record(addr, size, true, truncate(value, size));
        }
        let r = self.space.access(
            i,
            off,
            size,
            write,
            self.now,
            &mut self.irq,
            &mut self.dma,
            &mut self.stop,
        );
        let ((value, counted), refused) = r.ok_or(unmapped)?;
        self.horizon = 0;
        if write.is_none() && traced {
            self.record(addr, size, false, value);
        }
        if let Some((kind, detail)) = refused {
            self.refused = Some(detail);
            return Err(Fault::Device(kind));
        }
        Ok((value, counted))
    }

    /// An access made from outside the core at the current tick: RAM, or device accesses (split as the core's
    /// are) traced as the core's would be and not counted by the poll detector. The value read, or 0 for a
    /// write. A refusal's kind and detail are kept for [`Machine::take_refusal`](crate::Machine::take_refusal).
    pub(crate) fn host_access(
        &mut self,
        addr: u32,
        size: Size,
        write: Option<u32>,
    ) -> Result<u32, Fault> {
        self.horizon = 0;
        let r = match write {
            None => match self.space.ram_read(addr, size) {
                Some(v) => Ok(v),
                None => self.device_op(addr, size, None, false),
            },
            Some(value) if self.space.ram_write(addr, size, value).is_some() => Ok(0),
            Some(value) => self.device_op(addr, size, Some(value), false),
        };
        let detail = self.refused.take();
        if let Err(Fault::Device(kind)) = r {
            self.host_refusal = Some(StopReason {
                kind,
                pc: self.pc,
                detail: detail.unwrap_or_default(),
            });
        }
        r
    }

    fn record(&mut self, addr: u32, size: Size, write: bool, value: u32) {
        if let Some(trace) = &mut self.trace {
            trace.push(Access {
                t: self.now,
                pc: self.pc,
                addr,
                size,
                write,
                value,
            });
        }
    }

    /// Map `dev` at `base..base + size`. Panics when the window is empty, wraps the address space or overlaps RAM
    /// or another device.
    pub fn add_device(&mut self, base: u32, size: u32, dev: Box<dyn Device>) -> DeviceId {
        let w = self.device_window(base, size);
        self.space.devices.add_device(Some(w), dev)
    }

    /// Map bus master `dev` at `base..base + size`, as [`Memory::add_device`]: every call into it gets a
    /// [`Bus`](crate::bus::Bus), and it is serviced at every instruction boundary (see [`crate::bus`]).
    pub fn add_bus_master(&mut self, base: u32, size: u32, dev: Box<dyn Device>) -> DeviceId {
        let w = self.device_window(base, size);
        self.space.devices.add_master(w, dev)
    }

    /// Map interrupt controller `c` at `base..base + size`, as [`Memory::add_device`].
    pub fn add_interrupt_controller(
        &mut self,
        base: u32,
        size: u32,
        c: Box<dyn InterruptController>,
    ) -> DeviceId {
        let w = self.device_window(base, size);
        self.space.devices.add_controller(w, c)
    }

    /// Add `dev` with no registers: it only schedules events.
    pub fn add_timer(&mut self, dev: Box<dyn Device>) -> DeviceId {
        self.horizon = 0;
        self.space.devices.add_device(None, dev)
    }

    fn device_window(&mut self, base: u32, size: u32) -> (u32, u32) {
        let (base, last) = window("device", base, size);
        assert!(
            !self.space.ram.iter().any(|r| r.overlaps(base, last)),
            "device overlaps RAM at {base:#x}"
        );
        self.horizon = 0;
        (base, last)
    }

    /// Device `id`. Panics when it is not a `T`.
    pub fn device<T: Device>(&self, id: DeviceId) -> &T {
        let dev: &dyn std::any::Any = self.space.devices.get(id.0);
        dev.downcast_ref()
            .unwrap_or_else(|| panic!("device {id:?} is not a {}", std::any::type_name::<T>()))
    }

    /// Device `id`, mutably. Panics when it is not a `T`.
    pub fn device_mut<T: Device>(&mut self, id: DeviceId) -> &mut T {
        self.horizon = 0;
        let dev: &mut dyn std::any::Any = self.space.devices.get_mut(id.0);
        dev.downcast_mut()
            .unwrap_or_else(|| panic!("device {id:?} is not a {}", std::any::type_name::<T>()))
    }

    /// Call `f` on device `id` with a context at the current tick (with a [`Bus`](crate::bus::Bus) when it is a
    /// bus master). Panics when it is not a `T`. Bus masters are not serviced afterwards; see
    /// [`Machine::with_device`](crate::Machine::with_device).
    pub(crate) fn with_device<T: Device, R>(
        &mut self,
        id: DeviceId,
        f: impl FnOnce(&mut T, &mut DeviceCx) -> R,
    ) -> R {
        self.horizon = 0;
        self.space
            .call_halting(
                id.0,
                self.now,
                &mut self.irq,
                &mut self.dma,
                &mut self.stop,
                |d, cx| {
                    let d: &mut dyn std::any::Any = d;
                    let d = d.downcast_mut().unwrap_or_else(|| {
                        panic!("device {id:?} is not a {}", std::any::type_name::<T>())
                    });
                    f(d, cx)
                },
            )
            .expect("no device is being called")
    }

    /// The clock gates: bit n is set while gate n is closed. All are open (0) until set.
    pub fn clock_gates(&self) -> u128 {
        self.space.devices.closed()
    }

    /// Set the clock gates at the current tick (bit n set: gate n closed); the devices whose gated state changes
    /// are told ([`Device::gate`]).
    pub fn set_clock_gates(&mut self, closed: u128) {
        self.horizon = 0;
        self.space.devices.set_closed(closed, self.now);
    }

    /// Tie device `id` to clock gate `gate` (0-127), or to none (the default: never gated). While its gate is
    /// closed the device is *gated*: an access to it reads all ones or is dropped without calling it, and its
    /// events are held (see [`Device::gate`]). Calls from the host
    /// ([`Machine::with_device`](crate::Machine::with_device), [`Memory::device_mut`]) still reach it. Panics when
    /// `gate` is above 127.
    pub fn set_clock_gate(&mut self, id: DeviceId, gate: Option<u8>) {
        self.horizon = 0;
        self.space.devices.set_gate(id.0, gate, self.now);
    }

    /// Whether device `id`'s clock gate is closed.
    pub fn gated(&self, id: DeviceId) -> bool {
        self.space.devices.gated(id.0)
    }

    /// The device whose window holds `addr`, if any.
    pub fn device_at(&self, addr: u32) -> Option<DeviceId> {
        self.space.devices.find(addr, 1).map(|(i, _)| DeviceId(i))
    }

    /// The interrupt request lines.
    pub fn irq_lines(&self) -> &IrqLines {
        &self.irq
    }

    /// The interrupt request lines, to drive from outside the devices. A change is seen at the next instruction
    /// boundary.
    pub fn irq_lines_mut(&mut self) -> &mut IrqLines {
        self.horizon = 0;
        &mut self.irq
    }

    /// The DMA request lines.
    pub fn dma_lines(&self) -> &DmaLines {
        &self.dma
    }

    /// Record every device access ([`Memory::take_trace`]) or stop recording.
    pub fn trace(&mut self, on: bool) {
        match (on, &self.trace) {
            (true, None) => self.trace = Some(Vec::new()),
            (false, _) => self.trace = None,
            (true, Some(_)) => {}
        }
    }

    /// The device accesses recorded since tracing started or the trace was last taken.
    pub fn take_trace(&mut self) -> Vec<Access> {
        self.trace.as_mut().map(std::mem::take).unwrap_or_default()
    }

    /// Stop a run after more than `reads` reads of one device register by one instruction with no device write in
    /// between (default [`DEFAULT_POLL_LIMIT`]). Reads of a device whose [`Device::poll_counted`] is `false` are
    /// not counted.
    pub fn set_poll_limit(&mut self, reads: u32) {
        self.poll_limit = reads;
    }

    /// Forget the reads counted for the poll detector.
    pub(crate) fn reset_polls(&mut self) {
        self.polls.clear();
    }

    /// The earliest event a device has scheduled.
    pub(crate) fn next_event(&self) -> Option<u64> {
        self.space.devices.next_event()
    }

    /// Advance the devices whose events are due, in the order they were added.
    pub(crate) fn advance_due(&mut self) {
        let now = self.now;
        for i in 0..self.space.devices.len() {
            if self.space.devices.due(i, now) {
                self.space.call_halting(
                    i,
                    now,
                    &mut self.irq,
                    &mut self.dma,
                    &mut self.stop,
                    |d, cx| d.advance(now, cx),
                );
            }
        }
    }

    /// Whether [`Memory::service_masters`] would call any bus master now: one that is not gated serves the DMA
    /// request lines as they are ([`Device::serves`]).
    pub(crate) fn masters_serve(&self) -> bool {
        let devices = &self.space.devices;
        devices
            .masters()
            .iter()
            .any(|&i| !devices.gated(i) && devices.try_get(i).is_some_and(|d| d.serves(&self.dma)))
    }

    /// Call [`Device::service`] on every bus master, in the order they were added, until a round leaves the DMA
    /// request lines as it found them. When [`SERVICE_ROUNDS`] rounds all change them, asks the run to stop.
    pub(crate) fn service_masters(&mut self) {
        if self.space.devices.masters().is_empty() {
            return;
        }
        let now = self.now;
        for _ in 0..SERVICE_ROUNDS {
            let before = self.dma;
            for k in 0..self.space.devices.masters().len() {
                let i = self.space.devices.masters()[k];
                if self.space.devices.gated(i)
                    || !self
                        .space
                        .devices
                        .try_get(i)
                        .is_some_and(|d| d.serves(&self.dma))
                {
                    continue;
                }
                self.space.call_halting(
                    i,
                    now,
                    &mut self.irq,
                    &mut self.dma,
                    &mut self.stop,
                    |d, cx| d.service(cx),
                );
            }
            if self.dma == before {
                return;
            }
        }
        if self.stop.is_none() {
            self.stop = Some(StopReason {
                kind: Stop::Unmodelled,
                pc: 0,
                detail: format!(
                    "DMA requests never settle: bus masters changed the request lines ({:#018x}) in each of \
                     {SERVICE_ROUNDS} rounds",
                    self.dma.asserted()
                ),
            });
        }
    }

    /// Note what a call into the bus masters could change without calling another device: the device accesses
    /// made, the DMA and interrupt request lines, the masters' events.
    pub(crate) fn mark(&mut self) -> Mark {
        self.marked_irq.clone_from(&self.irq);
        Mark {
            touched: self.space.devices.touched,
            dma: self.dma,
            masters_event: self.space.devices.masters_event(),
        }
    }

    /// Whether, since `mark`, no device access was made, no clock gate was set, the request lines and the bus
    /// masters' events are as they were, no code was written and no halt asked for.
    pub(crate) fn unchanged_since(&self, mark: Mark) -> bool {
        self.space.devices.touched == mark.touched
            && self.dma == mark.dma
            && self.irq == self.marked_irq
            && self.stop.is_none()
            && self.space.written.is_empty()
            && self.space.devices.masters_event() == mark.masters_event
    }

    /// The highest request over all controllers for the request lines `lines`.
    pub(crate) fn pending_with(&self, lines: &IrqLines, allow_level7: bool) -> Option<Irq> {
        self.space
            .devices
            .pending(lines, allow_level7)
            .map(|(_, irq)| irq)
    }

    /// The highest request over all controllers, ignoring level 7 unless `allow_level7`, and the index of the
    /// controller making it.
    pub(crate) fn pending(&self, allow_level7: bool) -> Option<(usize, Irq)> {
        self.space.devices.pending(&self.irq, allow_level7)
    }

    /// Tell controller `i` that the core took its request `irq` ([`InterruptController::acknowledge`]). A
    /// refusal acts as a halt.
    pub(crate) fn acknowledge(&mut self, i: usize, irq: Irq) {
        self.horizon = 0;
        let gates = self.space.devices.closed();
        let mut cx = DeviceCx {
            now: self.now,
            irq: &mut self.irq,
            dma: &mut self.dma,
            stop: &mut self.stop,
            space: None,
            refused: None,
            gates,
        };
        self.space
            .devices
            .controller_mut(i)
            .acknowledge(irq, &mut cx);
        let (refused, set) = (cx.refused, cx.gates);
        halt_on(&mut self.stop, refused);
        if set != gates {
            self.space.devices.set_closed(set, self.now);
        }
    }

    /// The request lines as they could become while the core waits after STOP: the asserted lines plus every
    /// source the devices list ([`Device::wake_sources`]); `None` when a device with a pending event cannot say.
    pub(crate) fn wake_lines(&self) -> Option<IrqLines> {
        let mut sources = Vec::new();
        if !self.space.devices.wake_sources(&mut sources) {
            return None;
        }
        let mut lines = self.irq.clone();
        for (controller, source) in sources {
            lines.set(controller, source, true);
        }
        Some(lines)
    }

    /// Whether the level-7 line is up.
    pub(crate) fn level7(&self) -> bool {
        self.space.devices.level7(&self.irq)
    }

    /// Copy `bytes` into RAM; fails with `Unmapped` at `addr`, the start of the copy, unless a single RAM region
    /// covers all of `bytes`. Devices are not written.
    pub fn load(&mut self, addr: u32, bytes: &[u8]) -> Result<(), Fault> {
        self.space.load(addr, bytes)?;
        if !self.space.written.is_empty() {
            self.horizon = 0;
        }
        Ok(())
    }

    /// Copy `len` bytes out of RAM without side effects; `None` unless one RAM region covers all of them.
    pub fn peek(&self, addr: u32, len: u32) -> Option<Vec<u8>> {
        let ram = &self.space.ram;
        let i = ram.partition_point(|r| r.base <= addr).checked_sub(1)?;
        let r = &ram[i];
        let off = r.span(addr, len)?;
        Some(self.space.stores[r.store][off..off + len as usize].to_vec())
    }
}
