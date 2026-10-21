//! Big-endian physical memory: sorted RAM regions and device windows behind a faulting access interface.
//!
//! RAM is the fast path. An access that no RAM region covers goes to the device whose window holds all of it,
//! with a [`DeviceCx`] for the clock, the interrupt lines and halting; every such access also ends the current
//! instruction's run of uninterrupted execution, so the machine looks at interrupts and events before the next
//! instruction.

use std::collections::HashMap;

use crate::device::{Access, Device, DeviceCx, DeviceId};
use crate::irq::{InterruptController, IrqLines};
use crate::sched::Devices;
use crate::stop::{Stop, StopReason};

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
    /// The access cannot complete and the run stops with this kind (a fault during exception processing).
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

/// The default for [`Memory::set_poll_limit`].
pub const DEFAULT_POLL_LIMIT: u32 = 10_000;

/// The physical address space: RAM, devices, the interrupt lines and the clock the devices see.
pub struct Memory {
    /// Sorted by base, non-overlapping.
    ram: Vec<Ram>,
    /// Index of the region that served the last access.
    last: usize,
    devices: Devices,
    irq: IrqLines,
    /// The clock, in ticks.
    pub(crate) now: u64,
    /// The machine looks at events, limits and interrupts once `now` reaches this; a device access sets it to 0.
    pub(crate) horizon: u64,
    /// The instruction being executed, for the trace and the poll detector.
    pub(crate) pc: u32,
    /// A halt a device asked for, reported at the next instruction boundary.
    pub(crate) stop: Option<StopReason>,
    trace: Option<Vec<Access>>,
    /// Device reads per (pc, addr) since the last device write. Only counted, never iterated.
    polls: HashMap<(u32, u32), u32>,
    poll_limit: u32,
}

impl Default for Memory {
    fn default() -> Self {
        Memory {
            ram: Vec::new(),
            last: 0,
            devices: Devices::default(),
            irq: IrqLines::default(),
            now: 0,
            horizon: 0,
            pc: 0,
            stop: None,
            trace: None,
            polls: HashMap::new(),
            poll_limit: DEFAULT_POLL_LIMIT,
        }
    }
}

impl std::fmt::Debug for Memory {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Memory")
            .field(
                "ram",
                &self
                    .ram
                    .iter()
                    .map(|r| (r.base, r.len()))
                    .collect::<Vec<_>>(),
            )
            .field("devices", &self.devices.len())
            .field("irq", &self.irq)
            .field("now", &self.now)
            .finish_non_exhaustive()
    }
}

/// `value` truncated to `size`.
#[inline(always)]
fn truncate(value: u32, size: Size) -> u32 {
    match size {
        Size::Byte => value & 0xFF,
        Size::Word => value & 0xFFFF,
        Size::Long => value,
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

impl Memory {
    /// Map zeroed RAM at `base`. Panics when the region is empty, wraps the address space or overlaps another
    /// region or a device.
    pub fn map_ram(&mut self, base: u32, size: u32) {
        let (_, last) = window("RAM", base, size);
        assert!(
            !self.devices.overlaps(base, last),
            "RAM overlaps a device at {base:#x}"
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
    #[inline]
    pub fn read(&mut self, addr: u32, size: Size) -> Result<u32, Fault> {
        let n = size.bytes();
        let Some((i, off)) = self.find(addr, n) else {
            return self.device_read(addr, size);
        };
        let b = &self.ram[i].data[off..off + n as usize];
        Ok(b.iter().fold(0u32, |v, &x| (v << 8) | x as u32))
    }

    /// Write the low bytes of `value`, big-endian.
    #[inline]
    pub fn write(&mut self, addr: u32, size: Size, value: u32) -> Result<(), Fault> {
        let n = size.bytes() as usize;
        let Some((i, off)) = self.find(addr, n as u32) else {
            return self.device_write(addr, size, value);
        };
        self.ram[i].data[off..off + n].copy_from_slice(&value.to_be_bytes()[4 - n..]);
        Ok(())
    }

    /// A read RAM does not cover: the device whose window holds it, or unmapped. More than the poll limit of
    /// reads of one register by one instruction, with no device write in between, asks the run to stop.
    #[cold]
    #[inline(never)]
    fn device_read(&mut self, addr: u32, size: Size) -> Result<u32, Fault> {
        let (i, off) = self
            .devices
            .find(addr, size.bytes())
            .ok_or(Fault::Unmapped { addr, size })?;
        let mut cx = DeviceCx {
            now: self.now,
            irq: &mut self.irq,
            stop: &mut self.stop,
        };
        let value = truncate(self.devices.get_mut(i).read(off, size, &mut cx), size);
        self.horizon = 0;
        self.record(addr, size, false, value);
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
        Ok(value)
    }

    /// A write RAM does not cover: the device whose window holds it, or unmapped.
    #[cold]
    #[inline(never)]
    fn device_write(&mut self, addr: u32, size: Size, value: u32) -> Result<(), Fault> {
        let (i, off) = self
            .devices
            .find(addr, size.bytes())
            .ok_or(Fault::Unmapped { addr, size })?;
        let value = truncate(value, size);
        self.record(addr, size, true, value);
        self.polls.clear();
        let mut cx = DeviceCx {
            now: self.now,
            irq: &mut self.irq,
            stop: &mut self.stop,
        };
        self.devices.get_mut(i).write(off, size, value, &mut cx);
        self.horizon = 0;
        Ok(())
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
        self.devices.add_device(Some(w), dev)
    }

    /// Map interrupt controller `c` at `base..base + size`, as [`Memory::add_device`].
    pub fn add_interrupt_controller(
        &mut self,
        base: u32,
        size: u32,
        c: Box<dyn InterruptController>,
    ) -> DeviceId {
        let w = self.device_window(base, size);
        self.devices.add_controller(w, c)
    }

    /// Add `dev` with no registers: it only schedules events.
    pub fn add_timer(&mut self, dev: Box<dyn Device>) -> DeviceId {
        self.horizon = 0;
        self.devices.add_device(None, dev)
    }

    fn device_window(&mut self, base: u32, size: u32) -> (u32, u32) {
        let (base, last) = window("device", base, size);
        assert!(
            !self
                .ram
                .iter()
                .any(|r| base as u64 <= r.base as u64 + r.len() as u64 - 1 && r.base <= last),
            "device overlaps RAM at {base:#x}"
        );
        self.horizon = 0;
        (base, last)
    }

    /// Device `id`. Panics when it is not a `T`.
    pub fn device<T: Device>(&self, id: DeviceId) -> &T {
        let dev: &dyn std::any::Any = self.devices.get(id.0);
        dev.downcast_ref()
            .unwrap_or_else(|| panic!("device {id:?} is not a {}", std::any::type_name::<T>()))
    }

    /// Device `id`, mutably. Panics when it is not a `T`.
    pub fn device_mut<T: Device>(&mut self, id: DeviceId) -> &mut T {
        self.horizon = 0;
        let dev: &mut dyn std::any::Any = self.devices.get_mut(id.0);
        dev.downcast_mut()
            .unwrap_or_else(|| panic!("device {id:?} is not a {}", std::any::type_name::<T>()))
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
    /// between (default [`DEFAULT_POLL_LIMIT`]).
    pub fn set_poll_limit(&mut self, reads: u32) {
        self.poll_limit = reads;
    }

    /// Forget the reads counted for the poll detector.
    pub(crate) fn reset_polls(&mut self) {
        self.polls.clear();
    }

    /// The earliest event a device has scheduled.
    pub(crate) fn next_event(&self) -> Option<u64> {
        self.devices.next_event()
    }

    /// Advance the devices whose events are due.
    pub(crate) fn advance_due(&mut self) {
        self.devices
            .advance_due(self.now, &mut self.irq, &mut self.stop);
    }

    /// The highest request over all controllers for the request lines `lines`.
    pub(crate) fn pending_with(
        &self,
        lines: &IrqLines,
        allow_level7: bool,
    ) -> Option<crate::irq::Irq> {
        self.devices.pending(lines, allow_level7)
    }

    /// The highest request over all controllers, ignoring level 7 unless `allow_level7`.
    pub(crate) fn pending(&self, allow_level7: bool) -> Option<crate::irq::Irq> {
        self.devices.pending(&self.irq, allow_level7)
    }

    /// Whether the level-7 line is up.
    pub(crate) fn level7(&self) -> bool {
        self.devices.level7(&self.irq)
    }

    /// Copy `bytes` into RAM; fails with the first address that is not covered by a single RAM region. Devices are
    /// not written.
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

    /// Copy `len` bytes out of RAM without side effects; `None` unless one RAM region covers all of them.
    pub fn peek(&self, addr: u32, len: u32) -> Option<Vec<u8>> {
        let i = self
            .ram
            .partition_point(|r| r.base <= addr)
            .checked_sub(1)?;
        let off = self.ram[i].span(addr, len)?;
        Some(self.ram[i].data[off..off + len as usize].to_vec())
    }
}
