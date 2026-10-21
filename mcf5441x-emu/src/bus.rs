//! Bus masters and DMA request lines.
//!
//! A peripheral that moves data on request drives the level of its DMA request line ([`DmaLines`], one line per
//! eDMA channel). A *bus master* (a device added with [`Machine::add_bus_master`](crate::Machine::add_bus_master),
//! such as the eDMA) accesses the address space through a [`Bus`] it gets from [`DeviceCx::bus`] in every call into
//! it: register accesses, [`Device::advance`](crate::Device::advance) and
//! [`Device::service`](crate::Device::service).
//!
//! The machine services the bus masters at every instruction boundary, after the due device events and before
//! interrupts are sampled, and after every host-side call ([`Machine::access`](crate::Machine::access),
//! [`Machine::idle_until`](crate::Machine::idle_until), [`Machine::with_device`](crate::Machine::with_device)): it
//! calls [`Device::service`](crate::Device::service) on every bus master, in the order they were added, and again
//! while a round changed the request lines. A request is therefore served in the tick it is raised, before the next
//! instruction.

use crate::device::DeviceCx;
use crate::irq::IrqLines;
use crate::mem::{Fault, Size, Space, truncate};
use crate::stop::StopReason;

/// The DMA request lines: 64, one per channel of the eDMA.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct DmaLines {
    asserted: u64,
}

impl DmaLines {
    /// Assert (`on`) or negate the request of channel `channel` (0-63). Panics when `channel` is out of range.
    pub fn set(&mut self, channel: u8, on: bool) {
        assert!(channel < 64, "DMA request line {channel} out of range");
        let bit = 1u64 << channel;
        if on {
            self.asserted |= bit;
        } else {
            self.asserted &= !bit;
        }
    }

    /// The asserted requests: bit `n` is channel `n`.
    pub fn asserted(&self) -> u64 {
        self.asserted
    }
}

/// The address space as a bus master sees it: RAM and every device but the master itself, whose window is
/// unmapped. Accesses through it are not traced and not counted by the poll detector; a write over decoded code
/// is seen by the next instruction fetched, as with a write by the core. A device reached through it gets a
/// [`DeviceCx`] with the master's clock and lines (and its own `Bus` when it is a bus master too).
///
/// It borrows the master's [`DeviceCx`] (see [`DeviceCx::bus`]): the lines it drives are the ones the master sees
/// in `cx.irq` and `cx.dma` once the bus is dropped.
pub struct Bus<'b> {
    now: u64,
    irq: &'b mut IrqLines,
    dma: &'b mut DmaLines,
    stop: &'b mut Option<StopReason>,
    space: &'b mut Space,
}

impl<'b> Bus<'b> {
    /// The bus of `cx`, when it has an address space.
    pub(crate) fn of(cx: &'b mut DeviceCx<'_>) -> Option<Bus<'b>> {
        Some(Bus {
            now: cx.now,
            irq: &mut *cx.irq,
            dma: &mut *cx.dma,
            stop: &mut *cx.stop,
            space: cx.space.as_deref_mut()?,
        })
    }

    /// Read a big-endian value of `size` at `addr`, zero-extended.
    pub fn read(&mut self, addr: u32, size: Size) -> Result<u32, Fault> {
        if let Some(v) = self.space.ram_read(addr, size) {
            return Ok(v);
        }
        let unmapped = Fault::Unmapped { addr, size };
        let (i, off) = self
            .space
            .devices
            .find(addr, size.bytes())
            .ok_or(unmapped)?;
        self.space
            .call(i, self.now, self.irq, self.dma, self.stop, |d, cx| {
                d.read(off, size, cx)
            })
            .map(|v| truncate(v, size))
            .ok_or(unmapped)
    }

    /// Write the low `size` bytes of `value` at `addr`, big-endian.
    pub fn write(&mut self, addr: u32, size: Size, value: u32) -> Result<(), Fault> {
        if self.space.ram_write(addr, size, value).is_some() {
            return Ok(());
        }
        let unmapped = Fault::Unmapped { addr, size };
        let (i, off) = self
            .space
            .devices
            .find(addr, size.bytes())
            .ok_or(unmapped)?;
        let value = truncate(value, size);
        self.space
            .call(i, self.now, self.irq, self.dma, self.stop, |d, cx| {
                d.write(off, size, value, cx)
            })
            .ok_or(unmapped)
    }

    /// Copy `out.len()` bytes of RAM at `addr` into `out`. Fails with `Unmapped` at `addr` (as a byte access)
    /// unless one RAM region covers them all; devices are never read this way.
    pub fn read_bytes(&mut self, addr: u32, out: &mut [u8]) -> Result<(), Fault> {
        self.space.copy_out(addr, out).ok_or(Fault::Unmapped {
            addr,
            size: Size::Byte,
        })
    }

    /// Copy `data` into RAM at `addr`, as [`Memory::load`](crate::Memory::load): fails with `Unmapped` at `addr`
    /// (as a byte access) unless one RAM region covers it all; devices are never written this way.
    pub fn write_bytes(&mut self, addr: u32, data: &[u8]) -> Result<(), Fault> {
        self.space.load(addr, data)
    }
}
