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

use crate::device::{DeviceCx, Refusal, halt_on};
use crate::irq::IrqLines;
use crate::mem::{Fault, Size, Space, truncate};
use crate::stop::StopReason;

/// The DMA request lines: 64, one per channel of the eDMA.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct DmaLines {
    asserted: u64,
    enabled: u64,
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

    /// Publish the channels whose requests the eDMA serves (its ERQ bits): bit `n` is channel `n`.
    pub fn set_enabled(&mut self, channels: u64) {
        self.enabled = channels;
    }

    /// The channels whose requests the eDMA serves, as it last published them. The chip has no such signal: a
    /// peripheral model uses it only for host-side statistics, never for behaviour.
    pub fn enabled(&self) -> u64 {
        self.enabled
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

    /// Read a big-endian value of `size` at `addr`, zero-extended. An access is made as it is, even when it is
    /// not aligned to its size (only the core splits misaligned accesses, RM Table 3-12). An access a device
    /// refuses fails with [`Fault::Device`] and stops the run as [`DeviceCx::halt`] does.
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
        let (v, refused) = self
            .space
            .call(i, self.now, self.irq, self.dma, self.stop, |d, cx| {
                d.read(off, size, cx)
            })
            .ok_or(unmapped)?;
        self.refused(refused)?;
        Ok(truncate(v, size))
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
        let ((), refused) = self
            .space
            .call(i, self.now, self.irq, self.dma, self.stop, |d, cx| {
                d.write(off, size, value, cx)
            })
            .ok_or(unmapped)?;
        self.refused(refused)
    }

    /// A refused access fails with its kind and asks the run to stop with its detail.
    fn refused(&mut self, refused: Option<Refusal>) -> Result<(), Fault> {
        match refused {
            None => Ok(()),
            Some((kind, detail)) => {
                halt_on(self.stop, Some((kind, detail)));
                Err(Fault::Device(kind))
            }
        }
    }

    /// Whether one RAM region covers the access of `size` at `addr` (else it reaches a device, or nothing).
    pub fn is_ram(&mut self, addr: u32, size: Size) -> bool {
        self.space.is_ram(addr, size.bytes())
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
