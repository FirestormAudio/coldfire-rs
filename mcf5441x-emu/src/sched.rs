//! The device table: address windows, the interrupt controllers and bus masters among the devices, and the events
//! they schedule.
//!
//! Everything here walks the devices in the order they were added, so the order of advances, of bus master
//! servicing and the choice between equal interrupt requests never depend on anything but that order.
//!
//! A bus master is taken out of its slot while it is called (it gets the rest of the address space through a
//! [`Bus`](crate::bus::Bus)) and put back afterwards; while it is out, its window reads as unmapped.

use crate::device::{Device, DeviceId};
use crate::irq::{InterruptController, Irq, IrqLines};

/// A device, keeping the controller interface of the ones that have it.
enum Model {
    Plain(Box<dyn Device>),
    /// A device that may access the address space.
    Master(Box<dyn Device>),
    Controller(Box<dyn InterruptController>),
    /// A bus master taken out of its slot for the duration of a call into it.
    Out,
}

/// What a slot holds, for [`Devices::kind`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Kind {
    /// A device that is not a bus master (a controller is one of these).
    Plain,
    Master,
    /// A bus master that is being called.
    Out,
}

/// A device's address window: `base..=last`.
#[derive(Debug, Clone, Copy)]
struct Window {
    base: u32,
    last: u32,
    index: usize,
}

/// The devices of a [`Memory`](crate::Memory).
#[derive(Default)]
pub(crate) struct Devices {
    /// In the order they were added; a [`DeviceId`] indexes this.
    models: Vec<Model>,
    /// Sorted by base, non-overlapping.
    windows: Vec<Window>,
    /// Indices of the interrupt controllers in `models`.
    controllers: Vec<usize>,
    /// Indices of the bus masters in `models`.
    masters: Vec<usize>,
}

impl Devices {
    pub(crate) fn len(&self) -> usize {
        self.models.len()
    }

    /// Whether `base..=last` overlaps a device window.
    pub(crate) fn overlaps(&self, base: u32, last: u32) -> bool {
        self.windows
            .iter()
            .any(|w| base <= w.last && w.base <= last)
    }

    fn add(&mut self, window: Option<(u32, u32)>, model: Model) -> DeviceId {
        let index = self.models.len();
        if let Some((base, last)) = window {
            assert!(
                !self.overlaps(base, last),
                "device overlaps another at {base:#x}"
            );
            let at = self.windows.partition_point(|w| w.base < base);
            self.windows.insert(at, Window { base, last, index });
        }
        match model {
            Model::Controller(_) => self.controllers.push(index),
            Model::Master(_) => self.masters.push(index),
            _ => {}
        }
        self.models.push(model);
        DeviceId(index)
    }

    pub(crate) fn add_device(
        &mut self,
        window: Option<(u32, u32)>,
        dev: Box<dyn Device>,
    ) -> DeviceId {
        self.add(window, Model::Plain(dev))
    }

    pub(crate) fn add_master(&mut self, window: (u32, u32), dev: Box<dyn Device>) -> DeviceId {
        self.add(Some(window), Model::Master(dev))
    }

    pub(crate) fn add_controller(
        &mut self,
        window: (u32, u32),
        c: Box<dyn InterruptController>,
    ) -> DeviceId {
        self.add(Some(window), Model::Controller(c))
    }

    /// The device whose window holds all of `[addr, addr + len)`, and the offset of `addr` in it.
    pub(crate) fn find(&self, addr: u32, len: u32) -> Option<(usize, u32)> {
        let w = self.windows[..self.windows.partition_point(|w| w.base <= addr)].last()?;
        let off = addr - w.base;
        (off as u64 + len as u64 <= (w.last - w.base) as u64 + 1).then_some((w.index, off))
    }

    /// Device `i`, unless it is a bus master being called.
    fn try_get(&self, i: usize) -> Option<&dyn Device> {
        match &self.models[i] {
            Model::Plain(d) | Model::Master(d) => Some(&**d),
            Model::Controller(c) => Some(&**c),
            Model::Out => None,
        }
    }

    pub(crate) fn get(&self, i: usize) -> &dyn Device {
        self.try_get(i)
            .unwrap_or_else(|| panic!("device {i} is being called"))
    }

    pub(crate) fn get_mut(&mut self, i: usize) -> &mut dyn Device {
        match &mut self.models[i] {
            Model::Plain(d) | Model::Master(d) => &mut **d,
            Model::Controller(c) => &mut **c,
            Model::Out => panic!("device {i} is being called"),
        }
    }

    pub(crate) fn kind(&self, i: usize) -> Kind {
        match self.models[i] {
            Model::Plain(_) | Model::Controller(_) => Kind::Plain,
            Model::Master(_) => Kind::Master,
            Model::Out => Kind::Out,
        }
    }

    /// Take bus master `i` out of its slot; panics unless it is one and is in its slot.
    pub(crate) fn take(&mut self, i: usize) -> Box<dyn Device> {
        match std::mem::replace(&mut self.models[i], Model::Out) {
            Model::Master(d) => d,
            _ => panic!("device {i} is not a bus master in its slot"),
        }
    }

    /// Put bus master `i` back after [`Devices::take`].
    pub(crate) fn restore(&mut self, i: usize, dev: Box<dyn Device>) {
        debug_assert!(matches!(self.models[i], Model::Out));
        self.models[i] = Model::Master(dev);
    }

    /// The bus masters, in the order they were added.
    pub(crate) fn masters(&self) -> &[usize] {
        &self.masters
    }

    /// Whether device `i` has an event due at `now`.
    pub(crate) fn due(&self, i: usize, now: u64) -> bool {
        self.try_get(i)
            .and_then(|d| d.next_event())
            .is_some_and(|t| t <= now)
    }

    /// The earliest event any device has scheduled.
    pub(crate) fn next_event(&self) -> Option<u64> {
        (0..self.models.len())
            .filter_map(|i| self.try_get(i)?.next_event())
            .min()
    }

    fn controllers(&self) -> impl Iterator<Item = &dyn InterruptController> + '_ {
        self.controllers.iter().map(|&i| match &self.models[i] {
            Model::Controller(c) => &**c,
            _ => unreachable!("controller index of a device that is not a controller"),
        })
    }

    /// The highest request over all controllers. At equal levels the controller added first wins: RM 17.3.2 gives
    /// INTC0 the highest priority and INTC2 the lowest, so the chip's controllers are added in that order.
    pub(crate) fn pending(&self, lines: &IrqLines, allow_level7: bool) -> Option<Irq> {
        self.controllers()
            .filter_map(|c| c.pending(lines, allow_level7))
            .fold(None, |best: Option<Irq>, irq| match best {
                Some(b) if b.level >= irq.level => Some(b),
                _ => Some(irq),
            })
    }

    /// Whether any controller holds the level-7 line up.
    pub(crate) fn level7(&self, lines: &IrqLines) -> bool {
        self.controllers().any(|c| c.level7(lines))
    }
}
