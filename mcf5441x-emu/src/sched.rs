//! The device table: address windows, the interrupt controllers among the devices, and the events they schedule.
//!
//! Everything here walks the devices in the order they were added, so the order of advances and the choice
//! between equal interrupt requests never depend on anything but that order.

use crate::device::{Device, DeviceCx, DeviceId};
use crate::irq::{InterruptController, Irq, IrqLines};
use crate::stop::StopReason;

/// A device, keeping the controller interface of the ones that have it.
enum Model {
    Plain(Box<dyn Device>),
    Controller(Box<dyn InterruptController>),
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
        if let Model::Controller(_) = model {
            self.controllers.push(index);
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

    pub(crate) fn get(&self, i: usize) -> &dyn Device {
        match &self.models[i] {
            Model::Plain(d) => &**d,
            Model::Controller(c) => &**c,
        }
    }

    pub(crate) fn get_mut(&mut self, i: usize) -> &mut dyn Device {
        match &mut self.models[i] {
            Model::Plain(d) => &mut **d,
            Model::Controller(c) => &mut **c,
        }
    }

    /// The earliest event any device has scheduled.
    pub(crate) fn next_event(&self) -> Option<u64> {
        (0..self.models.len())
            .filter_map(|i| self.get(i).next_event())
            .min()
    }

    /// Advance every device whose event is due at `now`, in the order they were added.
    pub(crate) fn advance_due(
        &mut self,
        now: u64,
        irq: &mut IrqLines,
        stop: &mut Option<StopReason>,
    ) {
        for i in 0..self.models.len() {
            let dev = self.get_mut(i);
            if dev.next_event().is_some_and(|t| t <= now) {
                dev.advance(now, &mut DeviceCx { now, irq, stop });
            }
        }
    }

    pub(crate) fn has_controllers(&self) -> bool {
        !self.controllers.is_empty()
    }

    fn controllers(&self) -> impl Iterator<Item = &dyn InterruptController> + '_ {
        self.controllers.iter().map(|&i| match &self.models[i] {
            Model::Controller(c) => &**c,
            Model::Plain(_) => unreachable!("controller index of a plain device"),
        })
    }

    /// The highest request over all controllers.
    // guess: RM ch. 17 orders sources within one controller (the higher-numbered source wins) but not equal levels
    // across controllers; the higher vector wins, which continues that order from INTC0 to INTC2.
    pub(crate) fn pending(&self, lines: &IrqLines, allow_level7: bool) -> Option<Irq> {
        self.controllers()
            .filter_map(|c| c.pending(lines, allow_level7))
            .max_by_key(|irq| (irq.level, irq.vector))
    }

    /// Whether any controller holds the level-7 line up.
    pub(crate) fn level7(&self, lines: &IrqLines) -> bool {
        self.controllers().any(|c| c.level7(lines))
    }
}
