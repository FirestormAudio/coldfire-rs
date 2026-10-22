//! The device table: address windows, the interrupt controllers and bus masters among the devices, and the events
//! they schedule.
//!
//! Everything here walks the devices in the order they were added, so the order of advances, of bus master
//! servicing and the choice between equal interrupt requests never depend on anything but that order.
//!
//! A bus master is taken out of its slot while it is called (it gets the rest of the address space through a
//! [`Bus`](crate::bus::Bus)) and put back afterwards; while it is out, its window reads as unmapped.
//!
//! The table also holds the clock gates: 128 gate bits, set when the gate is closed, and the gate each device is
//! tied to, if any. A device whose gate is closed is *gated*: its accesses do not reach it, its events are held
//! (it is neither advanced nor, for a bus master, serviced) and it lists no wake sources. Its request lines stay
//! as it left them. The gated state is kept as one flag per device, so a device access pays one check.

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
    /// The clock gates: bit n is set while gate n is closed.
    closed: u128,
    /// Per device: the gate it is tied to, if any.
    gate: Vec<Option<u8>>,
    /// Per device: its gate is closed.
    gated: Vec<bool>,
    /// Per device: the gated state it was last told of ([`Device::gate`]).
    told: Vec<bool>,
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
        self.gate.push(None);
        self.gated.push(false);
        self.told.push(false);
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

    /// Whether device `i` has an event due at `now`; a gated device has none.
    pub(crate) fn due(&self, i: usize, now: u64) -> bool {
        !self.gated[i]
            && self
                .try_get(i)
                .and_then(|d| d.next_event())
                .is_some_and(|t| t <= now)
    }

    /// The earliest event any device that is not gated has scheduled.
    pub(crate) fn next_event(&self) -> Option<u64> {
        (0..self.models.len())
            .filter(|&i| !self.gated[i])
            .filter_map(|i| self.try_get(i)?.next_event())
            .min()
    }

    /// Whether device `i`'s clock gate is closed.
    #[inline(always)]
    pub(crate) fn gated(&self, i: usize) -> bool {
        self.gated[i]
    }

    /// The clock gates: bit n is set while gate n is closed.
    pub(crate) fn closed(&self) -> u128 {
        self.closed
    }

    /// Set the clock gates at tick `now`, telling every device in its slot whose gated state changed.
    // guess: a gated module's events are held and its interrupt and DMA request lines stay as it left them; an
    // interrupt controller's requests still reach the core while it is gated (none of this is measured).
    pub(crate) fn set_closed(&mut self, closed: u128, now: u64) {
        self.closed = closed;
        for i in 0..self.models.len() {
            self.gated[i] = self.gate[i].is_some_and(|g| closed >> g & 1 != 0);
            self.tell(i, now);
        }
    }

    /// Tie device `i` to gate `gate` (0-127), or to none, at tick `now`. Panics when `gate` is above 127.
    pub(crate) fn set_gate(&mut self, i: usize, gate: Option<u8>, now: u64) {
        assert!(
            gate.is_none_or(|g| g < 128),
            "no clock gate {gate:?}: 0-127"
        );
        self.gate[i] = gate;
        self.gated[i] = gate.is_some_and(|g| self.closed >> g & 1 != 0);
        self.tell(i, now);
    }

    /// Tell device `i` its gated state ([`Device::gate`]) if it changed since it was last told and the device is
    /// in its slot (a bus master being called is told when it is put back).
    pub(crate) fn tell(&mut self, i: usize, now: u64) {
        let gated = self.gated[i];
        if self.told[i] != gated && !matches!(self.models[i], Model::Out) {
            self.told[i] = gated;
            self.get_mut(i).gate(gated, now);
        }
    }

    /// The interrupt controllers with their indices, in the order they were added.
    fn controllers(&self) -> impl Iterator<Item = (usize, &dyn InterruptController)> + '_ {
        self.controllers.iter().map(|&i| match &self.models[i] {
            Model::Controller(c) => (i, &**c),
            _ => unreachable!("controller index of a device that is not a controller"),
        })
    }

    /// Controller `i`, mutably; panics when device `i` is not an interrupt controller.
    pub(crate) fn controller_mut(&mut self, i: usize) -> &mut dyn InterruptController {
        match &mut self.models[i] {
            Model::Controller(c) => &mut **c,
            _ => panic!("device {i} is not an interrupt controller"),
        }
    }

    /// The highest request over all controllers, and the index of the controller making it. At equal levels the
    /// controller added first wins: RM 17.3.2 gives INTC0 the highest priority and INTC2 the lowest, so the
    /// chip's controllers are added in that order.
    pub(crate) fn pending(&self, lines: &IrqLines, allow_level7: bool) -> Option<(usize, Irq)> {
        self.controllers()
            .filter_map(|(i, c)| Some((i, c.pending(lines, allow_level7)?)))
            .fold(None, |best: Option<(usize, Irq)>, (i, irq)| match best {
                Some(b) if b.1.level >= irq.level => Some(b),
                _ => Some((i, irq)),
            })
    }

    /// Whether any controller holds the level-7 line up.
    pub(crate) fn level7(&self, lines: &IrqLines) -> bool {
        self.controllers().any(|(_, c)| c.level7(lines))
    }

    /// Push to `out` the interrupt sources every device that is not gated lists ([`Device::wake_sources`]), in the
    /// order the devices were added, whether it has a pending event or not (a bus master lists the routes of the
    /// requests it would serve); `false` when a device with a pending event cannot say.
    pub(crate) fn wake_sources(&self, out: &mut Vec<(usize, u8)>) -> bool {
        let mut known = true;
        for d in (0..self.models.len())
            .filter(|&i| !self.gated[i])
            .filter_map(|i| self.try_get(i))
        {
            let mark = out.len();
            if !d.wake_sources(out) {
                out.truncate(mark);
                if d.next_event().is_some() {
                    known = false;
                }
            }
        }
        known
    }
}
