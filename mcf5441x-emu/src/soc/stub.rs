//! Blocks that accept writes and read back canned values, and blocks nobody models.

use std::any::Any;
use std::collections::BTreeMap;
use std::ops::Range;

use crate::device::{Device, DeviceCx};
use crate::mem::Size;
use crate::stop::Stop;

/// What a [`StubBlock`] reads at an offset that has no canned value.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StubMode {
    /// Zero; writes are logged and dropped.
    ReadZero,
    /// The last value written at that exact offset, else zero.
    ReadBack,
}

/// A write hook: offset, size, value (the low `size` bytes) and the device context.
type WriteHook = Box<dyn FnMut(u32, Size, u32, &mut DeviceCx)>;

/// A register window that accepts every access.
///
/// A read returns the canned value at that exact offset if there is one, else zero ([`StubMode::ReadZero`]) or
/// the last value written at that exact offset ([`StubMode::ReadBack`]), truncated to the access size. There is no
/// byte-lane slicing: a value is keyed by the offset it was given at. Writes are logged (unless
/// [`StubBlock::record`] turned the log off), passed to the hook, and stored in `ReadBack` mode. Reads are counted
/// by the poll detector.
///
/// A range of offsets can be given to a register model of its own ([`StubBlock::with_registers`]): accesses there
/// are served by the model, and writes there are still logged and passed to the hook.
pub struct StubBlock {
    name: &'static str,
    mode: StubMode,
    canned: BTreeMap<u32, u32>,
    stored: BTreeMap<u32, u32>,
    log: Vec<(u64, u32, Size, u32)>,
    record: bool,
    hook: Option<WriteHook>,
    registers: Option<(Range<u32>, Box<dyn Device>)>,
}

impl StubBlock {
    /// A stub named `name` (for the board's own diagnostics) in `mode`.
    pub fn new(name: &'static str, mode: StubMode) -> StubBlock {
        StubBlock {
            name,
            mode,
            canned: BTreeMap::new(),
            stored: BTreeMap::new(),
            log: Vec::new(),
            record: true,
            hook: None,
            registers: None,
        }
    }

    /// Serve the accesses at offsets `range` with `model` (at the same offsets) instead of the canned and stored
    /// values. Writes there are logged and passed to the hook as any other. `range` should be aligned to 4 bytes,
    /// so that no aligned access runs across its ends.
    pub fn with_registers(mut self, range: Range<u32>, model: Box<dyn Device>) -> StubBlock {
        self.registers = Some((range, model));
        self
    }

    /// The model [`StubBlock::with_registers`] gave, if it is a `T`.
    pub fn registers<T: Device>(&self) -> Option<&T> {
        let (_, model) = self.registers.as_ref()?;
        (&**model as &dyn Any).downcast_ref()
    }

    /// The model [`StubBlock::with_registers`] gave, mutably, if it is a `T`.
    pub fn registers_mut<T: Device>(&mut self) -> Option<&mut T> {
        let (_, model) = self.registers.as_mut()?;
        (&mut **model as &mut dyn Any).downcast_mut()
    }

    /// The name given to [`StubBlock::new`].
    pub fn name(&self) -> &'static str {
        self.name
    }

    /// This exact offset reads `value`.
    pub fn canned(mut self, offset: u32, value: u32) -> StubBlock {
        self.canned.insert(offset, value);
        self
    }

    /// Change the canned value at `offset` from the host. In `ReadBack` mode a value written by the program at
    /// that offset is replaced too.
    pub fn set(&mut self, offset: u32, value: u32) {
        self.canned.insert(offset, value);
        self.stored.remove(&offset);
    }

    /// Every write so far: (tick, offset, size, value), in order.
    pub fn writes(&self) -> &[(u64, u32, Size, u32)] {
        &self.log
    }

    /// Every write so far, emptying the log.
    pub fn take_writes(&mut self) -> Vec<(u64, u32, Size, u32)> {
        std::mem::take(&mut self.log)
    }

    /// Whether writes are logged (default on). Off changes only the log, never the registers or the hook.
    pub fn record(&mut self, on: bool) {
        self.record = on;
    }

    /// Call `f` on every write, after it is logged (and stored), with the offset, size, value and context: a
    /// board's hook, e.g. to halt on a reset bit.
    pub fn on_write(&mut self, f: Box<dyn FnMut(u32, Size, u32, &mut DeviceCx)>) {
        self.hook = Some(f);
    }
}

fn truncate(value: u32, size: Size) -> u32 {
    match size {
        Size::Byte => value & 0xFF,
        Size::Word => value & 0xFFFF,
        Size::Long => value,
    }
}

impl Device for StubBlock {
    fn read(&mut self, offset: u32, size: Size, cx: &mut DeviceCx) -> u32 {
        if let Some((range, model)) = &mut self.registers
            && range.contains(&offset)
        {
            return truncate(model.read(offset, size, cx), size);
        }
        let v = match self.mode {
            StubMode::ReadBack => self
                .stored
                .get(&offset)
                .or_else(|| self.canned.get(&offset)),
            StubMode::ReadZero => self.canned.get(&offset),
        };
        truncate(v.copied().unwrap_or(0), size)
    }

    fn write(&mut self, offset: u32, size: Size, value: u32, cx: &mut DeviceCx) {
        let value = truncate(value, size);
        if self.record {
            self.log.push((cx.now, offset, size, value));
        }
        match &mut self.registers {
            Some((range, model)) if range.contains(&offset) => model.write(offset, size, value, cx),
            _ if self.mode == StubMode::ReadBack => {
                self.stored.insert(offset, value);
            }
            _ => {}
        }
        if let Some(hook) = &mut self.hook {
            hook(offset, size, value, cx);
        }
    }
}

/// A module window nobody models: every access is refused with [`Stop::Unmodelled`] and the detail
/// `"<name> +<offset>"`, so a program that reaches it stops on that instruction.
pub struct Unmodelled {
    name: &'static str,
}

impl Unmodelled {
    /// A window for the module `name`.
    pub fn new(name: &'static str) -> Unmodelled {
        Unmodelled { name }
    }
}

impl Device for Unmodelled {
    fn read(&mut self, offset: u32, _size: Size, cx: &mut DeviceCx) -> u32 {
        cx.refuse(Stop::Unmodelled, format!("{} +{offset:#x}", self.name));
        0
    }

    fn write(&mut self, offset: u32, _size: Size, _value: u32, cx: &mut DeviceCx) {
        cx.refuse(Stop::Unmodelled, format!("{} +{offset:#x}", self.name));
    }

    fn poll_counted(&self) -> bool {
        false
    }
}
