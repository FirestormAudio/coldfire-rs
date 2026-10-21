//! Stub blocks and unmodelled blocks.

mod common;

use std::cell::RefCell;
use std::rc::Rc;

use common::{BASE, prepare, until};
use mcf5441x_emu::soc::{StubBlock, StubMode, Unmodelled};
use mcf5441x_emu::{Fault, Size, Stop};

const DEV: u32 = 0xEC09_4000;

#[test]
fn canned_values_and_zero_reads() {
    let (mut m, _) = prepare("nop", &[]);
    let id = m.add_device(
        DEV,
        0x100,
        Box::new(StubBlock::new("gpio", StubMode::ReadZero).canned(0x14, 0xCAFE_BABE)),
    );
    assert_eq!(m.access(DEV + 0x14, Size::Long, None), Ok(0xCAFE_BABE));
    // Truncated to the access size; keyed by the exact offset only.
    assert_eq!(m.access(DEV + 0x14, Size::Word, None), Ok(0xBABE));
    assert_eq!(m.access(DEV + 0x14, Size::Byte, None), Ok(0xBE));
    assert_eq!(m.access(DEV + 0x15, Size::Byte, None), Ok(0));
    assert_eq!(m.access(DEV + 0x10, Size::Long, None), Ok(0));
    // A write is accepted and not stored in ReadZero mode.
    m.access(DEV + 0x10, Size::Long, Some(5)).unwrap();
    assert_eq!(m.access(DEV + 0x10, Size::Long, None), Ok(0));
    m.with_device::<StubBlock, _>(id, |d, _| d.set(0x14, 7));
    assert_eq!(m.access(DEV + 0x14, Size::Long, None), Ok(7));
}

#[test]
fn read_back_mode_returns_the_last_write() {
    let (mut m, _) = prepare("nop", &[]);
    m.add_device(
        DEV,
        0x100,
        Box::new(StubBlock::new("flexbus", StubMode::ReadBack).canned(0x4, 9)),
    );
    assert_eq!(
        m.access(DEV + 4, Size::Long, None),
        Ok(9),
        "canned until written"
    );
    m.access(DEV + 4, Size::Long, Some(0x00FF_0001)).unwrap();
    assert_eq!(m.access(DEV + 4, Size::Long, None), Ok(0x00FF_0001));
    assert_eq!(m.access(DEV + 4, Size::Word, None), Ok(1));
    assert_eq!(m.access(DEV + 8, Size::Long, None), Ok(0));
    m.access(DEV + 8, Size::Byte, Some(0x1AB)).unwrap();
    assert_eq!(
        m.access(DEV + 8, Size::Long, None),
        Ok(0xAB),
        "stored truncated"
    );
}

#[test]
fn writes_are_logged_in_order_with_ticks() {
    let (mut m, l) = prepare(
        "move.l #1,%d0\nmove.l %d0,(%a0)\naddq.l #1,%d0\nmove.w %d0,4(%a0)\nend: nop",
        &["end"],
    );
    let id = m.add_device(
        DEV,
        0x100,
        Box::new(StubBlock::new("scm", StubMode::ReadZero)),
    );
    m.cpu.a[0] = DEV;
    m.run(until(l[0]));
    let log = m.device::<StubBlock>(id).writes().to_vec();
    // Instruction n runs at tick n - 1 (the second instruction is tick 1).
    assert_eq!(log, vec![(1, 0, Size::Long, 1), (3, 4, Size::Word, 2)]);
}

#[test]
fn write_hook_sees_offset_and_value_and_can_halt() {
    let (mut m, l) = prepare(
        "move.l #0x12345678,%d0\nmove.l %d0,0x10(%a0)\nmove.l 0x14(%a0),%d1\nnop\nend: nop",
        &["end"],
    );
    let seen = Rc::new(RefCell::new(Vec::new()));
    let mut stub = StubBlock::new("ccm", StubMode::ReadZero).canned(0x14, 0xCAFE_BABE);
    let s = seen.clone();
    stub.on_write(Box::new(move |off, size, v, cx| {
        s.borrow_mut().push((off, size, v));
        cx.halt(Stop::Unmodelled, "reset");
    }));
    m.add_device(DEV, 0x100, Box::new(stub));
    m.cpu.a[0] = DEV;
    let r = m.run(until(l[0]));
    assert_eq!(*seen.borrow(), vec![(0x10, Size::Long, 0x1234_5678)]);
    assert_eq!((r.kind, r.detail.as_str()), (Stop::Unmodelled, "reset"));
    // The halt is after the write instruction; a further run reads the canned value.
    m.run(until(l[0]));
    assert_eq!(m.cpu.d[1], 0xCAFE_BABE);
}

#[test]
fn stub_reads_are_poll_counted() {
    let (mut m, l) = prepare("loop: move.l (%a0),%d0\nbra loop\nend: nop", &["end"]);
    m.add_device(
        DEV,
        0x100,
        Box::new(StubBlock::new("pll", StubMode::ReadZero)),
    );
    m.cpu.a[0] = DEV;
    m.set_poll_limit(50);
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::Poll, "{r:?}");
}

#[test]
fn unmodelled_block_refuses_with_its_name() {
    let (mut m, l) = prepare("move.l %d0,0x20(%a0)\nend: nop", &["end"]);
    m.add_device(DEV, 0x100, Box::new(Unmodelled::new("CAN0")));
    assert_eq!(
        m.access(DEV + 0x20, Size::Long, None),
        Err(Fault::Device(Stop::Unmodelled))
    );
    let r = m.take_refusal().expect("refusal");
    assert_eq!(
        (r.kind, r.detail.as_str()),
        (Stop::Unmodelled, "CAN0 +0x20")
    );
    assert_eq!(
        m.access(DEV + 4, Size::Byte, Some(1)),
        Err(Fault::Device(Stop::Unmodelled))
    );
    assert_eq!(m.take_refusal().unwrap().detail, "CAN0 +0x4");
    m.cpu.a[0] = DEV;
    let r = m.run(until(l[0]));
    assert_eq!((r.kind, r.pc), (Stop::Unmodelled, BASE), "{r:?}");
    assert_eq!(r.detail, "CAN0 +0x20");
}

#[test]
fn take_writes_drains_and_record_off_logs_nothing() {
    let (mut m, l) = prepare(
        "move.l #1,%d0\nmove.l %d0,(%a0)\nmove.l #2,%d0\nmove.w %d0,4(%a0)\nmove.l #3,%d0\nmove.l %d0,8(%a0)\nend: nop",
        &["end"],
    );
    let id = m.add_device(
        DEV,
        0x100,
        Box::new(StubBlock::new("scm", StubMode::ReadBack)),
    );
    m.cpu.a[0] = DEV;
    m.run(until(l[0]));
    assert_eq!(m.device::<StubBlock>(id).writes().len(), 3);
    let taken = m.device_mut::<StubBlock>(id).take_writes();
    assert_eq!(taken.len(), 3);
    assert_eq!(taken[1], (taken[1].0, 4, Size::Word, 2));
    assert!(m.device::<StubBlock>(id).writes().is_empty());
    // behaviour is unchanged with the log off: the register still reads back
    m.device_mut::<StubBlock>(id).record(false);
    m.device_mut::<StubBlock>(id).set(0x20, 7);
    assert_eq!(m.access(DEV + 0x20, Size::Long, None), Ok(7));
    m.access(DEV + 0x24, Size::Long, Some(9)).unwrap();
    assert_eq!(m.access(DEV + 0x24, Size::Long, None), Ok(9));
    assert!(m.device::<StubBlock>(id).writes().is_empty());
}
