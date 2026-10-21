//! The machine interface device models rely on: refused accesses, misaligned accesses to a device, instruction
//! fetches from a device, STOP wake hints and the interrupt acknowledge. The test devices are defined here.

mod common;

use common::{BASE, assemble, prepare, set_vector, until};
use mcf5441x_emu::{
    Access, Device, DeviceCx, Fault, InterruptController, Irq, IrqLines, Limit, Machine, Size, Stop,
};

/// Where the test devices are mapped.
const DEV: u32 = 0xEC00_0000;
/// Where the test interrupt controllers are mapped.
const INTC: u32 = 0xFC04_8000;

/// A register block whose every read or write is refused, as a model's tripwire refuses an access to a feature it
/// does not model.
struct Refuser;

impl Device for Refuser {
    fn read(&mut self, offset: u32, _: Size, cx: &mut DeviceCx) -> u32 {
        cx.refuse(
            Stop::Unmodelled,
            format!("test block: read of {offset:#x} not modelled"),
        );
        0xDEAD_BEEF
    }
    fn write(&mut self, offset: u32, _: Size, _: u32, cx: &mut DeviceCx) {
        cx.refuse(
            Stop::Unmodelled,
            format!("test block: write of {offset:#x} not modelled"),
        );
    }
}

#[test]
fn refused_access_rolls_back_and_stops_on_the_instruction() {
    let (mut m, l) = prepare("move.l (%a0)+,%d0\nend: nop", &["end"]);
    m.add_device(DEV, 0x100, Box::new(Refuser));
    m.cpu.a[0] = DEV + 4;
    m.cpu.d[0] = 0x1234_5678;
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::Unmodelled, "{r:?}");
    assert_eq!(r.pc, BASE, "PC stays on the refused instruction");
    assert_eq!(r.detail, "test block: read of 0x4 not modelled");
    assert_eq!(m.cpu.pc, BASE);
    assert_eq!(m.cpu.a[0], DEV + 4, "the postincrement is rolled back");
    assert_eq!(m.cpu.d[0], 0x1234_5678, "the destination is not written");
    assert_eq!(m.instructions(), 0, "the instruction did not complete");
    // The next run tries the instruction again and is refused again: nothing is left over from the first.
    let r = m.run(until(l[0]));
    assert_eq!((r.kind, r.pc), (Stop::Unmodelled, BASE), "{r:?}");
}

#[test]
fn refused_write_after_a_read_rolls_back_the_flags() {
    // ADD.L D1,(A0): the read is served by RAM-like storage, the write is refused; CCR and A0 stay as they were.
    struct ReadOnly;
    impl Device for ReadOnly {
        fn read(&mut self, _: u32, _: Size, _: &mut DeviceCx) -> u32 {
            0xFFFF_FFFF
        }
        fn write(&mut self, _: u32, _: Size, _: u32, cx: &mut DeviceCx) {
            cx.refuse(Stop::Unmodelled, "test block: read-only");
        }
    }
    let (mut m, l) = prepare("add.l %d1,-(%a0)\nend: nop", &["end"]);
    m.add_device(DEV, 0x100, Box::new(ReadOnly));
    m.cpu.a[0] = DEV + 8;
    m.cpu.d[1] = 1;
    m.cpu.set_ccr(0x0A);
    let r = m.run(until(l[0]));
    assert_eq!((r.kind, r.pc), (Stop::Unmodelled, BASE), "{r:?}");
    assert_eq!(r.detail, "test block: read-only");
    assert_eq!(m.cpu.a[0], DEV + 8);
    assert_eq!(m.cpu.ccr(), 0x0A, "the flags of the addition are not kept");
}

#[test]
fn refused_host_access_is_an_error_and_leaves_no_stop() {
    // A host-side access fails with the refusal's kind; the detail is kept for `take_refusal` (there is no run to
    // stop), so the next run starts normally.
    let (mut m, l) = prepare("nop\nend: nop", &["end"]);
    m.add_device(DEV, 0x100, Box::new(Refuser));
    assert_eq!(
        m.access(DEV, Size::Long, None),
        Err(Fault::Device(Stop::Unmodelled))
    );
    let taken = m.take_refusal().expect("refusal kept");
    assert_eq!(taken.kind, Stop::Unmodelled);
    assert_eq!(taken.detail, "test block: read of 0x0 not modelled");
    assert_eq!(m.take_refusal(), None, "taking clears it");
    assert_eq!(
        m.access(DEV, Size::Word, Some(1)),
        Err(Fault::Device(Stop::Unmodelled))
    );
    let taken = m.take_refusal().expect("refusal kept");
    assert_eq!(taken.detail, "test block: write of 0x0 not modelled");
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
}

#[test]
fn refusal_outside_an_access_halts() {
    // A refusal from `advance` (no access in flight) stops the run at that boundary like a halt, with its detail.
    struct Tripwire(Option<u64>);
    impl Device for Tripwire {
        fn read(&mut self, _: u32, _: Size, _: &mut DeviceCx) -> u32 {
            0
        }
        fn write(&mut self, _: u32, _: Size, _: u32, _: &mut DeviceCx) {}
        fn next_event(&self) -> Option<u64> {
            self.0
        }
        fn advance(&mut self, _: u64, cx: &mut DeviceCx) {
            self.0 = None;
            cx.refuse(Stop::Unmodelled, "test block: event not modelled");
        }
    }
    let (mut m, l) = prepare("nop\nnop\nnop\nend: nop", &["end"]);
    m.add_timer(Box::new(Tripwire(Some(2))));
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::Unmodelled, "{r:?}");
    assert_eq!(r.detail, "test block: event not modelled");
    assert_eq!(r.pc, BASE + 4, "stops at the boundary of tick 2");
}

/// Byte storage that records every access it gets as (offset, size, write, value).
struct Recorder {
    bytes: [u8; 0x100],
    log: Vec<(u32, Size, bool, u32)>,
}

impl Recorder {
    fn new() -> Recorder {
        Recorder {
            bytes: std::array::from_fn(|i| 0xA0u8.wrapping_add(i as u8)),
            log: Vec::new(),
        }
    }
}

impl Device for Recorder {
    fn read(&mut self, offset: u32, size: Size, _: &mut DeviceCx) -> u32 {
        let o = offset as usize;
        let v = self.bytes[o..o + size.bytes() as usize]
            .iter()
            .fold(0, |v, &b| v << 8 | b as u32);
        self.log.push((offset, size, false, v));
        v
    }
    fn write(&mut self, offset: u32, size: Size, value: u32, _: &mut DeviceCx) {
        let n = size.bytes() as usize;
        let o = offset as usize;
        self.bytes[o..o + n].copy_from_slice(&value.to_be_bytes()[4 - n..]);
        self.log.push((offset, size, true, value));
    }
}

#[test]
fn misaligned_long_to_a_device_is_split_into_aligned_pieces() {
    // RM Table 3-12: the core splits a misaligned operand into aligned accesses, in ascending address order. A
    // word at an odd address is byte, byte; a long at address[1:0] = 01 or 11 is byte, word, byte; a long at
    // 10 is word, word. Each piece is a device access of its own, traced, with its big-endian lanes.
    let src = "move.l %d0,(%a0)\n\
               move.l (%a1),%d1\n\
               move.w (%a2),%d2\n\
               move.l (%a3),%d3\n\
               move.w %d4,(%a2)\n\
               move.l %d5,(%a4)\n\
               end: nop";
    let (mut m, l) = prepare(src, &["end"]);
    let id = m.add_device(DEV, 0x100, Box::new(Recorder::new()));
    m.cpu.a[0] = DEV + 0x11;
    m.cpu.a[1] = DEV + 0x23;
    m.cpu.a[2] = DEV + 0x31;
    m.cpu.a[3] = DEV + 0x42;
    m.cpu.a[4] = DEV + 0x52;
    m.cpu.d[0] = 0x1122_3344;
    m.cpu.d[4] = 0x5566;
    m.cpu.d[5] = 0x7788_99AA;
    m.trace(true);
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    use Size::{Byte, Long, Word};
    let log = m.device::<Recorder>(id).log.clone();
    assert_eq!(
        log.as_slice(),
        &[
            // move.l %d0,(DEV+0x11): address[1:0] = 01 → byte, word, byte.
            (0x11, Byte, true, 0x11),
            (0x12, Word, true, 0x2233),
            (0x14, Byte, true, 0x44),
            // move.l (DEV+0x23),%d1: address[1:0] = 11 → byte, word, byte.
            (0x23, Byte, false, 0xC3),
            (0x24, Word, false, 0xC4C5),
            (0x26, Byte, false, 0xC6),
            // move.w (DEV+0x31),%d2: odd → byte, byte.
            (0x31, Byte, false, 0xD1),
            (0x32, Byte, false, 0xD2),
            // move.l (DEV+0x42),%d3: address[1:0] = 10 → word, word.
            (0x42, Word, false, 0xE2E3),
            (0x44, Word, false, 0xE4E5),
            // move.w %d4,(DEV+0x31): byte, byte.
            (0x31, Byte, true, 0x55),
            (0x32, Byte, true, 0x66),
            // move.l %d5,(DEV+0x52): word, word.
            (0x52, Word, true, 0x7788),
            (0x54, Word, true, 0x99AA),
        ]
    );
    assert_eq!(m.cpu.d[1], 0xC3C4_C5C6, "the pieces assemble big-endian");
    assert_eq!(m.cpu.d[2] & 0xFFFF, 0xD1D2);
    assert_eq!(m.cpu.d[3], 0xE2E3_E4E5);
    // Each piece is traced as a device access of the instruction making it.
    let trace: Vec<(u32, u32, Size, bool, u32)> = m
        .take_trace()
        .iter()
        .map(|a: &Access| (a.pc - BASE, a.addr - DEV, a.size, a.write, a.value))
        .collect();
    assert_eq!(
        &trace[..3],
        &[
            (0, 0x11, Byte, true, 0x11),
            (0, 0x12, Word, true, 0x2233),
            (0, 0x14, Byte, true, 0x44),
        ]
    );
    assert_eq!(trace.len(), log.len());
    // The same through a host-side access, which makes the access as the core would.
    m.device_mut::<Recorder>(id).log.clear();
    assert_eq!(m.access(DEV + 0x61, Long, None), Ok(0x0102_0304));
    assert_eq!(
        m.device::<Recorder>(id).log,
        [
            (0x61, Byte, false, 0x01),
            (0x62, Word, false, 0x0203),
            (0x64, Byte, false, 0x04)
        ]
    );
    // Aligned accesses are not split.
    m.device_mut::<Recorder>(id).log.clear();
    assert_eq!(m.access(DEV + 0x64, Long, None), Ok(0x0405_0607));
    assert_eq!(
        m.device::<Recorder>(id).log,
        [(0x64, Long, false, 0x0405_0607)]
    );
}

#[test]
fn misaligned_access_running_out_of_a_device_window_is_unmapped() {
    // Whether the access is mapped is decided before any piece is made: a misaligned long whose last pieces fall
    // outside the window is unmapped as a whole, at its own address, the device sees none of it and the
    // instruction is rolled back.
    let (mut m, l) = prepare("move.l (%a0)+,%d0\nend: nop", &["end"]);
    let id = m.add_device(DEV, 0x100, Box::new(Recorder::new()));
    m.cpu.a[0] = DEV + 0xFF;
    let r = m.run(until(l[0]));
    assert_eq!((r.kind, r.pc), (Stop::Unmapped, BASE), "{r:?}");
    assert!(r.detail.contains("0xec0000ff"), "{r:?}");
    assert_eq!(m.cpu.a[0], DEV + 0xFF);
    assert_eq!(m.device::<Recorder>(id).log, []);
}

#[test]
fn a_refused_piece_ends_a_misaligned_access() {
    // The pieces before the refused one were made; the instruction is rolled back.
    struct RefuseAt4(Vec<u32>);
    impl Device for RefuseAt4 {
        fn read(&mut self, _: u32, _: Size, _: &mut DeviceCx) -> u32 {
            0
        }
        fn write(&mut self, offset: u32, _: Size, _: u32, cx: &mut DeviceCx) {
            self.0.push(offset);
            if offset == 4 {
                cx.refuse(Stop::Unmodelled, "test block: offset 4 is read-only");
            }
        }
    }
    let (mut m, l) = prepare("move.l %d0,-(%a0)\nend: nop", &["end"]);
    let id = m.add_device(DEV, 0x100, Box::new(RefuseAt4(Vec::new())));
    m.cpu.a[0] = DEV + 5;
    let r = m.run(until(l[0]));
    assert_eq!((r.kind, r.pc), (Stop::Unmodelled, BASE), "{r:?}");
    assert_eq!(r.detail, "test block: offset 4 is read-only");
    assert_eq!(m.cpu.a[0], DEV + 5);
    assert_eq!(m.device::<RefuseAt4>(id).0, [1, 2, 4]);
}

/// Serves `code` (assembled at [`DEV`]) as read-only memory.
struct Rom(Vec<u8>);

impl Device for Rom {
    fn read(&mut self, offset: u32, size: Size, _: &mut DeviceCx) -> u32 {
        let o = offset as usize;
        self.0[o..o + size.bytes() as usize]
            .iter()
            .fold(0, |v, &b| v << 8 | b as u32)
    }
    fn write(&mut self, _: u32, _: Size, _: u32, _: &mut DeviceCx) {}
}

#[test]
fn fetches_from_a_device_window_are_not_traced() {
    // Instruction fetches from a device are neither traced nor counted by the poll detector: a loop running from
    // a device's window, with a poll limit of 3, runs to its instruction limit with an empty trace. The device's
    // operand reads are still both.
    let mut code = assemble("loop: addq.l #1,%d0\nbra.s loop\n", DEV);
    code.resize(0x100, 0);
    let mut m = Machine::new();
    m.add_device(DEV, 0x100, Box::new(Rom(code)));
    m.cpu.pc = DEV;
    m.set_poll_limit(3);
    m.trace(true);
    let r = m.run(Limit {
        max_instructions: Some(100),
        ..Limit::default()
    });
    assert_eq!(r.kind, Stop::InstructionLimit, "{r:?}");
    assert_eq!(m.cpu.d[0], 50);
    assert_eq!(m.take_trace(), []);

    // An operand read of the same window is traced.
    let (mut m, l) = prepare("move.w (%a0),%d1\nend: nop", &["end"]);
    m.add_device(DEV, 0x100, Box::new(Rom(vec![0x12, 0x34])));
    m.cpu.a[0] = DEV;
    m.trace(true);
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    let t = m.take_trace();
    assert_eq!(t.len(), 1);
    assert_eq!((t[0].addr, t[0].value, t[0].pc), (DEV, 0x1234, BASE));
}

/// An interrupt controller with 64 sources at one level, vector `64 + 64 * index + source`, that records the
/// requests the core takes from it.
struct AckIntc {
    index: usize,
    level: u8,
    acked: Vec<Irq>,
}

impl AckIntc {
    fn new(index: usize, level: u8) -> AckIntc {
        AckIntc {
            index,
            level,
            acked: Vec::new(),
        }
    }
}

impl Device for AckIntc {
    fn read(&mut self, _: u32, _: Size, _: &mut DeviceCx) -> u32 {
        0
    }
    fn write(&mut self, _: u32, _: Size, _: u32, _: &mut DeviceCx) {}
}

impl InterruptController for AckIntc {
    fn pending(&self, lines: &IrqLines, _: bool) -> Option<Irq> {
        let live = lines.asserted(self.index);
        (live != 0).then(|| Irq {
            level: self.level,
            vector: (64 + 64 * self.index + 63 - live.leading_zeros() as usize) as u8,
        })
    }
    fn level7(&self, _: &IrqLines) -> bool {
        false
    }
    fn acknowledge(&mut self, irq: Irq, cx: &mut DeviceCx) {
        self.acked.push(irq);
        // Negate the source taken, as a controller that clears its request on acknowledge would.
        cx.irq
            .set(self.index, irq.vector - 64 - 64 * self.index as u8, false);
    }
}

/// A timer with a period that asserts `asserts` (controller 0) at each event, and reports `lists` as the sources
/// it could assert.
struct Periodic {
    at: u64,
    period: u64,
    asserts: Option<u8>,
    lists: Vec<(usize, u8)>,
}

impl Device for Periodic {
    fn read(&mut self, _: u32, _: Size, _: &mut DeviceCx) -> u32 {
        0
    }
    fn write(&mut self, _: u32, _: Size, _: u32, _: &mut DeviceCx) {}
    fn next_event(&self) -> Option<u64> {
        Some(self.at)
    }
    fn advance(&mut self, now: u64, cx: &mut DeviceCx) {
        if let Some(s) = self.asserts {
            cx.irq.set(0, s, true);
        }
        self.at = now + self.period;
    }
    fn wake_sources(&self, out: &mut Vec<(usize, u8)>) -> bool {
        out.extend_from_slice(&self.lists);
        true
    }
}

/// `stop #0x2000` with an [`AckIntc`] (controller 0, level 3, every source unmasked) and `timer`; the handler for
/// source 6 ends the run. A time limit of 10 000 000 ticks bounds it.
fn stop_with(timer: Periodic) -> (Machine, u32) {
    let (mut m, l) = prepare("stop #0x2000\nend: nop\nhandler: nop", &["handler"]);
    set_vector(&mut m, 70, l[0]);
    m.add_interrupt_controller(INTC, 0x4000, Box::new(AckIntc::new(0, 3)));
    m.add_timer(Box::new(timer));
    m.set_time_limit(Some(10_000_000));
    (m, l[0])
}

#[test]
fn stop_with_a_timer_that_cannot_interrupt_halts_forever() {
    // The timer keeps scheduling events but says it asserts nothing: with the lines as they are, no request
    // can wake the core, so it halts at once instead of stepping from event to event to the time limit.
    let (mut m, handler) = stop_with(Periodic {
        at: 100,
        period: 100,
        asserts: None,
        lists: Vec::new(),
    });
    let r = m.run(until(handler));
    assert_eq!(r.kind, Stop::HaltedForever, "{r:?}");
    assert_eq!(m.now(), 1, "no event is stepped through");
}

#[test]
fn stop_with_a_timer_that_can_interrupt_wakes() {
    let (mut m, handler) = stop_with(Periodic {
        at: 1000,
        period: 1000,
        asserts: Some(6),
        lists: vec![(0, 6)],
    });
    let r = m.run(until(handler));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.now(), 1000, "woken at the timer's event");
}

#[test]
fn stop_with_listed_sources_held_off_by_the_mask_halts_forever() {
    // The listed sources add to the lines as they are: a source asserted but at a level the mask holds off
    // cannot wake, whatever the timer lists.
    let (mut m, l) = prepare("stop #0x2300\nend: nop", &["end"]);
    m.add_interrupt_controller(INTC, 0x4000, Box::new(AckIntc::new(0, 3)));
    m.add_timer(Box::new(Periodic {
        at: 100,
        period: 100,
        asserts: None,
        lists: vec![(0, 6)],
    }));
    m.mem.irq_lines_mut().set(0, 5, true);
    m.set_time_limit(Some(10_000_000));
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::HaltedForever, "{r:?}");
}

#[test]
fn acknowledge_is_called_on_the_taking_controller() {
    // Controller 1's level-5 request beats controller 0's level-3 one; only controller 1 is acknowledged, with
    // the request taken. Its acknowledge negates the source, so the next request taken is controller 0's.
    let src = "move.w #0x2000,%sr\nnop\nend: nop\nh0: rte\nh1: rte";
    let (mut m, l) = prepare(src, &["end", "h0", "h1"]);
    set_vector(&mut m, 64 + 2, l[1]);
    set_vector(&mut m, 128 + 9, l[2]);
    let c0 = m.add_interrupt_controller(INTC, 0x4000, Box::new(AckIntc::new(0, 3)));
    let c1 = m.add_interrupt_controller(INTC + 0x4000, 0x4000, Box::new(AckIntc::new(1, 5)));
    m.mem.irq_lines_mut().set(0, 2, true);
    m.mem.irq_lines_mut().set(1, 9, true);
    let r = m.run(until(l[2]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(
        m.device::<AckIntc>(c1).acked,
        [Irq {
            level: 5,
            vector: 128 + 9
        }]
    );
    assert_eq!(m.device::<AckIntc>(c0).acked, []);
    assert_eq!(
        m.mem.irq_lines().asserted(1) >> 9 & 1,
        0,
        "negated by the acknowledge"
    );
    let r = m.run(until(l[1]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(
        m.device::<AckIntc>(c0).acked,
        [Irq {
            level: 3,
            vector: 64 + 2
        }]
    );
    assert_eq!(m.device::<AckIntc>(c1).acked.len(), 1);
}
