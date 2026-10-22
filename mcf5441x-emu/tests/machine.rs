//! Devices on the bus: MMIO routing and tracing, the poll detector, device halts and the time limit.

mod common;

use common::{BASE, prepare, until};
use mcf5441x_emu::{Access, AccessKind, Device, DeviceCx, Limit, Size, Stop};

/// A register block: reads return `regs[offset / 4]` (truncated to the access), writes are recorded.
#[derive(Default)]
struct Stub {
    regs: [u32; 16],
    writes: Vec<(u32, Size, u32)>,
    reads: u32,
}

impl Device for Stub {
    fn read(&mut self, offset: u32, _: Size, _: &mut DeviceCx) -> u32 {
        self.reads += 1;
        self.regs[(offset / 4) as usize % 16]
    }
    fn write(&mut self, offset: u32, size: Size, value: u32, _: &mut DeviceCx) {
        self.writes.push((offset, size, value));
    }
}

/// A console that stops the run when a byte arrives.
#[derive(Default)]
struct Console {
    seen: Vec<u8>,
}

impl Device for Console {
    fn read(&mut self, _: u32, _: Size, _: &mut DeviceCx) -> u32 {
        0
    }
    fn write(&mut self, _: u32, _: Size, value: u32, cx: &mut DeviceCx) {
        self.seen.push(value as u8);
        cx.halt(
            Stop::Halt("console"),
            format!("line {}", value as u8 as char),
        );
    }
}

#[test]
fn mmio_reads_and_writes_are_traced() {
    let src = "move.l #0x12345678,%d0\nmove.l %d0,0xFC000010\nmove.l 0xFC000014,%d1\nmove.l %d1,(%a1)\n\
               move.b 0xFC000014,%d2\nend:";
    let (mut m, l) = prepare(src, &["end"]);
    m.cpu.a[1] = common::MEM;
    let mut stub = Stub::default();
    stub.regs[5] = 0xCAFE_BABE;
    let id = m.add_device(0xFC00_0000, 0x1000, Box::new(stub));
    m.trace(true);
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.cpu.d[1], 0xCAFE_BABE);
    assert_eq!(
        m.cpu.d[2], 0xBE,
        "a byte read sees the value truncated to the access"
    );
    // RAM accesses (the store through A1, the fetches) are not traced; the tick is the clock during the access.
    let t = m.take_trace();
    assert_eq!(
        t,
        [
            Access {
                t: 1,
                pc: BASE + 6,
                addr: 0xFC00_0010,
                size: Size::Long,
                write: true,
                value: 0x1234_5678
            },
            Access {
                t: 2,
                pc: BASE + 12,
                addr: 0xFC00_0014,
                size: Size::Long,
                write: false,
                value: 0xCAFE_BABE
            },
            Access {
                t: 4,
                pc: BASE + 20,
                addr: 0xFC00_0014,
                size: Size::Byte,
                write: false,
                value: 0xBE
            },
        ]
    );
    assert!(m.take_trace().is_empty(), "taking the trace empties it");
    assert_eq!(
        m.device::<Stub>(id).writes,
        [(0x10, Size::Long, 0x1234_5678)]
    );
    assert_eq!(m.device::<Stub>(id).reads, 2);

    // With tracing off nothing is recorded.
    m.trace(false);
    m.cpu.pc = BASE;
    m.run(until(l[0]));
    assert!(m.take_trace().is_empty());
    m.device_mut::<Stub>(id).reads = 0;
    assert_eq!(m.device::<Stub>(id).reads, 0);
}

#[test]
fn poll_loop_stops_with_report() {
    let src = "wait: move.l 0xFC000020,%d0\nbtst #0,%d0\nbeq.s wait\nnop\nend:";
    let (mut m, l) = prepare(src, &["end"]);
    m.add_device(0xFC00_0000, 0x1000, Box::new(Stub::default()));
    m.set_poll_limit(1000);
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::Poll, "{r:?}");
    assert_eq!(r.detail, "spinning at 0x40100000 reading 0xfc000020");
    // The read that crossed the limit completed; the run stops at the instruction after it.
    assert_eq!(r.pc, BASE + 6);
    assert_eq!(m.instructions(), 1000 * 3 + 1, "1001 reads");
}

#[test]
fn mmio_write_between_reads_is_not_a_poll() {
    let src = "move.l #3000,%d1\nwait: move.l 0xFC000020,%d0\nmove.l %d0,0xFC000024\nsubq.l #1,%d1\nbne.s wait\nend:";
    let (mut m, l) = prepare(src, &["end"]);
    m.add_device(0xFC00_0000, 0x1000, Box::new(Stub::default()));
    m.set_poll_limit(1000);
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
}

#[test]
fn long_ram_loop_after_one_mmio_read_is_not_a_poll() {
    let src = "move.l 0xFC000020,%d0\nmove.l #20000,%d1\nloop: move.l %d1,(%a1)\nsubq.l #1,%d1\nbne.s loop\nend:";
    let (mut m, l) = prepare(src, &["end"]);
    m.cpu.a[1] = common::MEM;
    m.add_device(0xFC00_0000, 0x1000, Box::new(Stub::default()));
    m.set_poll_limit(1000);
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.instructions(), 2 + 3 * 20000);
}

#[test]
fn unmapped_access_stops() {
    let src = "move.l #0x12345678,%d0\nmove.l %d0,0xFC000010\nend:";
    let (mut m, l) = prepare(src, &["end"]);
    let r = m.run(until(l[0]));
    assert_eq!((r.kind, r.pc), (Stop::Unmapped, BASE + 6), "{r:?}");
    assert!(r.detail.contains("0xfc000010"), "{r:?}");

    // An access that runs past the end of a device is unmapped too, and the device does not see it.
    let (mut m, l) = prepare("move.l 0xFC000FFE,%d0\nend:", &["end"]);
    let id = m.add_device(0xFC00_0000, 0x1000, Box::new(Stub::default()));
    let r = m.run(until(l[0]));
    assert_eq!((r.kind, r.pc), (Stop::Unmapped, BASE), "{r:?}");
    assert!(r.detail.contains("0xfc000ffe"), "{r:?}");
    assert_eq!(m.device::<Stub>(id).reads, 0);
}

#[test]
fn device_halt_stops_the_run() {
    // The access that asks to halt completes, and so does its instruction: the run stops before the next one.
    let src = "moveq #0x41,%d0\nmove.b %d0,0xEC000000\nmoveq #5,%d2\nend:";
    let (mut m, l) = prepare(src, &["end"]);
    let id = m.add_device(0xEC00_0000, 0x100, Box::new(Console::default()));
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::Halt("console"), "{r:?}");
    assert_eq!(r.detail, "line A");
    assert_eq!(r.pc, BASE + 8);
    assert_eq!(m.cpu.pc, BASE + 8);
    assert_eq!(m.instructions(), 2);
    assert_eq!(m.cpu.d[2], 0);
    assert_eq!(m.device::<Console>(id).seen, b"A");
    // The next run resumes after it.
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.cpu.d[2], 5);
}

#[test]
fn time_limit_stops_exactly() {
    let (mut m, l) = prepare("loop: addq.l #1,%d0\nbra.s loop", &["loop"]);
    m.set_time_limit(Some(101));
    let r = m.run(Limit::default());
    assert_eq!((r.kind, r.pc), (Stop::TimeLimit, l[0] + 2), "{r:?}");
    assert_eq!((m.now(), m.instructions()), (101, 101));
    assert_eq!(m.cpu.d[0], 51);
    // The limit is on the clock, not per run: a run at the limit stops at once, a later limit continues.
    let r = m.run(Limit::default());
    assert_eq!((r.kind, m.now()), (Stop::TimeLimit, 101), "{r:?}");
    m.set_time_limit(Some(104));
    let r = m.run(Limit::default());
    assert_eq!(
        (r.kind, m.now(), m.cpu.d[0]),
        (Stop::TimeLimit, 104, 52),
        "{r:?}"
    );
    // The instruction limit still counts this run's instructions.
    m.set_time_limit(None);
    let r = m.run(Limit {
        max_instructions: Some(7),
        ..Limit::default()
    });
    assert_eq!((r.kind, m.now()), (Stop::InstructionLimit, 111), "{r:?}");
}

#[test]
#[should_panic]
fn device_of_the_wrong_type_panics() {
    let (mut m, _) = prepare("nop", &[]);
    let id = m.add_device(0xFC00_0000, 0x1000, Box::new(Stub::default()));
    m.device::<Console>(id);
}

#[test]
#[should_panic]
fn overlapping_devices_panic() {
    let (mut m, _) = prepare("nop", &[]);
    m.add_device(0xFC00_0000, 0x1000, Box::new(Stub::default()));
    m.add_device(0xFC00_0800, 0x1000, Box::new(Stub::default()));
}

/// A run to `end` with an access breakpoint.
fn watching(end: u32, base: u32, len: u32, kind: AccessKind) -> Limit {
    Limit {
        until_pc: Some(end),
        until_access: Some((base, len, kind)),
        ..Limit::default()
    }
}

#[test]
fn until_access_stops_before_the_first_write_to_a_device_range() {
    // A write outside the range and a read inside it (the watch is on writes) go through; the long write that
    // overlaps the range's first bytes is not made, and the run stops on it with the registers as before it.
    let src = "move.l #0x55,%d0\nmove.l %d0,0xFC000000\nmove.l 0xFC000014,%d1\n\
               at: move.l %d0,-(%a0)\nmove.l %d1,0xFC000020\nend: nop";
    let (mut m, l) = prepare(src, &["at", "end"]);
    m.cpu.a[0] = 0xFC00_0014;
    let mut stub = Stub::default();
    stub.regs[5] = 0x1234;
    let id = m.add_device(0xFC00_0000, 0x1000, Box::new(stub));
    let limit = watching(l[1], 0xFC00_0012, 0x10, AccessKind::Write);
    let r = m.run(limit);
    assert_eq!(
        (r.kind, r.pc, m.cpu.pc),
        (Stop::UntilAccess, l[0], l[0]),
        "{r:?}"
    );
    assert!(r.detail.contains("0xfc000010"), "{r:?}");
    assert_eq!(m.cpu.a[0], 0xFC00_0014, "the predecrement is undone");
    assert_eq!(m.cpu.d[1], 0x1234);
    let stub = m.device::<Stub>(id);
    assert_eq!(stub.writes, [(0, Size::Long, 0x55)]);
    assert_eq!(stub.reads, 1);
    // The same limit stops there again at once; without it the write is made.
    let r = m.run(limit);
    assert_eq!((r.kind, r.pc), (Stop::UntilAccess, l[0]), "{r:?}");
    assert_eq!(m.device::<Stub>(id).writes.len(), 1);
    let r = m.run(until(l[1]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(
        m.device::<Stub>(id).writes,
        [
            (0, Size::Long, 0x55),
            (0x10, Size::Long, 0x55),
            (0x20, Size::Long, 0x1234)
        ]
    );
}

#[test]
fn until_access_on_ram() {
    // Watched RAM is RAM to every access the watch lets through: the bytes beside it, a read of it while writes
    // are watched, and the host. Cached and uncached runs stop alike.
    let w = common::MEM + 0x1000;
    let src = "move.l #0x77,%d0\nmove.l %d0,-4(%a1)\nmove.l (%a1),%d1\nlea 4(%a1),%a0\n\
               at: move.l %d0,-(%a0)\nend: nop";
    for cached in [true, false] {
        let (mut m, l) = prepare(src, &["at", "end"]);
        m.cpu.a[1] = w;
        m.mem.load(w, &[0xA5; 4]).unwrap();
        let limit = watching(l[1], w, 4, AccessKind::Write);
        let r = if cached {
            m.run(limit)
        } else {
            m.run_uncached(limit)
        };
        assert_eq!((r.kind, r.pc), (Stop::UntilAccess, l[0]), "{r:?}");
        assert!(r.detail.contains(&format!("{w:#010x}")), "{r:?}");
        assert_eq!(m.cpu.a[0], w + 4, "the predecrement is undone");
        assert_eq!(m.cpu.d[1], 0xA5A5_A5A5, "the read went through");
        assert_eq!(
            m.mem.peek(w - 4, 8).unwrap(),
            [0, 0, 0, 0x77, 0xA5, 0xA5, 0xA5, 0xA5]
        );
        // The host's accesses are not watched.
        assert_eq!(m.access(w, Size::Long, None), Ok(0xA5A5_A5A5));
        // A read watch lets the write through.
        let r = m.run(watching(l[1], w, 4, AccessKind::Read));
        assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
        assert_eq!(m.mem.peek(w, 4).unwrap(), [0, 0, 0, 0x77]);
    }
    // A watch on the last byte of a read stops it; one on any access stops a byte write.
    let (mut m, l) = prepare(
        "at: move.l (%a1),%d1\nmove.b %d1,7(%a1)\nend: nop",
        &["at", "end"],
    );
    m.cpu.a[1] = w;
    let r = m.run(watching(l[1], w + 3, 1, AccessKind::Read));
    assert_eq!((r.kind, r.pc), (Stop::UntilAccess, l[0]), "{r:?}");
    let r = m.run(watching(l[1], w + 7, 1, AccessKind::Any));
    assert_eq!((r.kind, r.pc), (Stop::UntilAccess, l[0] + 2), "{r:?}");
    // An empty range watches nothing.
    let r = m.run(watching(l[1], w + 7, 0, AccessKind::Any));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
}

#[test]
fn the_access_breakpoint_ends_with_its_run() {
    // After a watched run, whether it stopped at the watch or not, the core's accesses between runs and the next run
    // without a watch see the watched bytes as RAM.
    let w = common::MEM + 0x1000;
    let (mut m, l) = prepare("at: move.l %d0,(%a1)\nend: nop", &["at", "end"]);
    m.cpu.a[1] = w;
    m.cpu.d[0] = 0x1234_5678;
    let r = m.run(watching(l[0], w, 4, AccessKind::Any));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.mem.write(w, Size::Long, 0xA5A5_A5A5), Ok(()));
    assert_eq!(m.mem.read(w, Size::Long), Ok(0xA5A5_A5A5));
    let r = m.run(watching(l[1], w, 4, AccessKind::Any));
    assert_eq!((r.kind, r.pc), (Stop::UntilAccess, l[0]), "{r:?}");
    assert_eq!(m.mem.read(w, Size::Long), Ok(0xA5A5_A5A5));
    assert_eq!(m.mem.write(w + 2, Size::Word, 0x0101), Ok(()));
    let r = m.run(until(l[1]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.mem.read(w, Size::Long), Ok(0x1234_5678));
}
