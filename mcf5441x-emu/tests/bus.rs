//! Bus masters, DMA request lines and host-side access: a master's view of the address space, servicing at the
//! instruction boundary, and the accesses, clock moves and device calls a host makes between runs.

mod common;

use std::collections::VecDeque;

use common::{BASE, prepare, until};
use mcf5441x_emu::{Access, Device, DeviceCx, Fault, Limit, Machine, Size, Stop};

const FIFO: u32 = 0xFC00_0000;
const MOVER: u32 = 0xFC01_0000;
/// The DMA request line the FIFO drives.
const LINE: u8 = 5;

/// A FIFO of longs: a read of +0 pops one (0 when empty), a write of +4 pushes one, +8 reads the count. DMA line
/// [`LINE`] is asserted while it holds anything.
#[derive(Default)]
struct Fifo {
    q: VecDeque<u32>,
}

impl Fifo {
    fn drive(&self, cx: &mut DeviceCx) {
        cx.dma.set(LINE, !self.q.is_empty());
    }
}

impl Device for Fifo {
    fn read(&mut self, offset: u32, _: Size, cx: &mut DeviceCx) -> u32 {
        let v = match offset {
            0 => self.q.pop_front().unwrap_or(0),
            _ => self.q.len() as u32,
        };
        self.drive(cx);
        v
    }
    fn write(&mut self, offset: u32, _: Size, value: u32, cx: &mut DeviceCx) {
        if offset == 4 {
            self.q.push_back(value);
        }
        self.drive(cx);
    }
}

/// A bus master: while line [`LINE`] is asserted, copies one long from the FIFO's +0 to `dst`, advancing `dst`.
/// A write to its +0 writes two NOPs at the address written, with [`Bus::write_bytes`]; a write to +4 reads its
/// own +0 through the bus and keeps the result.
struct Mover {
    dst: u32,
    moved: u32,
    own: Vec<Result<u32, Fault>>,
}

impl Mover {
    fn new(dst: u32) -> Mover {
        Mover {
            dst,
            moved: 0,
            own: Vec::new(),
        }
    }
}

impl Device for Mover {
    fn read(&mut self, _: u32, _: Size, _: &mut DeviceCx) -> u32 {
        self.moved
    }
    fn write(&mut self, offset: u32, _: Size, value: u32, cx: &mut DeviceCx) {
        let mut bus = cx.bus().expect("a bus master has a bus");
        match offset {
            0 => bus.write_bytes(value, &[0x4e, 0x71, 0x4e, 0x71]).unwrap(),
            _ => {
                let r = bus.read(MOVER, Size::Long);
                self.own.push(r);
                let w = bus.write(MOVER + 8, Size::Word, 1).map(|()| 0);
                self.own.push(w);
                let b = bus.read_bytes(MOVER, &mut [0; 4]).map(|()| 0);
                self.own.push(b);
            }
        }
    }
    fn service(&mut self, cx: &mut DeviceCx) {
        while cx.dma.asserted() & 1 << LINE != 0 {
            assert!(self.moved < 100, "the request never dropped");
            let mut bus = cx.bus().expect("a bus master has a bus");
            let v = bus.read(FIFO, Size::Long).unwrap();
            bus.write(self.dst, Size::Long, v).unwrap();
            self.dst += 4;
            self.moved += 1;
        }
    }
}

/// A machine with RAM at `0x40100000..0x40200000`, the FIFO and a mover copying to `dst`.
fn fifo_and_mover(dst: u32) -> (Machine, mcf5441x_emu::DeviceId, mcf5441x_emu::DeviceId) {
    let mut m = Machine::new();
    m.mem.map_ram(0x4010_0000, 0x10_0000);
    let fifo = m.add_device(FIFO, 0x100, Box::new(Fifo::default()));
    let mover = m.add_bus_master(MOVER, 0x100, Box::new(Mover::new(dst)));
    (m, fifo, mover)
}

#[test]
fn bus_master_moves_words_until_the_request_drops() {
    let (mut m, fifo, mover) = fifo_and_mover(0x4010_0000);
    m.with_device::<Fifo, _>(fifo, |f, cx| {
        f.q.extend([0x1111_1111, 0x2222_2222, 0x3333_3333]);
        f.drive(cx);
        assert_eq!(cx.dma.asserted(), 1 << LINE);
    });
    assert_eq!(
        m.mem.peek(0x4010_0000, 12).unwrap(),
        [
            0x11, 0x11, 0x11, 0x11, 0x22, 0x22, 0x22, 0x22, 0x33, 0x33, 0x33, 0x33
        ]
    );
    assert_eq!(m.dma_lines().asserted(), 0, "the request dropped");
    assert_eq!(m.device::<Mover>(mover).moved, 3);
    assert!(m.device::<Fifo>(fifo).q.is_empty());
}

#[test]
fn dma_is_serviced_before_the_next_instruction() {
    let src = "move.l #0xCAFEF00D,%d1\nmove.l %d1,0xFC000004\nmove.l 0x40300000,%d0\nend:";
    let (mut m, l) = prepare(src, &["end"]);
    m.add_device(FIFO, 0x100, Box::new(Fifo::default()));
    m.add_bus_master(MOVER, 0x100, Box::new(Mover::new(0x4030_0000)));
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.cpu.d[0], 0xCAFE_F00D);
    assert_eq!(m.instructions(), 3);
    assert_eq!(m.dma_lines().asserted(), 0);
}

/// A bus master that reads the FIFO's count ten times each time it is serviced.
#[derive(Default)]
struct Poller {
    reads: u32,
}

impl Device for Poller {
    fn read(&mut self, _: u32, _: Size, _: &mut DeviceCx) -> u32 {
        0
    }
    fn write(&mut self, _: u32, _: Size, _: u32, _: &mut DeviceCx) {}
    fn service(&mut self, cx: &mut DeviceCx) {
        let mut bus = cx.bus().unwrap();
        for _ in 0..10 {
            bus.read(FIFO + 8, Size::Long).unwrap();
            self.reads += 1;
        }
    }
}

#[test]
fn bus_accesses_are_not_traced_or_poll_counted() {
    let src = "move.l #0x12345678,%d1\nmove.l %d1,0xFC000004\nmove.l #0x9ABCDEF0,%d1\nmove.l %d1,0xFC000004\n\
               move.l 0x40300004,%d0\nend:";
    let (mut m, l) = prepare(src, &["end"]);
    m.add_device(FIFO, 0x100, Box::new(Fifo::default()));
    m.add_bus_master(MOVER, 0x100, Box::new(Mover::new(0x4030_0000)));
    let poller = m.add_bus_master(0xFC02_0000, 0x100, Box::new(Poller::default()));
    m.set_poll_limit(2);
    m.trace(true);
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.cpu.d[0], 0x9ABC_DEF0);
    // Only the CPU's own accesses: the mover's reads of the FIFO and the poller's are not recorded.
    let t = m.take_trace();
    assert_eq!(
        t,
        [
            Access {
                t: 1,
                pc: BASE + 6,
                addr: FIFO + 4,
                size: Size::Long,
                write: true,
                value: 0x1234_5678
            },
            Access {
                t: 3,
                pc: BASE + 18,
                addr: FIFO + 4,
                size: Size::Long,
                write: true,
                value: 0x9ABC_DEF0
            },
        ]
    );
    assert!(
        m.device::<Poller>(poller).reads >= 30,
        "serviced at each boundary"
    );
}

#[test]
fn reads_of_a_device_that_is_not_poll_counted_never_stop_the_run() {
    struct Clock;
    impl Device for Clock {
        fn read(&mut self, _: u32, _: Size, cx: &mut DeviceCx) -> u32 {
            cx.now as u32
        }
        fn write(&mut self, _: u32, _: Size, _: u32, _: &mut DeviceCx) {}
        fn poll_counted(&self) -> bool {
            false
        }
    }
    let src = "move.l #100,%d1\nwait: move.l 0xFC000000,%d0\nsubq.l #1,%d1\nbne.s wait\nend:";
    let (mut m, l) = prepare(src, &["end"]);
    m.add_device(FIFO, 0x100, Box::new(Clock));
    m.set_poll_limit(2);
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
}

#[test]
fn bus_writes_invalidate_cached_code() {
    // A register write to a master (an access in an instruction) patches the subroutine after it ran once.
    let src = "bsr.w sub\n\
               lea sub,%a3\nmove.l %a3,0xFC010000\n\
               bsr.w sub\n\
               bra.w end\n\
               sub: moveq #1,%d0\nadd.l %d0,%d1\nrts\nend: nop";
    let (mut m, l) = prepare(src, &["end", "sub"]);
    m.add_bus_master(MOVER, 0x100, Box::new(Mover::new(0)));
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    // The second call runs `nop; nop; rts`.
    assert_eq!(m.cpu.d[1], 1);
    assert_eq!(m.mem.peek(l[1], 4).unwrap(), [0x4e, 0x71, 0x4e, 0x71]);

    // DMA serviced at the boundary writes over the subroutine: the next call runs the new code.
    let src = "bsr.w sub\n\
               move.l #0x4e714e71,%d2\nmove.l %d2,0xFC000004\n\
               bsr.w sub\n\
               bra.w end\n\
               sub: moveq #1,%d0\nadd.l %d0,%d1\nrts\nend: nop";
    let (mut m, l) = prepare(src, &["end", "sub"]);
    m.add_device(FIFO, 0x100, Box::new(Fifo::default()));
    m.add_bus_master(MOVER, 0x100, Box::new(Mover::new(l[1])));
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.cpu.d[1], 1);

    // Between runs, from the host: a loop runs (and is cached), then the master overwrites it with NOPs.
    let src = "loop: addq.l #1,%d1\nsubq.l #1,%d2\nbne.s loop\nend: nop";
    let (mut m, l) = prepare(src, &["end"]);
    let mover = m.add_bus_master(MOVER, 0x100, Box::new(Mover::new(0)));
    m.cpu.d[2] = 3;
    let r = m.run(until(l[0]));
    assert_eq!((r.kind, m.cpu.d[1]), (Stop::UntilPc, 3), "{r:?}");
    m.with_device::<Mover, _>(mover, |_, cx| {
        cx.bus().unwrap().write_bytes(BASE, &[0x4e, 0x71]).unwrap();
    });
    m.cpu.pc = BASE;
    m.cpu.d[2] = 3;
    let r = m.run(until(l[0]));
    assert_eq!((r.kind, m.cpu.d[1]), (Stop::UntilPc, 3), "{r:?}");
}

#[test]
fn master_access_to_its_own_window_is_unmapped() {
    let (mut m, _, mover) = fifo_and_mover(0x4010_0000);
    m.access(MOVER + 4, Size::Long, Some(0)).unwrap();
    assert_eq!(
        m.device::<Mover>(mover).own,
        [
            Err(Fault::Unmapped {
                addr: MOVER,
                size: Size::Long
            }),
            Err(Fault::Unmapped {
                addr: MOVER + 8,
                size: Size::Word
            }),
            Err(Fault::Unmapped {
                addr: MOVER,
                size: Size::Byte
            }),
        ]
    );
    // Byte copies reach RAM only.
    m.with_device::<Mover, _>(mover, |_, cx| {
        let mut bus = cx.bus().unwrap();
        assert_eq!(
            bus.read_bytes(FIFO, &mut [0; 4]),
            Err(Fault::Unmapped {
                addr: FIFO,
                size: Size::Byte
            })
        );
        assert_eq!(
            bus.write_bytes(0x401F_FFFE, &[1, 2, 3, 4]),
            Err(Fault::Unmapped {
                addr: 0x401F_FFFE,
                size: Size::Byte
            })
        );
        bus.write_bytes(0x4010_0010, &[1, 2, 3, 4]).unwrap();
        let mut out = [0; 4];
        bus.read_bytes(0x4010_0010, &mut out).unwrap();
        assert_eq!(out, [1, 2, 3, 4]);
        assert_eq!(bus.read(0x4010_0010, Size::Word), Ok(0x0102));
        assert_eq!(
            bus.read(0xFD00_0000, Size::Long),
            Err(Fault::Unmapped {
                addr: 0xFD00_0000,
                size: Size::Long
            })
        );
    });
}

/// A bus master that flips DMA line 9 each time it is serviced.
struct Toggler;

impl Device for Toggler {
    fn read(&mut self, _: u32, _: Size, _: &mut DeviceCx) -> u32 {
        0
    }
    fn write(&mut self, _: u32, _: Size, _: u32, _: &mut DeviceCx) {}
    fn service(&mut self, cx: &mut DeviceCx) {
        let on = cx.dma.asserted() & 1 << 9 == 0;
        cx.dma.set(9, on);
    }
}

#[test]
fn requests_that_never_settle_stop_the_run() {
    let (mut m, l) = prepare("nop\nend:", &["end"]);
    m.add_bus_master(MOVER, 0x100, Box::new(Toggler));
    let r = m.run(until(l[0]));
    assert_eq!((r.kind, r.pc), (Stop::Unmodelled, BASE), "{r:?}");
    assert!(r.detail.contains("never settle"), "{r:?}");
    assert_eq!(m.instructions(), 0);
}

#[test]
fn host_access_reads_and_writes_like_the_cpu() {
    let (mut m, fifo, _) = fifo_and_mover(0x4010_0100);
    m.cpu.pc = 0x4010_0000;
    m.trace(true);
    assert_eq!(m.access(0x4010_0000, Size::Long, Some(0x1234_5678)), Ok(0));
    assert_eq!(m.access(0x4010_0002, Size::Word, None), Ok(0x5678));
    assert_eq!(m.access(0x4010_0000, Size::Byte, None), Ok(0x12));
    // A device access, then the mover services the request it raised.
    assert_eq!(m.access(FIFO + 4, Size::Long, Some(0xAABB_CCDD)), Ok(0));
    assert_eq!(
        m.mem.peek(0x4010_0100, 4).unwrap(),
        [0xAA, 0xBB, 0xCC, 0xDD]
    );
    assert!(m.device::<Fifo>(fifo).q.is_empty());
    assert_eq!(m.access(FIFO + 8, Size::Long, None), Ok(0));
    assert_eq!(
        m.access(0xFD00_0000, Size::Long, None),
        Err(Fault::Unmapped {
            addr: 0xFD00_0000,
            size: Size::Long
        })
    );
    // Device accesses are traced as the CPU's would be at the current tick; RAM accesses are not.
    let at = |addr, write, value| Access {
        t: 0,
        pc: 0x4010_0000,
        addr,
        size: Size::Long,
        write,
        value,
    };
    assert_eq!(
        m.take_trace(),
        [at(FIFO + 4, true, 0xAABB_CCDD), at(FIFO + 8, false, 0)]
    );
}

/// Records the ticks it was advanced at; its events are every 50 ticks.
#[derive(Default)]
struct Ticker {
    at: Vec<u64>,
}

impl Device for Ticker {
    fn read(&mut self, _: u32, _: Size, _: &mut DeviceCx) -> u32 {
        0
    }
    fn write(&mut self, _: u32, _: Size, _: u32, _: &mut DeviceCx) {}
    fn next_event(&self) -> Option<u64> {
        Some(50 * (self.at.len() as u64 + 1))
    }
    fn advance(&mut self, now: u64, cx: &mut DeviceCx) {
        self.at.push(now);
        let _ = cx;
    }
}

#[test]
fn idle_until_advances_devices_without_instructions() {
    let mut m = Machine::new();
    m.mem.map_ram(0x4010_0000, 0x10_0000);
    let t = m.add_timer(Box::new(Ticker::default()));
    assert_eq!(m.idle_until(120), None);
    assert_eq!(m.device::<Ticker>(t).at, [50, 100]);
    assert_eq!((m.now(), m.instructions()), (120, 0));
    assert_eq!(m.idle_until(150), None);
    assert_eq!(m.device::<Ticker>(t).at, [50, 100, 150]);
    assert_eq!(m.now(), 150);
    assert_eq!(m.cpu.pc, 0);

    // Bus masters are serviced after each event: here the request comes from a timer at tick 5.
    struct Request {
        done: bool,
    }
    impl Device for Request {
        fn read(&mut self, _: u32, _: Size, _: &mut DeviceCx) -> u32 {
            0
        }
        fn write(&mut self, _: u32, _: Size, _: u32, _: &mut DeviceCx) {}
        fn next_event(&self) -> Option<u64> {
            (!self.done).then_some(5)
        }
        fn advance(&mut self, _: u64, cx: &mut DeviceCx) {
            self.done = true;
            cx.dma.set(LINE, true);
        }
    }
    let (mut m, fifo, mover) = fifo_and_mover(0x4010_0000);
    m.add_timer(Box::new(Request { done: false }));
    m.with_device::<Fifo, _>(fifo, |f, _| f.q.push_back(7));
    assert_eq!(m.device::<Mover>(mover).moved, 0, "no request yet");
    assert_eq!(m.idle_until(10), None);
    assert_eq!(m.mem.peek(0x4010_0000, 4).unwrap(), [0, 0, 0, 7]);
    assert_eq!(m.device::<Mover>(mover).moved, 1);
    assert_eq!(m.dma_lines().asserted(), 0);
    assert_eq!((m.now(), m.instructions()), (10, 0));
}

#[test]
fn with_device_gets_a_bus_for_masters_only() {
    let (mut m, fifo, mover) = fifo_and_mover(0x4010_0000);
    let plain = m.with_device::<Fifo, _>(fifo, |_, cx| cx.bus().is_none());
    assert!(plain);
    let master = m.with_device::<Mover, _>(mover, |_, cx| {
        let mut bus = cx.bus().unwrap();
        bus.write(FIFO + 4, Size::Long, 0x55).unwrap();
        bus.read(FIFO + 8, Size::Long).unwrap()
    });
    assert_eq!(master, 1, "the FIFO saw the master's write");
    // Then the master was serviced: the request the write raised moved the long.
    assert_eq!(m.mem.peek(0x4010_0000, 4).unwrap(), [0, 0, 0, 0x55]);
    assert_eq!(m.dma_lines().asserted(), 0);
}

#[test]
#[should_panic]
fn dma_line_out_of_range_panics() {
    let (mut m, fifo, _) = fifo_and_mover(0x4010_0000);
    m.with_device::<Fifo, _>(fifo, |_, cx| cx.dma.set(64, true));
}

/// Halts the run when advanced at `at`, then schedules nothing; or, with `stuck`, keeps its event at `at`.
struct Alarm {
    at: u64,
    fired: bool,
    stuck: bool,
}

impl Device for Alarm {
    fn read(&mut self, _: u32, _: Size, _: &mut DeviceCx) -> u32 {
        0
    }
    fn write(&mut self, _: u32, _: Size, _: u32, _: &mut DeviceCx) {}
    fn next_event(&self) -> Option<u64> {
        (self.stuck || !self.fired).then_some(self.at)
    }
    fn advance(&mut self, now: u64, cx: &mut DeviceCx) {
        if !self.stuck {
            self.fired = true;
            cx.halt(Stop::Halt("alarm"), format!("at {now}"));
        }
    }
}

#[test]
fn a_halt_from_with_device_is_reported_by_the_next_run() {
    let (mut m, l) = prepare("moveq #1,%d0\nend:", &["end"]);
    let fifo = m.add_device(FIFO, 0x100, Box::new(Fifo::default()));
    m.with_device::<Fifo, _>(fifo, |_, cx| cx.halt(Stop::Halt("host"), "from the host"));
    // Idling does not report it: it stays pending for the run.
    assert_eq!(m.idle_until(10), None);
    assert_eq!(m.now(), 10);
    let r = m.run(until(l[0]));
    assert_eq!((r.kind, r.pc), (Stop::Halt("host"), BASE), "{r:?}");
    assert_eq!(r.detail, "from the host");
    assert_eq!((m.cpu.pc, m.instructions(), m.cpu.d[0]), (BASE, 0, 0));
    // Reported once.
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
}

#[test]
fn idle_until_stops_at_a_device_halt_and_returns_it() {
    let (mut m, _) = prepare("nop", &[]);
    m.add_timer(Box::new(Alarm {
        at: 30,
        fired: false,
        stuck: false,
    }));
    let r = m.idle_until(100).expect("halted");
    assert_eq!((r.kind, r.pc), (Stop::Halt("alarm"), BASE), "{r:?}");
    assert_eq!(r.detail, "at 30");
    assert_eq!((m.now(), m.instructions()), (30, 0));
    // Taken: the next run does not report it again, and idling continues from there.
    let r = m.run(Limit {
        max_instructions: Some(1),
        ..Limit::default()
    });
    assert_eq!((r.kind, m.now()), (Stop::InstructionLimit, 31), "{r:?}");
    assert_eq!(m.idle_until(100), None);
    assert_eq!(m.now(), 100);
}

#[test]
fn idle_until_stops_at_an_event_that_is_not_advanced() {
    let mut m = Machine::new();
    m.add_timer(Box::new(Alarm {
        at: 10,
        fired: false,
        stuck: true,
    }));
    let r = m.idle_until(50).expect("halted");
    assert_eq!(r.kind, Stop::Halt("device event not advanced"), "{r:?}");
    assert!(r.detail.contains("tick 10"), "{r:?}");
    assert_eq!(m.now(), 10);
}

/// A bus master that, on a write to its +0, writes the value to `peer` through its bus; on a write to its +4,
/// reads `peer` through its bus. It keeps the results.
struct Relay {
    peer: u32,
    seen: Vec<Result<u32, Fault>>,
}

impl Device for Relay {
    fn read(&mut self, _: u32, _: Size, _: &mut DeviceCx) -> u32 {
        0x5A
    }
    fn write(&mut self, offset: u32, _: Size, value: u32, cx: &mut DeviceCx) {
        let mut bus = cx.bus().expect("bus master");
        let r = match offset {
            0 => bus.write(self.peer, Size::Long, value).map(|()| 0),
            _ => bus.read(self.peer, Size::Long),
        };
        self.seen.push(r);
    }
}

#[test]
fn nested_masters_reach_each_other_but_not_back() {
    const A: u32 = 0xFC03_0000;
    const B: u32 = 0xFC04_0000;
    let mut m = Machine::new();
    // A forwards a write to B's +4; B then reads A through its own bus while A is being called.
    let a = m.add_bus_master(
        A,
        0x100,
        Box::new(Relay {
            peer: B + 4,
            seen: Vec::new(),
        }),
    );
    let b = m.add_bus_master(
        B,
        0x100,
        Box::new(Relay {
            peer: A,
            seen: Vec::new(),
        }),
    );
    m.access(A, Size::Long, Some(1)).unwrap();
    assert_eq!(m.device::<Relay>(a).seen, [Ok(0)]);
    assert_eq!(
        m.device::<Relay>(b).seen,
        [Err(Fault::Unmapped {
            addr: A,
            size: Size::Long
        })]
    );
    // From the host, B reads A: A is not being called, so it answers.
    m.access(B + 4, Size::Long, Some(0)).unwrap();
    assert_eq!(m.device::<Relay>(b).seen[1], Ok(0x5A));
}
