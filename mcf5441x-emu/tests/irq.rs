//! Interrupts (CFPRM ch. 11, RM 3.3.4.13, RM 17.1.1): the IPL mask, level 7 as a non-maskable edge, the clock, timers and
//! STOP. A small interrupt controller and timer are defined here; the chip's INTC is modelled elsewhere.

mod common;

use common::{BASE, STACK, prepare, set_vector, until};
use mcf5441x_emu::{
    Device, DeviceCx, InterruptController, Irq, IrqLines, Limit, Machine, Size, Stop,
};

/// Where [`TestIntc`] is mapped.
const INTC: u32 = 0xFC04_8000;
/// Where [`Trigger`] is mapped.
const TRIGGER: u32 = 0xEC00_0000;

/// An interrupt controller with 64 sources: a level per source (0 disables it), a mask bit per source (all masked
/// at reset), and vector `64 + 64 * index + source`. Within a level the higher-numbered source wins.
struct TestIntc {
    index: usize,
    icr: [u8; 64],
    imr: u64,
}

impl TestIntc {
    const SIMR: u32 = 0x1C;
    const CIMR: u32 = 0x1D;
    const ICR0: u32 = 0x40;

    fn new(index: usize) -> TestIntc {
        TestIntc {
            index,
            icr: [0; 64],
            imr: !0,
        }
    }

    /// Unmask `source` at `level`.
    fn enable(&mut self, source: u8, level: u8) {
        self.icr[source as usize] = level;
        self.imr &= !(1 << source);
    }

    /// Asserted, unmasked sources with a level, as (level, source), highest first.
    fn candidates(&self, lines: &IrqLines) -> impl Iterator<Item = (u8, u8)> + '_ {
        let live = lines.asserted(self.index) & !self.imr;
        (0..64u8)
            .rev()
            .filter(move |s| live >> s & 1 != 0 && self.icr[*s as usize] != 0)
            .map(|s| (self.icr[s as usize], s))
    }
}

impl Device for TestIntc {
    fn read(&mut self, _: u32, _: Size, _: &mut DeviceCx) -> u32 {
        0
    }
    fn write(&mut self, offset: u32, _: Size, value: u32, _: &mut DeviceCx) {
        match offset {
            Self::SIMR => self.imr |= 1 << (value & 63),
            Self::CIMR => self.imr &= !(1 << (value & 63)),
            o if (Self::ICR0..Self::ICR0 + 64).contains(&o) => {
                self.icr[(o - Self::ICR0) as usize] = value as u8 & 7
            }
            _ => {}
        }
    }
}

impl InterruptController for TestIntc {
    fn pending(&self, lines: &IrqLines, allow_level7: bool) -> Option<Irq> {
        self.candidates(lines)
            .filter(|&(level, _)| level != 7 || allow_level7)
            .max_by_key(|&(level, s)| (level, s))
            .map(|(level, s)| Irq {
                level,
                vector: (64 + 64 * self.index + s as usize) as u8,
            })
    }
    fn level7(&self, lines: &IrqLines) -> bool {
        self.candidates(lines).any(|(level, _)| level == 7)
    }
}

/// A write of `on << 8 | source` sets that source of controller 0. The tests point A2 at it.
struct Trigger;

impl Device for Trigger {
    fn read(&mut self, _: u32, _: Size, _: &mut DeviceCx) -> u32 {
        0
    }
    fn write(&mut self, _: u32, _: Size, value: u32, cx: &mut DeviceCx) {
        cx.irq.set(0, value as u8 & 63, value & 0x100 != 0);
    }
}

/// Asserts `source` of controller 0 at tick `at`, then every `period` ticks.
struct Timer {
    at: Option<u64>,
    period: Option<u64>,
    source: u8,
    fired: Vec<u64>,
}

impl Timer {
    fn once(at: u64, source: u8) -> Timer {
        Timer {
            at: Some(at),
            period: None,
            source,
            fired: Vec::new(),
        }
    }
}

impl Device for Timer {
    fn read(&mut self, _: u32, _: Size, _: &mut DeviceCx) -> u32 {
        0
    }
    fn write(&mut self, _: u32, _: Size, _: u32, _: &mut DeviceCx) {}
    fn next_event(&self) -> Option<u64> {
        self.at
    }
    fn advance(&mut self, now: u64, cx: &mut DeviceCx) {
        self.fired.push(now);
        cx.irq.set(0, self.source, true);
        self.at = self.period.map(|p| now + p);
    }
}

/// The interrupted program: a 200-iteration loop in D2, then `done`; the handler for source 6 (vector 70) counts in
/// D4 and masks source 6 again, as an acknowledge would.
const SPIN: &str = "sr_set: move.w #0x2000,%sr\nclr.l %d2\n\
                    spin: addq.l #1,%d2\ncmp.l #200,%d2\nbne.s spin\n\
                    done: nop\n\
                    handler: addq.l #1,%d4\nmoveq #6,%d1\nmove.b %d1,0xFC04801C\nrte";

/// [`SPIN`] in a machine with a [`TestIntc`] (index 0) whose source 6 is at `level`, unmasked or not, and asserted.
fn spin(sr: u16, level: u8, unmask: bool) -> (Machine, u32) {
    let (mut m, l) = prepare(SPIN, &["sr_set", "done", "handler"]);
    // Patch the immediate of the first MOVE to SR.
    m.mem.write(l[0] + 2, Size::Word, sr as u32).unwrap();
    set_vector(&mut m, 70, l[2]);
    let mut intc = TestIntc::new(0);
    intc.icr[6] = level;
    if unmask {
        intc.imr &= !(1 << 6);
    }
    m.add_interrupt_controller(INTC, 0x4000, Box::new(intc));
    m.mem.irq_lines_mut().set(0, 6, true);
    (m, l[1])
}

#[test]
fn cycle_clock_counts_instructions() {
    let (mut m, done) = spin(0x2000, 3, false);
    let r = m.run(until(done));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.now(), m.instructions());
    assert_eq!(m.now(), 2 + 200 * 3);
}

#[test]
fn masked_source_not_delivered() {
    let (mut m, done) = spin(0x2000, 3, false);
    let r = m.run(until(done));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!((m.cpu.d[4], m.cpu.d[2]), (0, 200));
}

#[test]
fn unmasked_source_delivered_and_rte_resumes() {
    let (mut m, done) = spin(0x2000, 3, true);
    let r = m.run(until(done));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.cpu.d[4], 1, "taken once: the handler masks it again");
    assert_eq!(m.cpu.d[2], 200, "the interrupted loop completes");
    assert_eq!(m.cpu.sr() & 0xFF00, 0x2000, "RTE restores the IPL");
    assert_eq!(m.cpu.a[7], STACK);
    // The frame: format 4, vector 70, the SR before the interrupt (IPL 0).
    let fv = m.mem.read(STACK - 8, Size::Long).unwrap();
    assert_eq!(fv >> 28, 4);
    assert_eq!(fv >> 18 & 0xFF, 70);
    assert_eq!(fv as u16 & 0x0700, 0);
}

#[test]
fn interrupt_entry_sets_the_ipl_and_clears_m() {
    // CFPRM ch. 11: S set, T and M cleared, I set to the level being taken.
    let src = "move.w #0x3000,%sr\nnop\nend: nop\nhandler: move.w %sr,%d5\nrte";
    let (mut m, l) = prepare(src, &["end", "handler"]);
    set_vector(&mut m, 70, l[1]);
    let mut intc = TestIntc::new(0);
    intc.enable(6, 5);
    m.add_interrupt_controller(INTC, 0x4000, Box::new(intc));
    m.mem.irq_lines_mut().set(0, 6, true);
    let r = m.run(Limit {
        until_pc: Some(l[1] + 2),
        ..Limit::default()
    });
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.cpu.d[5] as u16, 0x2500);
}

#[test]
fn level_not_above_ipl_waits_until_sr_lowered() {
    // Level 3 is not above IPL 3: never taken.
    let (mut m, done) = spin(0x2300, 3, true);
    m.run(until(done));
    assert_eq!(m.cpu.d[4], 0);
    // Level 4 is.
    let (mut m, done) = spin(0x2300, 4, true);
    m.run(until(done));
    assert_eq!(m.cpu.d[4], 1);

    // A held level-3 request is taken right after the instruction that lowers the mask below 3.
    let src = "move.w #0x2300,%sr\nmoveq #1,%d0\nmoveq #2,%d0\nmove.w #0x2200,%sr\nafter: moveq #3,%d0\nend: nop\n\
               handler: move.l %d0,%d5\nmove.l 4(%sp),%d6\naddq.l #1,%d4\nmoveq #6,%d1\nmove.b %d1,0xFC04801C\nrte";
    let (mut m, l) = prepare(src, &["after", "end", "handler"]);
    set_vector(&mut m, 70, l[2]);
    let mut intc = TestIntc::new(0);
    intc.enable(6, 3);
    m.add_interrupt_controller(INTC, 0x4000, Box::new(intc));
    m.mem.irq_lines_mut().set(0, 6, true);
    let r = m.run(until(l[1]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!((m.cpu.d[4], m.cpu.d[5], m.cpu.d[6]), (1, 2, l[0]));
    assert_eq!(m.cpu.d[0], 3);
}

/// Sources 5 and 6 at level 7; their handlers count in D4 and D6 and never clear the request.
fn level7(body: &str) -> (Machine, u32) {
    let src = format!("{body}\nend: nop\nh5: addq.l #1,%d4\nrte\nh6: addq.l #1,%d6\nrte");
    let (mut m, l) = prepare(&src, &["end", "h5", "h6"]);
    set_vector(&mut m, 69, l[1]);
    set_vector(&mut m, 70, l[2]);
    let mut intc = TestIntc::new(0);
    intc.enable(5, 7);
    intc.enable(6, 7);
    m.add_interrupt_controller(INTC, 0x4000, Box::new(intc));
    m.add_device(TRIGGER, 0x100, Box::new(Trigger));
    m.cpu.a[2] = TRIGGER;
    (m, l[0])
}

#[test]
fn level_7_is_taken_once_per_assertion() {
    // IPL 7 cannot mask level 7; the handler returns with the request still asserted.
    let (mut m, end) = level7(
        "move.w #0x2700,%sr\n\
         move.l #0x105,(%a2)\n\
         move.l #200,%d3\n1: subq.l #1,%d3\nbne.s 1b\n\
         move.l #0x005,(%a2)\n\
         move.l #0x105,(%a2)\n\
         move.l #200,%d3\n2: subq.l #1,%d3\nbne.s 2b",
    );
    let r = m.run(until(end));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.cpu.d[4], 2, "once per rise of the line");
}

#[test]
fn a_second_level_7_source_while_one_is_held_is_not_a_new_edge() {
    // The core sees one level-7 line: a second source raising it while it is held, or the first one letting go
    // while the second holds it, is not a new edge.
    let (mut m, end) = level7(
        "move.w #0x2000,%sr\n\
         move.l #0x105,(%a2)\n\
         move.l #0x106,(%a2)\n\
         move.l #0x005,(%a2)\n\
         nop\n\
         move.l #0x006,(%a2)\n\
         move.l #0x106,(%a2)\n\
         nop",
    );
    let r = m.run(until(end));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!((m.cpu.d[4], m.cpu.d[6]), (1, 1));
}

#[test]
fn stop_with_only_a_held_level_7_line_halts() {
    // The held line was taken once and no new edge can come while it is held, even though a timer keeps
    // re-asserting the source that holds it: nothing can wake the STOP.
    let (mut m, end) = level7(
        "move.w #0x2000,%sr\n\
         move.l #0x105,(%a2)\n\
         stop #0x2000\n\
         after: nop",
    );
    m.add_timer(periodic(5));
    let r = m.run(until(end));
    assert_eq!(r.kind, Stop::HaltedForever, "{r:?}");
    assert_eq!(r.pc, end - 2, "PC is past the STOP");
    assert_eq!(m.cpu.d[4], 1);
    // Running again keeps waiting.
    let r = m.run(until(end));
    assert_eq!(r.kind, Stop::HaltedForever, "{r:?}");
    assert_eq!(m.cpu.d[4], 1);
}

/// `stop #0x2000`, then `moveq #1,%d7`; a [`Timer`] raises source 6 (level 3) at `at`.
fn stopped(at: u64) -> (Machine, u32) {
    let src = "stop #0x2000\nmoveq #1,%d7\nend: nop\n\
               handler: addq.l #1,%d4\nmoveq #6,%d1\nmove.b %d1,0xFC04801C\nrte";
    let (mut m, l) = prepare(src, &["end", "handler"]);
    set_vector(&mut m, 70, l[1]);
    let mut intc = TestIntc::new(0);
    intc.enable(6, 3);
    m.add_interrupt_controller(INTC, 0x4000, Box::new(intc));
    m.add_timer(Box::new(Timer::once(at, 6)));
    (m, l[0])
}

#[test]
fn stop_fast_forwards_to_the_timer() {
    let (mut m, end) = stopped(1000);
    let r = m.run(until(end));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.cpu.d[4], 1);
    assert_eq!(m.cpu.d[7], 1);
    // STOP is tick 0; the clock jumps to 1000, where the handler's four instructions and the MOVEQ run.
    assert_eq!(m.now(), 1005);
    assert_eq!(m.instructions(), 6);

    // An event past the time limit: the clock stops at the limit, still waiting; a later limit wakes it.
    let (mut m, end) = stopped(1000);
    m.set_time_limit(Some(500));
    let r = m.run(until(end));
    assert_eq!(
        (r.kind, r.pc, m.now()),
        (Stop::TimeLimit, BASE + 4, 500),
        "{r:?}"
    );
    assert_eq!(m.cpu.d[4], 0);
    m.set_time_limit(Some(2000));
    let r = m.run(until(end));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!((m.cpu.d[4], m.cpu.d[7], m.now()), (1, 1, 1005));
}

/// A timer asserting `source` of controller 0 every 100 ticks from tick 100.
fn periodic(source: u8) -> Box<Timer> {
    Box::new(Timer {
        at: Some(100),
        period: Some(100),
        source,
        fired: Vec::new(),
    })
}

#[test]
fn stop_with_a_masked_periodic_timer_halts() {
    // The timer keeps scheduling events, but its source is masked in the controller and nothing can unmask it
    // while the core waits: no event can wake the STOP, with or without a time limit.
    let src = "stop #0x2000\nend: nop";
    let (mut m, l) = prepare(src, &["end"]);
    m.add_interrupt_controller(INTC, 0x4000, Box::new(TestIntc::new(0)));
    let id = m.add_timer(periodic(6));
    let r = m.run(until(l[0]));
    assert_eq!((r.kind, r.pc), (Stop::HaltedForever, BASE + 4), "{r:?}");
    assert_eq!(m.now(), 1);
    assert!(m.device::<Timer>(id).fired.is_empty());

    // The same with the source unmasked but at a level not above the STOP's mask.
    let src = "stop #0x2300\nend: nop";
    let (mut m, l) = prepare(src, &["end"]);
    let mut intc = TestIntc::new(0);
    intc.enable(6, 3);
    m.add_interrupt_controller(INTC, 0x4000, Box::new(intc));
    m.add_timer(periodic(6));
    m.set_time_limit(Some(450));
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::HaltedForever, "{r:?}");
}

#[test]
fn stop_with_an_unmasked_periodic_timer_wakes() {
    // A level above the STOP's mask: the first event wakes it.
    let src = "stop #0x2000\nend: nop\nhandler: moveq #6,%d1\nmove.b %d1,0xFC04801C\nrte";
    let (mut m, l) = prepare(src, &["end", "handler"]);
    set_vector(&mut m, 70, l[1]);
    let mut intc = TestIntc::new(0);
    intc.enable(6, 3);
    m.add_interrupt_controller(INTC, 0x4000, Box::new(intc));
    let id = m.add_timer(periodic(6));
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.device::<Timer>(id).fired, [100]);
    assert_eq!(m.now(), 103);
}

#[test]
fn equal_levels_on_two_controllers_go_to_the_first() {
    // RM 17.3.2: INTC0 has the highest priority; at the same level its request is serviced first.
    let src = "move.w #0x2000,%sr\nnop\nend: nop\n\
               h0: move.l %d7,%d5\naddq.l #1,%d7\nmoveq #6,%d1\nmove.b %d1,0xFC04801C\nrte\n\
               h1: move.l %d7,%d6\naddq.l #1,%d7\nmoveq #6,%d1\nmove.b %d1,0xFC04C01C\nrte";
    let (mut m, l) = prepare(src, &["end", "h0", "h1"]);
    set_vector(&mut m, 70, l[1]);
    set_vector(&mut m, 64 + 64 + 6, l[2]);
    // Controllers rank in the order they are added: INTC0 first.
    for (index, base) in [(0, INTC), (1, INTC + 0x4000)] {
        let mut intc = TestIntc::new(index);
        intc.enable(6, 3);
        m.add_interrupt_controller(base, 0x4000, Box::new(intc));
        m.mem.irq_lines_mut().set(index, 6, true);
    }
    m.cpu.d[5] = 0xFF;
    m.cpu.d[6] = 0xFF;
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!((m.cpu.d[5], m.cpu.d[6], m.cpu.d[7]), (0, 1, 2));
}

#[test]
fn irq_from_mmio_write_taken_at_next_instruction() {
    let src = "move.w #0x2000,%sr\nmove.l #0x106,(%a2)\nnext: moveq #1,%d0\nend: nop\n\
               handler: move.l %d0,%d5\nmove.l 4(%sp),%d6\naddq.l #1,%d4\nmove.l #0x006,(%a2)\nrte";
    let (mut m, l) = prepare(src, &["next", "end", "handler"]);
    set_vector(&mut m, 70, l[2]);
    let mut intc = TestIntc::new(0);
    intc.enable(6, 3);
    m.add_interrupt_controller(INTC, 0x4000, Box::new(intc));
    m.add_device(TRIGGER, 0x100, Box::new(Trigger));
    m.cpu.a[2] = TRIGGER;
    m.cpu.d[0] = 0xAAAA;
    let r = m.run(until(l[1]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(
        m.cpu.d[5], 0xAAAA,
        "the instruction after the write had not run"
    );
    assert_eq!(m.cpu.d[6], l[0], "the frame returns to it");
    assert_eq!((m.cpu.d[4], m.cpu.d[0]), (1, 1));
}

#[test]
fn event_due_mid_block_is_serviced_at_its_instruction() {
    // A RAM-only loop; the timer is due at tick 50, and the handler's first instruction runs at tick 50.
    let src = "move.w #0x2000,%sr\nloop: addq.l #1,%d0\nbra.s loop\n\
               handler: move.l %d0,0xEC000000\nmoveq #6,%d1\nmove.b %d1,0xFC04801C\nrte";
    let (mut m, l) = prepare(src, &["handler"]);
    set_vector(&mut m, 70, l[0]);
    let mut intc = TestIntc::new(0);
    intc.enable(6, 3);
    m.add_interrupt_controller(INTC, 0x4000, Box::new(intc));
    m.add_device(TRIGGER, 0x100, Box::new(Trigger));
    let timer = m.add_timer(Box::new(Timer::once(50, 6)));
    m.trace(true);
    let r = m.run(Limit {
        max_instructions: Some(100),
        ..Limit::default()
    });
    assert_eq!(r.kind, Stop::InstructionLimit, "{r:?}");
    assert_eq!(m.device::<Timer>(timer).fired, [50]);
    let first = m.take_trace()[0];
    assert_eq!((first.t, first.pc, first.addr), (50, l[0], TRIGGER));
    // Ticks 1-49 ran the loop: 25 ADDQs.
    assert_eq!(first.value, 25);
}

#[test]
fn highest_level_wins_and_a_policy_can_stop_on_an_interrupt() {
    let src = "move.w #0x2000,%sr\nnop\nend: nop\nh5: moveq #5,%d5\nmoveq #5,%d1\nmove.b %d1,0xFC04801C\nrte\nh6: moveq #6,%d6\nrte";
    let (mut m, l) = prepare(src, &["end", "h5", "h6"]);
    set_vector(&mut m, 69, l[1]);
    set_vector(&mut m, 70, l[2]);
    let mut intc = TestIntc::new(0);
    intc.enable(5, 4);
    intc.enable(6, 2);
    m.add_interrupt_controller(INTC, 0x4000, Box::new(intc));
    m.mem.irq_lines_mut().set(0, 5, true);
    m.mem.irq_lines_mut().set(0, 6, true);
    m.policy.stop_on[70] = true;
    let r = m.run(until(l[0]));
    // Level 4 first (its handler runs and returns), then level 2, which the policy stops on before entry.
    assert_eq!((r.kind, r.pc), (Stop::Exception(70), BASE + 4), "{r:?}");
    assert_eq!((m.cpu.d[5], m.cpu.d[6]), (5, 0));
    assert_eq!(m.cpu.sr(), 0x2000);
}
