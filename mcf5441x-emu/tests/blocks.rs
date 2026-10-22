//! The decoded-block cache: self-modifying code, loads over executed code, exact instruction limits inside a block,
//! the execution profile, and agreement with the instruction-at-a-time loop.

mod common;

use common::{BASE, STACK, prepare, until};
use mcf5441x_emu::{Limit, Machine, Stop};

#[test]
fn smc_overwrites_a_cached_block() {
    // The subroutine runs once (and is cached), then a MOVE.L stores new opcodes over its first two instructions.
    let src = "bsr.w sub\n\
               lea sub,%a3\nmove.l #0x7005d281,(%a3)\n\
               bsr.w sub\n\
               bra.w end\n\
               sub: moveq #1,%d0\nadd.l %d0,%d1\nrts\nend: nop";
    let (mut m, l) = prepare(src, &["end", "sub"]);
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    // First call: D1 += 1. Second call runs `moveq #5,%d0; add.l %d1,%d1`: D1 = 2 * 1.
    assert_eq!((m.cpu.d[0], m.cpu.d[1]), (5, 2));
    assert_eq!(m.mem.peek(l[1], 4).unwrap(), [0x70, 0x05, 0xd2, 0x81]);
}

#[test]
fn store_into_the_running_block_takes_effect_at_once() {
    // Pass n stores `moveq #6+n,%d0` a little ahead of the store, in the same straight-line run (and, from the
    // second pass on, in the same cached block): stores are seen by the next instruction fetched, as with the
    // instruction-at-a-time loop.
    let src = "lea patch,%a0\nmove.w #0x7007,%d3\nmoveq #3,%d6\n\
               again: move.w %d3,(%a0)\naddq.l #1,%d3\n\
               patch: moveq #1,%d0\nadd.l %d0,%d1\nsubq.l #1,%d6\nbne.s again\nend: nop";
    let (mut m, l) = prepare(src, &["end"]);
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.cpu.d[1], 7 + 8 + 9);
}

#[test]
fn store_over_the_next_page_half_of_an_instruction() {
    // MOVE.L #imm,D0 starts 4 bytes before a page boundary; the low word of its immediate is in the next page.
    // Writing only that word must still drop the cached instruction.
    let src = "bsr.w sub\nmove.l %d0,%d2\n\
               lea sub+4,%a3\nmove.w #0x2222,(%a3)\n\
               bsr.w sub\nbra.w end\n\
               .org 0xffc\nsub: move.l #0x11111111,%d0\nrts\nend: nop";
    let (mut m, l) = prepare(src, &["end", "sub"]);
    assert_eq!(l[1], BASE + 0xFFC);
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!((m.cpu.d[2], m.cpu.d[0]), (0x1111_1111, 0x1111_2222));
}

#[test]
fn load_over_cached_code_takes_effect() {
    let (mut m, l) = prepare("moveq #1,%d0\nadd.l %d0,%d1\nend: nop", &["end"]);
    m.run(until(l[0]));
    assert_eq!(m.cpu.d[1], 1);
    m.mem.load(BASE, &[0x70, 0x04]).unwrap(); // moveq #4,%d0
    m.cpu.pc = BASE;
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.cpu.d[1], 5);
}

#[test]
fn ram_write_from_outside_over_cached_code_takes_effect() {
    let (mut m, l) = prepare("moveq #1,%d0\nadd.l %d0,%d1\nend: nop", &["end"]);
    m.run(until(l[0]));
    m.mem.write(BASE, mcf5441x_emu::Size::Word, 0x7003).unwrap();
    m.cpu.pc = BASE;
    m.run(until(l[0]));
    assert_eq!(m.cpu.d[1], 4);
}

/// Ten ADDQs and a counted loop around them: three passes.
const TEN: &str = "moveq #3,%d7\n\
                   loop: addq.l #1,%d0\naddq.l #1,%d0\naddq.l #1,%d0\naddq.l #1,%d0\naddq.l #1,%d0\n\
                   addq.l #1,%d0\naddq.l #1,%d0\naddq.l #1,%d0\naddq.l #1,%d0\naddq.l #1,%d0\n\
                   subq.l #1,%d7\nbne.s loop\nend: nop";

#[test]
fn instruction_limit_inside_a_block() {
    let (mut m, l) = prepare(TEN, &["loop", "end"]);
    // 1 + 12 (first pass, now cached) + 4 into the second pass.
    let r = m.run(Limit {
        max_instructions: Some(17),
        ..Limit::default()
    });
    assert_eq!(r.kind, Stop::InstructionLimit, "{r:?}");
    assert_eq!((m.instructions(), m.cpu.pc, m.cpu.d[0]), (17, l[0] + 8, 14));
    // Three more stop in the middle of the same block again.
    let r = m.run(Limit {
        max_instructions: Some(3),
        ..Limit::default()
    });
    assert_eq!(r.kind, Stop::InstructionLimit, "{r:?}");
    assert_eq!(
        (m.instructions(), m.cpu.pc, m.cpu.d[0]),
        (20, l[0] + 14, 17)
    );
    let r = m.run(until(l[1]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!((m.instructions(), m.cpu.d[0]), (1 + 3 * 12, 30));
}

#[test]
fn until_pc_inside_a_block() {
    let (mut m, l) = prepare(TEN, &["loop", "end"]);
    m.run(Limit {
        max_instructions: Some(13),
        ..Limit::default()
    });
    let r = m.run(until(l[0] + 10));
    assert_eq!((r.kind, r.pc), (Stop::UntilPc, l[0] + 10), "{r:?}");
    assert_eq!((m.instructions(), m.cpu.d[0]), (18, 15));
}

#[test]
fn profile_counts_block_starts() {
    let src = "moveq #5,%d0\nloop: addq.l #1,%d1\nsubq.l #1,%d0\nbne.s loop\nafter: nop\nend: nop";
    let (mut m, l) = prepare(src, &["loop", "after", "end"]);
    // Off by default.
    m.run(Limit {
        max_instructions: Some(3),
        ..Limit::default()
    });
    assert_eq!(m.take_profile(), []);
    m.cpu.pc = BASE;
    m.cpu.d = [0; 8];
    m.profile(true);
    let r = m.run(until(l[2]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    // The first block runs into the loop's first pass; the loop block starts the other four.
    assert_eq!(m.take_profile(), [(BASE, 1), (l[0], 4), (l[1], 1)]);
    // Taking the profile clears it.
    assert_eq!(m.take_profile(), []);
    m.profile(false);
    m.cpu.pc = BASE;
    m.run(until(l[2]));
    assert_eq!(m.take_profile(), []);
}

/// The data [`MIXED`] writes.
const DATA: u32 = 0x4030_0000;

/// Branches, a subroutine, MOVEM to and from the stack, EMAC with a parallel load, stores and a TRAP handler.
const MIXED: &str = "\
        lea 0x40300000,%a0\n\
        moveq #0,%d0\n\
        move.l %d0,%macsr\n\
        move.l %d0,%acc0\n\
        move.l #40,%d7\n\
    fill: move.l %d7,(%a0)+\n\
        subq.l #1,%d7\n\
        bne.s fill\n\
        lea 0x40300000,%a0\n\
        move.l #40,%d7\n\
        moveq #3,%d1\n\
    loop: bsr.s sub\n\
        mac.l %d1,%d2,(%a0)+,%d2,%acc0\n\
        btst #0,%d2\n\
        beq.s even\n\
        add.l %d2,%d3\n\
        bra.s next\n\
    even: sub.l %d2,%d4\n\
    next: move.l %d3,-(%a0)\n\
        addq.l #4,%a0\n\
        trap #1\n\
        subq.l #1,%d7\n\
        bne.s loop\n\
        move.l %acc0,%d6\n\
        bra.w end\n\
    sub: lea -16(%sp),%sp\n\
        movem.l %d1-%d4,(%sp)\n\
        addq.l #1,%d1\n\
        movem.l (%sp),%d1-%d4\n\
        lea 16(%sp),%sp\n\
        addq.l #1,%d5\n\
        rts\n\
    handler: addq.l #2,%d5\n\
        rte\n\
    end: nop";

fn mixed() -> (Machine, u32) {
    let (mut m, l) = prepare(MIXED, &["end", "handler"]);
    common::set_vector(&mut m, 33, l[1]);
    (m, l[0])
}

fn same_state(a: &Machine, b: &Machine) {
    assert_eq!(a.instructions(), b.instructions());
    assert_eq!(a.now(), b.now());
    assert_eq!((a.cpu.d, a.cpu.a, a.cpu.pc), (b.cpu.d, b.cpu.a, b.cpu.pc));
    assert_eq!(a.cpu.sr(), b.cpu.sr());
    assert_eq!(a.cpu.emac, b.cpu.emac);
    assert_eq!(a.mem.peek(DATA, 0x200), b.mem.peek(DATA, 0x200));
    assert_eq!(
        a.mem.peek(STACK - 0x100, 0x100),
        b.mem.peek(STACK - 0x100, 0x100)
    );
}

#[test]
fn same_counts_with_and_without_cache() {
    let (mut cached, end) = mixed();
    let (mut plain, _) = mixed();
    let r = cached.run(until(end));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    let r = plain.run_uncached(until(end));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    same_state(&cached, &plain);
    assert!(cached.instructions() > 600, "{}", cached.instructions());
    assert_ne!(cached.cpu.d[6], 0);

    // In slices of seven instructions, which stop inside blocks and resume.
    let (mut sliced, _) = mixed();
    loop {
        let r = sliced.run(Limit {
            until_pc: Some(end),
            max_instructions: Some(7),
            ..Limit::default()
        });
        if r.kind == Stop::UntilPc {
            break;
        }
        assert_eq!(r.kind, Stop::InstructionLimit, "{r:?}");
    }
    same_state(&sliced, &plain);
}
