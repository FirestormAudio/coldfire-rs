//! Exception processing (CFPRM ch. 11, RM 3.3.3-3.3.4) and the supervisor instructions (CFPRM ch. 8).

mod common;

use common::{BASE, MEM, STACK};
use mcf5441x_emu::{Limit, Machine, Size, Stop, StopReason};

/// The vector table used by these tests.
const VBR: u32 = 0x4020_0000;
/// A user stack, away from the supervisor stack at `STACK`.
const USP: u32 = 0x4080_0000;
/// CACR[EUSP]: separate user and supervisor stack pointers (RM ch. 6, CACR bit 5).
const EUSP: u32 = 0x20;

/// Assemble `src` at `BASE` into a machine with RAM at `0x40100000..0x41100000`, A7 = `STACK` (supervisor mode, the
/// reset SR 0x2700) and VBR = `VBR`; returns it and the addresses of `labels`.
fn prepare(src: &str, labels: &[&str]) -> (Machine, Vec<u32>) {
    let tail: String = labels.iter().map(|l| format!(".long {l}\n")).collect();
    let code = common::assemble(&format!("{src}\n.balign 2\n{tail}"), BASE);
    let (code, tail) = code.split_at(code.len() - 4 * labels.len());
    let addrs = tail
        .chunks(4)
        .map(|c| u32::from_be_bytes(c.try_into().unwrap()))
        .collect();
    let mut m = Machine::new();
    m.mem.map_ram(0x4010_0000, 0x0100_0000);
    m.mem.load(BASE, code).unwrap();
    m.cpu.pc = BASE;
    m.cpu.a[7] = STACK;
    m.cpu.vbr = VBR;
    (m, addrs)
}

fn until(pc: u32) -> Limit {
    Limit {
        until_pc: Some(pc),
        ..Limit::default()
    }
}

fn long(m: &mut Machine, addr: u32) -> u32 {
    m.mem.read(addr, Size::Long).unwrap()
}

/// Point vector `v` of the table at `VBR` to `handler`.
fn set_vector(m: &mut Machine, v: u32, handler: u32) {
    m.mem.write(VBR + 4 * v, Size::Long, handler).unwrap();
}

/// FS = 0100 in a frame's first longword: FS[3:2] in bits 27-26, FS[1:0] in bits 17-16 (RM Figure 3-10).
const FS_FETCH: u32 = 0b01 << 26;
/// FS = 1100, an error on an operand read (RM Table 3-7).
const FS_OPERAND: u32 = 0b11 << 26;

/// The first longword of a frame (CFPRM Figure 11-1): format, vector, SR.
fn fv(format: u32, vector: u32, sr: u16) -> u32 {
    (format << 28) | (vector << 18) | sr as u32
}

#[test]
fn trap_is_delivered_through_vbr() {
    let src = "move.l #0x40200000,%d0\nmovec %d0,%vbr\nmove.w #0x2704,%sr\ntrap #3\nafter: moveq #1,%d2\nend: nop\n\
               handler: moveq #7,%d1\nrte";
    let (mut m, l) = prepare(src, &["after", "end", "handler"]);
    let (after, end, handler) = (l[0], l[1], l[2]);
    m.cpu.vbr = 0;
    set_vector(&mut m, 32 + 3, handler);
    let r = m.run(until(handler));
    assert_eq!((r.kind, r.pc), (Stop::UntilPc, handler), "{r:?}");
    assert_eq!(m.cpu.vbr, VBR);
    assert_eq!(m.cpu.a[7], STACK - 8);
    assert_eq!(
        long(&mut m, STACK - 8),
        fv(4, 35, 0x2704),
        "format 4, vector 35, SR"
    );
    assert_eq!(long(&mut m, STACK - 4), after, "TRAP stacks the next PC");
    assert_eq!(m.cpu.sr(), 0x2704);
    assert_eq!(m.instructions(), 4, "the trap counts as an instruction");

    let r = m.run(until(after));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.cpu.d[1], 7);
    assert_eq!(m.cpu.a[7], STACK);
    assert_eq!(
        m.cpu.sr(),
        0x2704,
        "the handler's MOVEQ cleared Z; RTE restored it"
    );
    let r = m.run(until(end));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.cpu.d[2], 1);
}

#[test]
fn trap_stops_when_policy_says_so() {
    let (mut m, l) = prepare("moveq #1,%d0\ntrap #3\nmoveq #2,%d0\nend:", &["end"]);
    m.policy.stop_on[35] = true;
    let r = m.run(until(l[0]));
    assert_eq!((r.kind, r.pc), (Stop::Exception(35), BASE + 2), "{r:?}");
    assert_eq!(m.cpu.pc, BASE + 2);
    assert_eq!(m.cpu.d[0], 1);
    assert_eq!(m.cpu.a[7], STACK);
    assert_eq!(m.cpu.sr(), 0x2700);
    assert_eq!(long(&mut m, STACK - 8), 0, "no frame");
    assert_eq!(m.instructions(), 1);
}

#[test]
fn default_policy_vectors_traps_and_stops_on_the_rest() {
    let m = Machine::new();
    for (v, &stop) in m.policy.stop_on.iter().enumerate() {
        let vectored = (32..48).contains(&v) || v >= 64;
        assert_eq!(stop, !vectored, "vector {v}");
    }
}

#[test]
fn rte_resumes_after_trap_with_sr_restored() {
    // User mode with separate stacks; the supervisor stack is misaligned by 2, so the frame has format 6.
    let src = "moveq #0x20,%d0\nmovec %d0,%cacr\nmove.l #0x40800000,%a0\nmove.l %a0,%usp\n\
               move.w #0x0013,%sr\ntrap #0\nafter: move.w %ccr,%d3\nend: nop\n\
               handler: moveq #0,%d4\nrte";
    let (mut m, l) = prepare(src, &["after", "end", "handler"]);
    let (after, end, handler) = (l[0], l[1], l[2]);
    m.cpu.a[7] = STACK - 2;
    set_vector(&mut m, 32, handler);
    let r = m.run(until(handler));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(
        m.cpu.a[7],
        STACK - 12,
        "SSP - 2 aligned down, minus the 8-byte frame"
    );
    assert_eq!(m.cpu.other_sp, USP);
    assert_eq!(long(&mut m, STACK - 12), fv(6, 32, 0x0013));
    assert_eq!(long(&mut m, STACK - 8), after);
    assert_eq!(m.cpu.sr(), 0x2013, "S set, CCR kept");

    let r = m.run(until(end));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.cpu.sr(), 0x0013, "user mode and the CCR from the frame");
    assert_eq!(m.cpu.a[7], USP);
    assert_eq!(m.cpu.other_sp, STACK - 2, "SP + 8 + format adjustment (2)");
    assert_eq!(m.cpu.d[3], 0x13);
}

#[test]
fn rte_with_a_bad_format_is_a_format_error() {
    let (mut m, l) = prepare("rte\nend:", &["end"]);
    m.mem.write(STACK - 8, Size::Long, 0x0000_2700).unwrap(); // a 68000-style SR word: format 0
    m.mem.write(STACK - 4, Size::Long, BASE).unwrap();
    m.cpu.a[7] = STACK - 8;
    let r = m.run(until(l[0]));
    assert_eq!((r.kind, r.pc), (Stop::Exception(14), BASE), "{r:?}");
    assert_eq!(m.cpu.a[7], STACK - 8);
}

/// CFPRM ch. 11 step 1 and RM Table 3-3: exception entry sets S and clears T; only an interrupt clears M and
/// loads the interrupt level into the mask.
#[test]
fn exception_entry_clears_t_and_keeps_m() {
    let (mut m, l) = prepare("trap #1\nend: nop\nhandler: nop", &["handler"]);
    let handler = l[0];
    set_vector(&mut m, 33, handler);
    m.cpu.set_sr(0xB305); // T S M, IPL 3, Z C
    let r = m.run(until(handler));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(long(&mut m, STACK - 8), fv(4, 33, 0xB305));
    assert_eq!(m.cpu.sr(), 0x3305, "T cleared, S and M set, IPL kept");
}

#[test]
fn interrupt_entry_clears_m_and_t_and_sets_the_level() {
    let (mut m, _) = prepare("nop", &[]);
    set_vector(&mut m, 70, 0x4011_0000);
    m.cpu.set_sr(0xB305);
    let m = &mut m;
    m.cpu.interrupt(&mut m.mem, 70, 5, BASE + 6).unwrap();
    assert_eq!(m.cpu.sr(), 0x2505, "T and M cleared, IPL 5");
    assert_eq!(m.cpu.pc, 0x4011_0000);
    assert_eq!(m.cpu.a[7], STACK - 8);
    assert_eq!(long(m, STACK - 8), fv(4, 70, 0xB305));
    assert_eq!(long(m, STACK - 4), BASE + 6);
}

#[test]
fn raise_from_user_mode_builds_the_frame_on_the_supervisor_stack() {
    let (mut m, _) = prepare("nop", &[]);
    set_vector(&mut m, 8, 0x4011_0000);
    m.cpu.cacr = EUSP;
    m.cpu.other_sp = USP + 3; // the user stack is the active A7 after the switch below
    m.cpu.set_sr(0x0000);
    assert_eq!(m.cpu.a[7], USP + 3);
    let m = &mut m;
    m.cpu.raise(&mut m.mem, 8, BASE).unwrap();
    assert_eq!(
        m.cpu.a[7],
        STACK - 8,
        "the frame goes on the SSP, whatever the user SP's alignment"
    );
    assert_eq!(m.cpu.other_sp, USP + 3);
    assert_eq!(long(m, STACK - 8), fv(4, 8, 0x0000));
    assert_eq!(m.cpu.sr(), 0x2000);
}

#[test]
fn trace_takes_vector_9_after_each_instruction() {
    let (mut m, l) = prepare("moveq #1,%d0\nmoveq #2,%d1\nend:", &["end"]);
    m.cpu.set_sr(0xA700);
    let r = m.run(until(l[0]));
    assert_eq!((r.kind, r.pc), (Stop::Exception(9), BASE + 2), "{r:?}");
    assert_eq!((m.cpu.d[0], m.cpu.d[1]), (1, 0));
    assert_eq!(m.instructions(), 1);

    // Vectored: the handler counts and returns; the RTE does not trace (T is clear when it starts).
    let (mut m, l) = prepare(
        "moveq #1,%d0\nmoveq #2,%d1\nend: nop\nhandler: addq.l #1,%d7\nrte",
        &["end", "handler"],
    );
    set_vector(&mut m, 9, l[1]);
    m.policy.stop_on[9] = false;
    m.cpu.set_sr(0xA700);
    let r = m.run(until(l[1]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(long(&mut m, STACK - 8), fv(4, 9, 0xA700));
    assert_eq!(
        long(&mut m, STACK - 4),
        BASE + 2,
        "trace stacks the next PC"
    );
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!((m.cpu.d[0], m.cpu.d[1], m.cpu.d[7]), (1, 2, 2));
    assert_eq!(m.cpu.sr(), 0xA700);
}

#[test]
fn privilege_violation_in_user_mode() {
    let rows = [
        "move.w %d0,%sr",
        "move.w #0x2700,%sr",
        "move.w %sr,%d0",
        "movec %d0,%vbr",
        "move.l %a0,%usp",
        "move.l %usp,%a0",
        "rte",
        "stop #0x2000",
        "halt",
        "cpushl %bc,(%a0)",
        "intouch %a0",
        // guess: CFPRM ch. 8 gives STRLDSR's user-mode operation as "Else TRAP" without a vector; this core takes
        // the privilege violation every other supervisor instruction takes.
        ".short 0x40e7,0x46fc,0x2700",
        "wdebug.l (%a0)",
    ];
    for insn in rows {
        let (mut m, l) = prepare(&format!("moveq #1,%d5\n{insn}\nend:"), &["end"]);
        m.cpu.set_sr(0x0000);
        m.cpu.d[0] = 0x1234_5678;
        m.cpu.a[0] = MEM;
        let r = m.run(until(l[0]));
        assert_eq!(
            (r.kind, r.pc),
            (Stop::Exception(8), BASE + 2),
            "{insn}: {r:?}"
        );
        assert_eq!(m.cpu.sr(), 0x0000, "{insn}");
        assert_eq!(
            (m.cpu.d[0], m.cpu.a[0], m.cpu.a[7]),
            (0x1234_5678, MEM, STACK),
            "{insn}"
        );
        assert_eq!(m.cpu.vbr, VBR, "{insn}");
        assert_eq!(m.instructions(), 1, "{insn}");
    }

    // Vectored, with a single stack pointer (CACR[EUSP] clear): the frame goes on the one A7.
    let (mut m, l) = prepare(
        "moveq #1,%d5\nmove.w %d0,%sr\nend: nop\nhandler: nop",
        &["handler"],
    );
    set_vector(&mut m, 8, l[0]);
    m.policy.stop_on[8] = false;
    m.cpu.set_sr(0x0010);
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.cpu.a[7], STACK - 8);
    assert_eq!(long(&mut m, STACK - 8), fv(4, 8, 0x0010));
    assert_eq!(
        long(&mut m, STACK - 4),
        BASE + 2,
        "the faulting instruction"
    );
    assert_eq!(m.cpu.sr(), 0x2010);
}

#[test]
fn user_mode_runs_the_unprivileged_system_instructions() {
    let src = "move.w %ccr,%d1\nmove.w #0x15,%ccr\npulse\nwddata.l (%a0)+\nend:";
    let (mut m, l) = prepare(src, &["end"]);
    m.cpu.set_sr(0x000A);
    m.cpu.a[0] = MEM;
    m.cpu.d[1] = 0xAAAA_AAAA;
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.cpu.d[1], 0xAAAA_000A, "MOVE from CCR writes a word");
    assert_eq!(m.cpu.sr(), 0x0015);
    assert_eq!(m.cpu.a[0], MEM + 4, "WDDATA reads its operand");
}

#[test]
fn move_to_and_from_sr() {
    let src = "move.w %d0,%sr\nmove.w %sr,%d1\nmove.w #0x2311,%sr\nmove.w %sr,%d2\nend:";
    let (mut m, l) = prepare(src, &["end"]);
    m.cpu.d[0] = 0x7FFF; // every bit but T: the reserved bits read as zero
    m.cpu.d[1] = 0xAAAA_AAAA;
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.cpu.d[1], 0xAAAA_371F, "MOVE from SR writes a word");
    assert_eq!(m.cpu.d[2] & 0xFFFF, 0x2311);
    assert_eq!(m.cpu.sr(), 0x2311);
}

/// ColdFire has no ANDI, ORI or EORI to SR (CFPRM ch. 3 lists them for Dx only): their 68000 opwords are illegal.
#[test]
fn andi_ori_eori_to_sr_are_illegal() {
    for w in [0x027C, 0x007C, 0x0A7C] {
        let (mut m, l) = prepare(&format!(".short {w:#06x},0x0700\nend:"), &["end"]);
        let r = m.run(until(l[0]));
        assert_eq!((r.kind, r.pc), (Stop::Illegal, BASE), "{w:#06x}: {r:?}");
    }
}

#[test]
fn movec_writes_the_control_registers() {
    // Reset values (RM Table 3-1).
    let (m, _) = prepare("end:", &["end"]);
    assert_eq!(
        (m.cpu.cacr, m.cpu.asid, m.cpu.mmubar, m.cpu.rgpiobar),
        (0, 0, 0, 0x8C00_0034)
    );

    // ACR4-7 and RGPIOBAR (RM Table 3-1) are hand-encoded: the assembler does not name them.
    let src = "move.l #0x11,%d1\nmovec %d1,%cacr\nmove.l #0x22,%d1\nmovec %d1,%asid\n\
               move.l #0x33,%a1\nmovec %a1,%acr0\nmovec %a1,%acr3\n.short 0x4e7b,0x900c\n.short 0x4e7b,0x900f\n\
               move.l #0x44,%d1\nmovec %d1,%mmubar\n.short 0x4e7b,0x1009\nmove.l #0x55,%d1\nmovec %d1,%rambar\nend:";
    let (mut m, l) = prepare(src, &["end"]);
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.cpu.cacr, 0x11);
    assert_eq!(m.cpu.asid, 0x22);
    assert_eq!(m.cpu.acr, [0x33, 0, 0, 0x33, 0x33, 0, 0, 0x33]);
    assert_eq!(
        (m.cpu.mmubar, m.cpu.rgpiobar, m.cpu.rambar),
        (0x44, 0x44, 0x55)
    );

    // A control register this chip does not have (RAMBAR0, 0xC04): the manual leaves the result undefined.
    let (mut m, l) = prepare(".short 0x4e7b,0x0c04\nend:", &["end"]);
    let r = m.run(until(l[0]));
    assert_eq!((r.kind, r.pc), (Stop::Unmodelled, BASE), "{r:?}");
    assert!(r.detail.contains("movec"), "{r:?}");
}

#[test]
fn vbr_low_bits_ignored() {
    let src = "move.l #0x40212345,%d0\nmovec %d0,%vbr\ntrap #2\nend: nop\nhandler: nop";
    let (mut m, l) = prepare(src, &["handler"]);
    m.cpu.vbr = 0;
    set_vector(&mut m, 34, l[0]);
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(
        m.cpu.vbr, 0x4020_0000,
        "VBR[19:0] are not implemented (RM 3.2.8)"
    );
}

#[test]
fn move_usp_and_s_bit_swap_stacks() {
    let src = "moveq #0x20,%d0\nmovec %d0,%cacr\nmove.l #0x40800000,%a0\nmove.l %a0,%usp\nmove.l %usp,%a1\n\
               move.w #0x0000,%sr\nuser: moveq #1,%d1\ntrap #0\nend: nop\nhandler: nop";
    let (mut m, l) = prepare(src, &["handler", "user"]);
    set_vector(&mut m, 32, l[0]);
    let r = m.run(until(l[1]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.cpu.a[1], USP);
    assert_eq!(
        (m.cpu.a[7], m.cpu.other_sp),
        (USP, STACK),
        "user mode: A7 is the USP"
    );
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(
        (m.cpu.a[7], m.cpu.other_sp),
        (STACK - 8, USP),
        "the exception switches to the SSP"
    );

    // CACR[EUSP] clear (the reset value): one stack pointer, no swap (RM 3.2.3).
    let (mut m, l) = prepare(
        "move.w #0x0000,%sr\ntrap #0\nend: nop\nhandler: nop",
        &["handler"],
    );
    set_vector(&mut m, 32, l[0]);
    m.cpu.other_sp = USP;
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!((m.cpu.a[7], m.cpu.other_sp), (STACK - 8, USP));
}

#[test]
fn move_usp_with_eusp_clear() {
    // guess: CFPRM ch. 8 says MOVE to/from USP execute correctly when CACR[EUSP] is set and is silent when it is
    // clear; both reach OTHER_A7 either way, and A7 itself is untouched.
    let (mut m, l) = prepare(
        "move.l #0x40800000,%a0\nmove.l %a0,%usp\nmove.l %usp,%a1\nend:",
        &["end"],
    );
    assert_eq!(m.cpu.cacr & EUSP, 0);
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!((m.cpu.other_sp, m.cpu.a[1], m.cpu.a[7]), (USP, USP, STACK));
}

#[test]
fn strldsr_pushes_sr_and_loads_the_immediate() {
    // STRLDSR #0x2700 is hand-encoded: the assembler accepts it only for ISA_A+ parts.
    let (mut m, l) = prepare(".short 0x40e7,0x46fc,0x2700\nend:", &["end"]);
    m.cpu.set_sr(0x2315);
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.cpu.a[7], STACK - 4);
    assert_eq!(long(&mut m, STACK - 4), 0x0000_2315, "zero-filled SR");
    assert_eq!(m.cpu.sr(), 0x2700);

    // guess: CFPRM ch. 8 gives the encoding as 0x40E7, 0x46FC, #imm and does not say what another second word
    // does; 0x40E7 followed by anything else is an illegal instruction, with nothing pushed.
    let (mut m, l) = prepare(".short 0x40e7,0x46fd,0x2700\nend:", &["end"]);
    let r = m.run(until(l[0]));
    assert_eq!((r.kind, r.pc), (Stop::Illegal, BASE), "{r:?}");
    assert_eq!((m.cpu.a[7], m.cpu.sr()), (STACK, 0x2700));

    // An immediate with S clear is a privilege violation; nothing is pushed.
    let (mut m, l) = prepare(".short 0x40e7,0x46fc,0x0700\nend:", &["end"]);
    let r = m.run(until(l[0]));
    assert_eq!((r.kind, r.pc), (Stop::Exception(8), BASE), "{r:?}");
    assert_eq!((m.cpu.a[7], m.cpu.sr()), (STACK, 0x2700));
}

#[test]
fn odd_pc_is_an_address_error() {
    // JMP to an odd address: the stacked PC is the jump (RM 3.3.4.2, Table 3-5 "Fault").
    let (mut m, l) = prepare("jmp (%a2)\nend:", &["end"]);
    m.cpu.a[2] = BASE + 0x201;
    let r = m.run(until(l[0]));
    assert_eq!((r.kind, r.pc), (Stop::Exception(3), BASE), "{r:?}");
    assert_eq!(m.cpu.pc, BASE);

    // BRA with an odd byte displacement.
    let (mut m, l) = prepare(".short 0x6001\nnop\nend:", &["end"]);
    let r = m.run(until(l[0]));
    assert_eq!((r.kind, r.pc), (Stop::Exception(3), BASE), "{r:?}");

    // RTS to an odd return address: the return PC stays on the stack.
    let (mut m, l) = prepare("rts\nend:", &["end"]);
    m.mem.write(STACK - 4, Size::Long, BASE + 0x301).unwrap();
    m.cpu.a[7] = STACK - 4;
    let r = m.run(until(l[0]));
    assert_eq!((r.kind, r.pc), (Stop::Exception(3), BASE), "{r:?}");
    assert_eq!(m.cpu.a[7], STACK - 4);

    // JSR: when the run stops on the exception, the registers are as before the JSR (A7 unchanged) so the run can
    // resume; the pushed longword may stay in memory below A7.
    let (mut m, l) = prepare("jsr (%a2)\nend:", &["end"]);
    m.cpu.a[2] = BASE + 0x201;
    let r = m.run(until(l[0]));
    assert_eq!((r.kind, r.pc), (Stop::Exception(3), BASE), "{r:?}");
    assert_eq!(m.cpu.a[7], STACK);

    // guess: a PC that is already odd (here set from outside; RTE can restore one) faults at the fetch, and the
    // stacked PC is the odd address itself.
    let (mut m, l) = prepare("nop\nend:", &["end"]);
    m.cpu.pc = BASE + 1;
    let r = m.run(until(l[0]));
    assert_eq!((r.kind, r.pc), (Stop::Exception(3), BASE + 1), "{r:?}");
}

/// Measured on an MCF54415: a jump to an odd address gives the frame word 0x440C2700 (format 4, vector 3, FS 0100
/// "error on instruction fetch", SR 0x2700) with the jump instruction as the stacked PC.
#[test]
fn odd_jump_address_error_is_an_instruction_fetch_error() {
    let (mut m, l) = prepare(
        "lea 1f+1,%a0\njmp_insn: jmp (%a0)\n1: moveq #2,%d0\nhandler: nop",
        &["jmp_insn", "handler"],
    );
    set_vector(&mut m, 3, l[1]);
    m.policy.stop_on[3] = false;
    let r = m.run(until(l[1]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.cpu.a[7], STACK - 8);
    assert_eq!(long(&mut m, STACK - 8), 0x440C_2700);
    assert_eq!(long(&mut m, STACK - 8), fv(4, 3, 0x2700) | FS_FETCH);
    assert_eq!(long(&mut m, STACK - 4), l[0]);
}

/// Measured on an MCF54415: a word-sized index, a scale factor of 8 and a full-format extension word (RM 3.3.4.2)
/// each give the frame word 0x4C0C2704 (format 4, vector 3, FS 1100 "error on operand read", SR 0x2704 after the
/// MOVEQ #0 that set Z) with the faulting instruction as the stacked PC.
#[test]
fn bad_index_address_error_is_an_operand_read_error() {
    // move.l (0,%a0,%d1.w),%d2 / (0,%a0,%d1.l*8),%d2 / full format with base and index suppressed.
    for ext in [0x1000u16, 0x1E00, 0x19D0] {
        let (mut m, l) = prepare(
            &format!(
                "lea 0x40180000,%a0\nmoveq #0,%d1\nfault: .short 0x2430,{ext:#06x}\nmoveq #2,%d0\nhandler: nop"
            ),
            &["fault", "handler"],
        );
        set_vector(&mut m, 3, l[1]);
        m.policy.stop_on[3] = false;
        let r = m.run(until(l[1]));
        assert_eq!(r.kind, Stop::UntilPc, "{ext:#06x}: {r:?}");
        assert_eq!(m.cpu.a[7], STACK - 8, "{ext:#06x}");
        assert_eq!(long(&mut m, STACK - 8), 0x4C0C_2704, "{ext:#06x}");
        assert_eq!(
            long(&mut m, STACK - 8),
            fv(4, 3, 0x2704) | FS_OPERAND,
            "{ext:#06x}"
        );
        assert_eq!(long(&mut m, STACK - 4), l[0], "{ext:#06x}");
    }
}

#[test]
fn address_error_frames_for_rts_and_jsr() {
    // RTS: the frame is written above (below in memory) the preserved return PC.
    let (mut m, l) = prepare("rts\nend: nop\nhandler: nop", &["handler"]);
    set_vector(&mut m, 3, l[0]);
    m.policy.stop_on[3] = false;
    m.mem.write(STACK - 4, Size::Long, BASE + 0x301).unwrap();
    m.cpu.a[7] = STACK - 4;
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.cpu.a[7], STACK - 12);
    // Not measured for RTS, JSR or BSR: an odd target is the same cause as the odd JMP measured on an MCF54415
    // (the fetch at the target), so FS is 0100 "error on instruction fetch" (RM Table 3-7).
    assert_eq!(long(&mut m, STACK - 12), fv(4, 3, 0x2700) | FS_FETCH);
    assert_eq!(long(&mut m, STACK - 8), BASE);
    assert_eq!(long(&mut m, STACK - 4), BASE + 0x301);

    // JSR: the return address is pushed first, the frame below it.
    let (mut m, l) = prepare("jsr (%a2)\nend: nop\nhandler: nop", &["handler"]);
    set_vector(&mut m, 3, l[0]);
    m.policy.stop_on[3] = false;
    m.cpu.a[2] = BASE + 0x201;
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.cpu.a[7], STACK - 12);
    assert_eq!(
        long(&mut m, STACK - 12),
        fv(4, 3, 0x2700) | FS_FETCH,
        "FS 0100, as for the measured odd JMP"
    );
    assert_eq!(long(&mut m, STACK - 8), BASE);
    assert_eq!(long(&mut m, STACK - 4), BASE + 2);

    // BSR to an odd target.
    // guess: RM 3.3.4.2 describes JSR only; BSR is taken to push the return address first in the same way, so the
    // frame sits below the pushed return address. The branch is hand-encoded (BSR.W, displacement 0x201).
    let (mut m, l) = prepare(".short 0x6100,0x0201\nend: nop\nhandler: nop", &["handler"]);
    set_vector(&mut m, 3, l[0]);
    m.policy.stop_on[3] = false;
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.cpu.a[7], STACK - 12);
    assert_eq!(
        long(&mut m, STACK - 12),
        fv(4, 3, 0x2700) | FS_FETCH,
        "FS 0100, as for the measured odd JMP"
    );
    assert_eq!(long(&mut m, STACK - 8), BASE);
    assert_eq!(
        long(&mut m, STACK - 4),
        BASE + 4,
        "guess: pushed before the error"
    );

    // A PC that is already odd: the stacked PC is the odd address (guess, see `odd_pc_is_an_address_error`).
    let (mut m, l) = prepare("nop\nend: nop\nhandler: nop", &["handler"]);
    set_vector(&mut m, 3, l[0]);
    m.policy.stop_on[3] = false;
    m.cpu.pc = BASE + 1;
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(
        long(&mut m, STACK - 8),
        fv(4, 3, 0x2700) | FS_FETCH,
        "FS 0100, as for the measured odd JMP"
    );
    assert_eq!(long(&mut m, STACK - 4), BASE + 1);
}

/// RM 3.3.4.14: a fault while processing an exception halts the core.
#[test]
fn odd_handler_address_is_a_fault_on_fault() {
    let (mut m, l) = prepare("trap #0\nend:", &["end"]);
    set_vector(&mut m, 32, BASE + 0x401);
    let r = m.run(until(l[0]));
    assert_eq!(
        (r.kind, r.pc),
        (Stop::Halt("fault-on-fault"), BASE),
        "{r:?}"
    );
    assert_eq!((m.cpu.a[7], m.cpu.sr()), (STACK, 0x2700));
}

#[test]
fn unmapped_vector_table_stops_unmapped() {
    let (mut m, l) = prepare("trap #0\nend:", &["end"]);
    m.cpu.vbr = 0;
    let r = m.run(until(l[0]));
    assert_eq!((r.kind, r.pc), (Stop::Unmapped, BASE), "{r:?}");
    assert!(r.detail.contains("0x00000080"), "{r:?}");
    assert_eq!((m.cpu.a[7], m.cpu.sr()), (STACK, 0x2700));
}

/// RM Table 3-12: misaligned operand accesses are split into aligned ones, not address errors.
#[test]
fn odd_long_read_is_performed_misaligned() {
    let (mut m, l) = prepare("move.l (%a0),%d0\nmove.w %d0,(%a1)\nend:", &["end"]);
    m.mem.load(MEM, &[0x11, 0x22, 0x33, 0x44, 0x55]).unwrap();
    m.cpu.a[0] = MEM + 1;
    m.cpu.a[1] = MEM + 0x11;
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.cpu.d[0], 0x2233_4455);
    assert_eq!(m.mem.peek(MEM + 0x11, 2).unwrap(), [0x44, 0x55]);
}

#[test]
fn fpu_opcode_takes_line_f() {
    // The chip has no FPU (RM Table 3-9, D0[FPU] = 0). fmove.l %d0,%fp0; fbf.w; fsave (%a0); frestore (%a0).
    let rows = [
        ".short 0xf200,0x4000",
        ".short 0xf280,0x0000",
        ".short 0xf310",
        ".short 0xf350",
    ];
    for insn in rows {
        for sr in [0x2700u16, 0x0000] {
            // guess: with no FPU the opword is unimplemented, so FSAVE and FRESTORE take line F in user mode too
            // rather than a privilege violation.
            let (mut m, l) = prepare(&format!("moveq #1,%d0\n{insn}\nend:"), &["end"]);
            m.cpu.set_sr(sr);
            m.cpu.a[0] = MEM;
            let r = m.run(until(l[0]));
            assert_eq!(
                (r.kind, r.pc),
                (Stop::Exception(11), BASE + 2),
                "{insn} sr {sr:#x}: {r:?}"
            );
        }
    }
    let (mut m, l) = prepare(".short 0xf200,0x4000\nend: nop\nhandler: nop", &["handler"]);
    set_vector(&mut m, 11, l[0]);
    m.policy.stop_on[11] = false;
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(long(&mut m, STACK - 8), fv(4, 11, 0x2700));
    assert_eq!(long(&mut m, STACK - 4), BASE);
}

#[test]
fn cau_opcode_stops_unmodelled() {
    // cp0ld (%a2) with its extension word; the opword layout is the assembler's (see isa/v4e.isa).
    let (mut m, l) = prepare(".short 0xfc92,0x0000\nend:", &["end"]);
    let r = m.run(until(l[0]));
    assert_eq!((r.kind, r.pc), (Stop::Unmodelled, BASE), "{r:?}");
    assert_eq!(r.detail, "CAU");
}

#[test]
fn stop_without_events_halts_forever() {
    let (mut m, l) = prepare("moveq #1,%d0\nstop #0x2013\nmoveq #2,%d0\nend:", &["end"]);
    let r = m.run(until(l[0]));
    assert_eq!((r.kind, r.pc), (Stop::HaltedForever, BASE + 6), "{r:?}");
    assert_eq!(m.cpu.pc, BASE + 6, "STOP advances PC past itself");
    assert_eq!(m.cpu.sr(), 0x2013);
    assert_eq!(m.cpu.d[0], 1);
    assert_eq!(m.instructions(), 2);

    // An immediate with S clear is a privilege violation (CFPRM ch. 8 STOP).
    let (mut m, l) = prepare("stop #0x0700\nend:", &["end"]);
    let r = m.run(until(l[0]));
    assert_eq!((r.kind, r.pc), (Stop::Exception(8), BASE), "{r:?}");
    assert_eq!(m.cpu.sr(), 0x2700);

    // RM 3.3.4.6: a STOP whose immediate sets T takes a trace exception with PC after the STOP.
    let (mut m, l) = prepare("stop #0xA000\nend:", &["end"]);
    let r = m.run(until(l[0]));
    assert_eq!((r.kind, r.pc), (Stop::Exception(9), BASE + 4), "{r:?}");
    assert_eq!(m.cpu.sr(), 0xA000);
}

#[test]
fn halt_stops_with_halt() {
    // guess: a GO from the debug module resumes at the next instruction (CFPRM ch. 8 HALT), so the stop reports
    // PC after the HALT.
    let (mut m, l) = prepare("moveq #1,%d0\nhalt\nmoveq #2,%d0\nend:", &["end"]);
    let r = m.run(until(l[0]));
    assert_eq!((r.kind, r.pc), (Stop::Halt("halt"), BASE + 4), "{r:?}");
    assert_eq!(m.cpu.d[0], 1);
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "a later run resumes after it: {r:?}");
    assert_eq!(m.cpu.d[0], 2);
}

#[test]
fn cache_and_debug_hints_do_nothing() {
    let src = "pulse\ncpushl %bc,(%a0)\ncpushl %dc,(%a0)\nintouch %a0\nwddata.l (%a0)\nend:";
    let (mut m, l) = prepare(src, &["end"]);
    m.cpu.a[0] = MEM;
    let before = m.cpu.clone();
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(
        (m.cpu.d, m.cpu.a, m.cpu.sr()),
        (before.d, before.a, before.sr())
    );
    assert_eq!(m.instructions(), 5);
}

#[test]
fn illegal_stops_by_default() {
    let (mut m, l) = prepare("moveq #1,%d0\nillegal\nend:", &["end"]);
    let r: StopReason = m.run(until(l[0]));
    assert_eq!((r.kind, r.pc), (Stop::Illegal, BASE + 2), "{r:?}");
    assert!(r.detail.contains("0x4afc"), "{r:?}");

    let (mut m, l) = prepare(
        "moveq #1,%d0\nillegal\nend: nop\nhandler: nop",
        &["handler"],
    );
    set_vector(&mut m, 4, l[0]);
    m.policy.stop_on[4] = false;
    let r = m.run(until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(long(&mut m, STACK - 8), fv(4, 4, 0x2700));
    assert_eq!(long(&mut m, STACK - 4), BASE + 2, "the illegal instruction");
}

/// RM 3.3.4.16: a faulting instruction leaves the registers as they were, (An)+ and MOVEM loads included.
#[test]
fn a_faulting_instruction_changes_no_register() {
    let (mut m, l) = prepare("move.l (%a0)+,(%a1)+\nend:", &["end"]);
    m.cpu.a[0] = MEM;
    m.cpu.a[1] = 0x4110_0000; // past the end of RAM
    let r = m.run(until(l[0]));
    assert_eq!((r.kind, r.pc), (Stop::Unmapped, BASE), "{r:?}");
    assert_eq!((m.cpu.a[0], m.cpu.a[1]), (MEM, 0x4110_0000));

    let (mut m, l) = prepare("movem.l (%a0),%d0-%d3\nend:", &["end"]);
    m.cpu.a[0] = 0x410F_FFF8; // D0 and D1 readable, D2 not
    m.cpu.d = [10, 11, 12, 13, 0, 0, 0, 0];
    let r = m.run(until(l[0]));
    assert_eq!((r.kind, r.pc), (Stop::Unmapped, BASE), "{r:?}");
    assert_eq!(m.cpu.d[..4], [10, 11, 12, 13]);
}
