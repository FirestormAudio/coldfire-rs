mod common;

use mcf5441x_emu::{Limit, Machine, Size, Stop, StopReason};

const BASE: u32 = 0x4010_0000;
const STACK: u32 = 0x4100_0000;
/// Input bytes for the addressing-mode test: the byte at `IN + k` is `k as u8`.
const IN: u32 = 0x4020_0000;
const OUT: u32 = 0x4030_0000;
const DST: u32 = 0x4040_0000;

/// Assemble `src` at `BASE` into a machine with RAM at `0x40100000..0x41100000` and A7 at `STACK`. The source
/// defines an `end` label; returns the machine and that label's address.
fn prepare(src: &str) -> (Machine, u32) {
    // The trailing long holds the address of `end`; it is never executed.
    let code = common::assemble(&format!("{src}\n.balign 2\n.long end\n"), BASE);
    let (code, tail) = code.split_at(code.len() - 4);
    let end = u32::from_be_bytes(tail.try_into().unwrap());
    let mut m = Machine::new();
    m.mem.map_ram(0x4010_0000, 0x0100_0000);
    m.mem.load(BASE, code).unwrap();
    m.cpu.pc = BASE;
    m.cpu.a[7] = STACK;
    (m, end)
}

fn until(end: u32) -> Limit {
    Limit {
        until_pc: Some(end),
        ..Limit::default()
    }
}

/// Run a prepared machine to `end`, which it must reach.
fn finish(m: &mut Machine, end: u32) {
    let r = m.run(until(end));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(r.pc, end);
    assert_eq!(m.cpu.pc, end);
}

fn run(src: &str) -> Machine {
    let (mut m, end) = prepare(src);
    finish(&mut m, end);
    m
}

const X: u8 = 0x10;
const N: u8 = 0x08;
const Z: u8 = 0x04;
const V: u8 = 0x02;
const C: u8 = 0x01;

#[test]
fn moveq_sets_n_and_z() {
    let (mut m, end) = prepare("moveq #-1,%d0\nend:");
    m.cpu.set_sr(0x2700 | (V | C) as u16);
    finish(&mut m, end);
    assert_eq!(m.cpu.d[0], 0xFFFF_FFFF);
    assert_eq!(m.cpu.ccr(), N, "N=1 Z=0 V=0 C=0");

    let m = run("moveq #0,%d1\nend:");
    assert_eq!(m.cpu.d[1], 0);
    assert_eq!(m.cpu.ccr(), Z);

    let m = run("moveq #127,%d2\nmoveq #-128,%d3\nend:");
    assert_eq!(m.cpu.d[2], 0x7F);
    assert_eq!(m.cpu.d[3], 0xFFFF_FF80);
}

/// The big-endian long at `IN + k`.
fn long_at(k: u32) -> u32 {
    u32::from_be_bytes([k as u8, (k + 1) as u8, (k + 2) as u8, (k + 3) as u8])
}

#[test]
fn move_l_through_every_addressing_mode() {
    let src = "
        move.l %d7,%d0                  | Dn
        move.l %d0,(%a2)+
        move.l %a3,%d0                  | An
        move.l %d0,(%a2)+
        move.l (%a0),%d0                | (An)
        move.l %d0,(%a2)+
        move.l (%a0)+,%d0               | (An)+
        move.l %d0,(%a2)+
        move.l (%a0)+,%d0
        move.l %d0,(%a2)+
        move.l -(%a0),%d0               | -(An)
        move.l %d0,(%a2)+
        move.l (8,%a0),%d0              | (d16,An)
        move.l %d0,(%a2)+
        move.l (-6,%a0),%d0
        move.l %d0,(%a2)+
        move.l (1,%a0,%d1.l),%d0        | (d8,An,Xi) scale 1
        move.l %d0,(%a2)+
        move.l (1,%a0,%d1.l*2),%d0      | scale 2
        move.l %d0,(%a2)+
        move.l (-128,%a0,%d1.l*4),%d0   | scale 4, negative d8
        move.l %d0,(%a2)+
        move.l (127,%a0,%a1.l*4),%d0    | address-register index
        move.l %d0,(%a2)+
        move.l (0,%a0,%d2.l*4),%d0      | negative index
        move.l %d0,(%a2)+
        move.l (-32752).w,%d0           | abs.W, sign-extended to 0xFFFF8010
        move.l %d0,(%a2)+
        move.l 0x40200010,%d0           | abs.L
        move.l %d0,(%a2)+
        move.l t(%pc),%d0               | (d16,PC)
        move.l %d0,(%a2)+
        move.l (t,%pc,%d1.l*4),%d0      | (d8,PC,Xi)
        move.l %d0,(%a2)+
        move.l #0x12345678,%d0          | #imm
        move.l %d0,(%a2)+

        move.l %d3,(%a4)                | destinations: (An)
        move.l %d3,(%a5)+               | (An)+
        move.l %d4,-(%a6)               | -(An)
        move.l %d5,(16,%a4)             | (d16,An)
        move.l %d3,(0x20,%a4,%d1.l*4)   | (d8,An,Xi)
        move.l %d4,(0x30,%a4,%d1.l*2)
        move.l %d5,(0x40,%a4,%d1.l)
        move.l %d4,(-32736).w           | abs.W
        move.l %d5,0x40400080           | abs.L
    end:
    t:  .long 0x0BADC0DE, 0x11111111, 0x22222222
    ";
    let (mut m, end) = prepare(src);
    m.mem.map_ram(0xFFFF_8000, 0x8000);
    m.mem.write(0xFFFF_8010, Size::Long, 0xCAFE_F00D).unwrap();
    for k in 0..512 {
        m.mem.write(IN + k, Size::Byte, k & 0xFF).unwrap();
    }
    m.cpu.a[0] = IN + 0x80;
    m.cpu.a[1] = 8;
    m.cpu.a[2] = OUT;
    m.cpu.a[3] = 0x3333_3333;
    m.cpu.a[4] = DST;
    m.cpu.a[5] = DST + 0x100;
    m.cpu.a[6] = DST + 0x200;
    m.cpu.d[1] = 2;
    m.cpu.d[2] = -3i32 as u32;
    m.cpu.d[3] = 0xD3D3_D3D3;
    m.cpu.d[4] = 0xD4D4_D4D4;
    m.cpu.d[5] = 0xD5D5_D5D5;
    m.cpu.d[7] = 0x7777_7777;
    finish(&mut m, end);

    let expected = [
        0x7777_7777,
        0x3333_3333,
        long_at(0x80),
        long_at(0x80),
        long_at(0x84),
        long_at(0x84),  // -(A0) back to IN + 0x84
        long_at(0x8C),  // 8 + 0x84
        long_at(0x7E),  // -6 + 0x84
        long_at(0x87),  // 1 + 0x84 + 2
        long_at(0x89),  // 1 + 0x84 + 2*2
        long_at(0x0C),  // -128 + 0x84 + 2*4
        long_at(0x123), // 127 + 0x84 + 8*4
        long_at(0x78),  // 0x84 + (-3)*4
        0xCAFE_F00D,
        long_at(0x10),
        0x0BAD_C0DE,
        0x2222_2222, // t + 2*4
        0x1234_5678,
    ];
    for (i, want) in expected.iter().enumerate() {
        let got = m.mem.read(OUT + 4 * i as u32, Size::Long).unwrap();
        assert_eq!(got, *want, "source mode #{i}: {got:#010x} != {want:#010x}");
    }
    assert_eq!(m.cpu.a[0], IN + 0x84);
    assert_eq!(m.cpu.a[2], OUT + 4 * expected.len() as u32);

    let at = |m: &mut Machine, a: u32| m.mem.read(a, Size::Long).unwrap();
    assert_eq!(at(&mut m, DST), 0xD3D3_D3D3);
    assert_eq!(at(&mut m, DST + 0x100), 0xD3D3_D3D3);
    assert_eq!(m.cpu.a[5], DST + 0x104);
    assert_eq!(at(&mut m, DST + 0x1FC), 0xD4D4_D4D4);
    assert_eq!(m.cpu.a[6], DST + 0x1FC);
    assert_eq!(at(&mut m, DST + 0x10), 0xD5D5_D5D5);
    assert_eq!(at(&mut m, DST + 0x28), 0xD3D3_D3D3);
    assert_eq!(at(&mut m, DST + 0x34), 0xD4D4_D4D4);
    assert_eq!(at(&mut m, DST + 0x42), 0xD5D5_D5D5);
    assert_eq!(at(&mut m, 0xFFFF_8020), 0xD4D4_D4D4);
    assert_eq!(at(&mut m, DST + 0x80), 0xD5D5_D5D5);
    assert_eq!(m.cpu.ccr(), N, "flags of the last move (0xD5D5D5D5)");
}

#[test]
fn move_b_w_write_the_low_part_and_flag_on_their_size() {
    let (mut m, end) = prepare("move.b %d0,%d1\nmove.w %d0,%d2\nend:");
    m.cpu.d[0] = 0x0001_8080;
    m.cpu.d[1] = 0xAAAA_AAAA;
    m.cpu.d[2] = 0xAAAA_AAAA;
    finish(&mut m, end);
    assert_eq!(m.cpu.d[1], 0xAAAA_AA80);
    assert_eq!(m.cpu.d[2], 0xAAAA_8080);
    assert_eq!(m.cpu.ccr(), N);

    let (mut m, end) = prepare("move.b %d0,%d1\nend:");
    m.cpu.d[0] = 0x0000_0100;
    finish(&mut m, end);
    assert_eq!(m.cpu.ccr(), Z, "the byte is zero although the long is not");

    let (mut m, end) =
        prepare("move.b #0x81,(%a0)+\nmove.w #0x7001,(%a0)+\nmove.b (%a1)+,%d3\nend:");
    m.cpu.a[0] = DST;
    m.cpu.a[1] = DST;
    finish(&mut m, end);
    assert_eq!(m.mem.read(DST, Size::Long).unwrap() >> 8, 0x81_7001);
    assert_eq!(m.cpu.a[0], DST + 3);
    assert_eq!(m.cpu.a[1], DST + 1);
    assert_eq!(m.cpu.d[3] & 0xFF, 0x81);
    assert_eq!(m.cpu.ccr(), N);
}

/// CFPRM ch. 2 (2.2.4, 2.2.5): the stack pointer is treated exactly like any other address register, so a byte
/// access through -(A7) or (A7)+ moves it by one.
#[test]
fn move_b_to_a7_predecrement_moves_one() {
    let (mut m, end) = prepare("move.b %d0,-(%a7)\nend:");
    m.cpu.d[0] = 0x1234_5678;
    finish(&mut m, end);
    assert_eq!(m.cpu.a[7], STACK - 1);
    assert_eq!(m.mem.read(STACK - 1, Size::Byte).unwrap(), 0x78);

    let (mut m, end) = prepare("move.b (%a7)+,%d1\nend:");
    m.mem.write(STACK, Size::Byte, 0x5A).unwrap();
    finish(&mut m, end);
    assert_eq!(m.cpu.a[7], STACK + 1);
    assert_eq!(m.cpu.d[1], 0x5A);
}

#[test]
fn move_leaves_x_alone() {
    // 0xFFFFFFFF + 1 carries out: X = C = 1, Z = 1.
    let m = run("moveq #-1,%d0\naddq.l #1,%d0\nend:");
    assert_eq!(m.cpu.d[0], 0);
    assert_eq!(m.cpu.ccr(), X | Z | C);

    let (mut m, end) = prepare("moveq #-1,%d0\naddq.l #1,%d0\nmove.l %d1,%d2\nend:");
    m.cpu.d[1] = 5;
    finish(&mut m, end);
    assert_eq!(m.cpu.d[2], 5);
    assert_eq!(
        m.cpu.ccr(),
        X,
        "MOVE clears V and C, sets N and Z from 5, keeps X"
    );

    // X set through SR also survives a MOVE, and a clear X stays clear.
    let (mut m, end) = prepare("move.l %d1,%d2\nmoveq #0,%d3\nend:");
    m.cpu.set_sr(0x2700 | X as u16);
    finish(&mut m, end);
    assert_eq!(m.cpu.ccr(), X | Z);
    let (mut m, end) = prepare("moveq #-1,%d3\nend:");
    m.cpu.set_sr(0x2700 | (N | Z | V | C) as u16);
    finish(&mut m, end);
    assert_eq!(m.cpu.ccr(), N);
}

#[test]
fn movea_doesnt_touch_ccr() {
    let (mut m, end) = prepare("movea.l %d0,%a1\nmovea.w %d0,%a2\nmovea.l #0,%a3\nend:");
    m.cpu.d[0] = 0x8000_8000;
    m.cpu.set_sr(0x2700 | 0x1F);
    finish(&mut m, end);
    assert_eq!(m.cpu.a[1], 0x8000_8000);
    assert_eq!(m.cpu.ccr(), 0x1F);

    // Pending flags from MOVEQ survive too.
    let (mut m, end) = prepare("moveq #0,%d0\nmovea.l #-1,%a1\nend:");
    m.cpu.set_sr(0x2700 | X as u16);
    finish(&mut m, end);
    assert_eq!(m.cpu.a[1], 0xFFFF_FFFF);
    assert_eq!(m.cpu.ccr(), X | Z);
}

#[test]
fn move_w_sign_extends_into_an() {
    let (mut m, end) = prepare(
        "movea.w %d0,%a1\nmove.w %d1,%a2\nmovea.w #0x7fff,%a3\nmovea.w (%a0),%a4\nmove.w %a1,%d2\nend:",
    );
    m.cpu.d[0] = 0x1234_8001;
    m.cpu.d[1] = 0x0000_7FFE;
    m.cpu.d[2] = 0xAAAA_AAAA;
    m.cpu.a[0] = DST;
    m.mem.write(DST, Size::Word, 0xFFFE).unwrap();
    finish(&mut m, end);
    assert_eq!(m.cpu.a[1], 0xFFFF_8001);
    assert_eq!(m.cpu.a[2], 0x0000_7FFE);
    assert_eq!(m.cpu.a[3], 0x0000_7FFF);
    assert_eq!(m.cpu.a[4], 0xFFFF_FFFE);
    assert_eq!(
        m.cpu.d[2], 0xAAAA_8001,
        "MOVE.W An,Dn takes the low word of An"
    );
    assert_eq!(m.cpu.ccr(), N);
}

#[test]
fn mvs_mvz_extend() {
    let (mut m, end) = prepare("mvs.b %d0,%d1\nend:");
    m.cpu.d[0] = 0x0000_0080;
    finish(&mut m, end);
    assert_eq!(m.cpu.d[1], 0xFFFF_FF80);
    assert_eq!(m.cpu.ccr(), N);

    let m = run("mvs.w #-32767,%d2\nend:");
    assert_eq!(m.cpu.d[2], 0xFFFF_8001);
    assert_eq!(m.cpu.ccr(), N);

    let m = run("mvs.b #-128,%d3\nend:");
    assert_eq!(m.cpu.d[3], 0xFFFF_FF80);

    let m = run("mvs.w #0x7fff,%d3\nend:");
    assert_eq!(m.cpu.d[3], 0x7FFF);
    assert_eq!(m.cpu.ccr(), 0);

    let (mut m, end) = prepare("mvz.b %d0,%d1\nend:");
    m.cpu.d[0] = 0xFFFF_FF80;
    m.cpu.set_sr(0x2700 | (X | V | C) as u16);
    finish(&mut m, end);
    assert_eq!(m.cpu.d[1], 0x80);
    assert_eq!(
        m.cpu.ccr(),
        X,
        "N clear (zero-extended), V and C clear, X kept"
    );

    let (mut m, end) = prepare("mvz.w (%a0),%d2\nend:");
    m.cpu.a[0] = DST;
    m.mem.write(DST, Size::Word, 0x8001).unwrap();
    finish(&mut m, end);
    assert_eq!(m.cpu.d[2], 0x0000_8001);
    assert_eq!(m.cpu.ccr(), 0);

    let (mut m, end) = prepare("mvz.b %d0,%d1\nend:");
    m.cpu.d[0] = 0x1200;
    m.cpu.d[1] = 0xFFFF_FFFF;
    finish(&mut m, end);
    assert_eq!(m.cpu.d[1], 0);
    assert_eq!(m.cpu.ccr(), Z);
}

/// Measured on an MCF54415: A1 = 0x123456F0, `mvs.b %a1,%d1` gives D1 = 0xFFFFFFF0 with N only.
#[test]
fn mvs_b_takes_an_measured() {
    let (mut m, end) = prepare("mvs.b %a1,%d1\nend:");
    m.cpu.a[1] = 0x1234_56F0;
    finish(&mut m, end);
    assert_eq!(m.cpu.d[1], 0xFFFF_FFF0);
    assert_eq!(m.cpu.ccr(), N);
}

/// Measured on an MCF54415: from CCR 0x0F, `mvz.b %a1,%d1` gives D1 = 0xF0 with N, Z, V and C all clear.
#[test]
fn mvz_b_takes_an_measured() {
    let (mut m, end) = prepare("mvz.b %a1,%d1\nend:");
    m.cpu.a[1] = 0x1234_56F0;
    m.cpu.set_sr(0x2700 | 0x0F);
    finish(&mut m, end);
    assert_eq!(m.cpu.d[1], 0x0000_00F0);
    assert_eq!(m.cpu.ccr(), 0);
}

/// Measured on an MCF54415: `cmp.b %a1,%d1` (0xB209) is an illegal instruction.
#[test]
fn cmp_b_with_an_is_illegal_measured() {
    let (mut m, end) = prepare(".short 0xB209\nend:");
    let r = m.run(until(end));
    assert_eq!((r.kind, r.pc), (Stop::Illegal, BASE), "{r:?}");
}

#[test]
fn illegal_opword_stops_with_illegal() {
    let (mut m, end) = prepare("moveq #1,%d0\n.short 0x4AFC\nmoveq #2,%d0\nend:");
    let r = m.run(until(end));
    assert_eq!(r.kind, Stop::Illegal, "{r:?}");
    assert_eq!(r.pc, BASE + 2);
    assert_eq!(m.cpu.pc, BASE + 2);
    assert_eq!(m.cpu.d[0], 1);
    assert_eq!(m.instructions(), 1);

    // An opword no form describes stops the same way.
    let (mut m, end) = prepare(".short 0x4AFB\nend:");
    let r = m.run(until(end));
    assert_eq!((r.kind, r.pc), (Stop::Illegal, BASE), "{r:?}");
}

/// RM 3.3.4.2: a word-sized index register, a scale factor of 8 or a full-format extension word (bit 8 set) is an
/// address error (vector 3).
#[test]
fn word_index_scale_8_and_full_format_are_address_errors() {
    // move.l (0,%a0,%d1.w),%d0 / (0,%a0,%d1.l*8) / full format; the assembler rejects all three for ColdFire.
    for ext in [0x1000u16, 0x1E00, 0x1900] {
        let (mut m, end) = prepare(&format!(
            "moveq #1,%d0\n.short 0x2030, {ext:#06x}\nmoveq #2,%d0\nend:"
        ));
        m.cpu.a[0] = DST;
        let r = m.run(until(end));
        assert_eq!(
            (r.kind, r.pc),
            (Stop::Exception(3), BASE + 2),
            "{ext:#06x}: {r:?}"
        );
        assert_eq!(m.cpu.d[0], 1);
    }
    // The valid long index with scale 4 runs.
    let (mut m, end) = prepare(".short 0x2030, 0x1C00\nend:");
    m.cpu.a[0] = DST;
    finish(&mut m, end);
}

#[test]
fn unmapped_access_stops() {
    let (mut m, end) = prepare("moveq #1,%d0\nmove.l 0x50000000,%d0\nend:");
    let r: StopReason = m.run(until(end));
    assert_eq!((r.kind, r.pc), (Stop::Unmapped, BASE + 2), "{r:?}");
    assert!(r.detail.contains("0x50000000"), "{r:?}");
    assert_eq!(m.cpu.d[0], 1);

    let (mut m, end) = prepare("move.l %d0,(%a0)\nend:");
    m.cpu.a[0] = 0x4110_0000;
    let r = m.run(until(end));
    assert_eq!((r.kind, r.pc), (Stop::Unmapped, BASE), "{r:?}");

    // Running off the end of RAM.
    let mut m = Machine::new();
    m.mem.map_ram(0x4010_0000, 0x1000);
    m.cpu.pc = 0x4010_1000;
    let r = m.run(Limit::default());
    assert_eq!((r.kind, r.pc), (Stop::Unmapped, 0x4010_1000), "{r:?}");
}

#[test]
fn until_pc_and_instruction_limit() {
    let src = "moveq #1,%d0\nmoveq #2,%d1\nmoveq #3,%d2\nmoveq #4,%d3\nmoveq #5,%d4\nend:";
    let (mut m, end) = prepare(src);
    finish(&mut m, end);
    assert_eq!(m.instructions(), 5);
    assert_eq!(m.now(), 5, "one instruction is one tick");
    assert_eq!(m.cpu.d[..5], [1, 2, 3, 4, 5]);

    let (mut m, end) = prepare(src);
    let r = m.run(Limit {
        max_instructions: Some(3),
        ..Limit::default()
    });
    assert_eq!((r.kind, r.pc), (Stop::InstructionLimit, BASE + 6), "{r:?}");
    assert_eq!(m.instructions(), 3);
    assert_eq!(m.cpu.d[..5], [1, 2, 3, 0, 0]);
    // A later run resumes where it stopped.
    let r = m.run(Limit {
        until_pc: Some(end),
        max_instructions: Some(10),
    });
    assert_eq!((r.kind, r.pc), (Stop::UntilPc, end), "{r:?}");
    assert_eq!(m.instructions(), 5);
    assert_eq!(m.cpu.d[..5], [1, 2, 3, 4, 5]);
}

#[test]
fn set_sr_swaps_stacks_on_s_change() {
    let mut m = Machine::new();
    assert_eq!(m.cpu.sr(), 0x2700, "reset: supervisor, IPL 7");
    m.cpu.cacr = 0x20; // EUSP: separate stack pointers (RM 3.2.3)
    m.cpu.a[7] = 0x4100_0000; // SSP
    m.cpu.other_sp = 0x4080_0000; // USP
    m.cpu.set_sr(0x0005);
    assert_eq!(m.cpu.a[7], 0x4080_0000);
    assert_eq!(m.cpu.other_sp, 0x4100_0000);
    assert_eq!(m.cpu.ccr(), 0x05);
    m.cpu.set_sr(0x0010);
    assert_eq!(m.cpu.a[7], 0x4080_0000, "no S change, no swap");
    m.cpu.set_sr(0xFFFF);
    assert_eq!(m.cpu.sr(), 0xB71F, "reserved bits read as zero");
    assert_eq!(m.cpu.a[7], 0x4100_0000);
}
