mod common;

use common::{BASE, C, MEM, N, STACK, V, Z, run_ok};
use mcf5441x_emu::{Limit, Machine, Stop};

trait MemLong {
    fn mem_long(&self, addr: u32) -> u32;
}

impl MemLong for Machine {
    fn mem_long(&self, addr: u32) -> u32 {
        u32::from_be_bytes(self.mem.peek(addr, 4).unwrap().try_into().unwrap())
    }
}

/// The condition mnemonics in code order 2..15, each with its truth function over (n, z, v, c) written from the
/// Scc table of CFPRM ch. 4.
type Cond = (&'static str, fn(bool, bool, bool, bool) -> bool);
const CONDS: [Cond; 14] = [
    ("hi", |_, z, _, c| !c && !z),
    ("ls", |_, z, _, c| c || z),
    ("cc", |_, _, _, c| !c),
    ("cs", |_, _, _, c| c),
    ("ne", |_, z, _, _| !z),
    ("eq", |_, z, _, _| z),
    ("vc", |_, _, v, _| !v),
    ("vs", |_, _, v, _| v),
    ("pl", |n, _, _, _| !n),
    ("mi", |n, _, _, _| n),
    ("ge", |n, _, v, _| (n && v) || (!n && !v)),
    ("lt", |n, _, v, _| (n && !v) || (!n && v)),
    ("gt", |n, z, v, _| (n && v && !z) || (!n && !v && !z)),
    ("le", |n, z, v, _| z || (n && !v) || (!n && v)),
];

#[test]
fn bcc_all_conditions_taken_and_not() {
    for (name, f) in CONDS {
        let src =
            format!("b{name} 1f\nmoveq #0,%d0\nbra 2f\n1: moveq #1,%d0\n2:").replace("\\n", "\n");
        for bits in 0u8..16 {
            let ccr = bits;
            let (n, z, v, c) = (ccr & N != 0, ccr & Z != 0, ccr & V != 0, ccr & C != 0);
            let m = run_ok(&src, |m| m.cpu.set_ccr(ccr));
            assert_eq!(m.cpu.d[0] == 1, f(n, z, v, c), "b{name} with ccr {ccr:#x}");
        }
    }
}

#[test]
fn bra_displacement_sizes_forward_and_backward() {
    // Byte, word and longword displacements (the longword form is ISA_B), forward over padding and backward.
    let m = run_ok(
        "bra.b 1f\n.word 0x4afc\n1: bra.w 2f\n.word 0x4afc\n2: .word 0x60ff\n.long 3f-.\n.word 0x4afc\n\
         3: moveq #5,%d0\nbra.b 5f\n4: moveq #9,%d1\nbra.b 6f\n5: bra.w 4b\n6:",
        |_| {},
    );
    assert_eq!((m.cpu.d[0], m.cpu.d[1]), (5, 9));
    let m = run_ok(
        ".word 0x66ff\n.long 1f-.\nlea 1(%a0),%a3\n1: .word 0x67ff\n.long 2f-.\nlea 1(%a0),%a4\n2:",
        |m| m.cpu.set_ccr(Z),
    );
    assert_eq!(
        (m.cpu.a[3], m.cpu.a[4]),
        (1, 0),
        "bne.l not taken, beq.l taken"
    );
    // The base is the opword address + 2: a word branch over exactly one nop.
    let m = run_ok("bra.w 1f\nnop\n1: moveq #3,%d0", |_| {});
    assert_eq!(m.cpu.d[0], 3);
    assert_eq!(m.instructions(), 2, "bra.w and moveq, not the nop");
}

#[test]
fn bsr_rts_round_trip() {
    let m = run_ok(
        "bsr.b sub\nmoveq #1,%d1\nbra.b done\nsub: moveq #7,%d0\nrts\ndone:\n\
         bsr.w sub2\nbra.w done2\nsub2: addq.l #1,%d2\nrts\ndone2:\n\
         .word 0x61ff\n.long sub3-.\nbra.b done3\nsub3: addq.l #2,%d3\nrts\ndone3:",
        |_| {},
    );
    assert_eq!(m.cpu.d[..4], [7, 1, 1, 2]);
    assert_eq!(m.cpu.a[7], STACK);
    // The return address that BSR pushed is the instruction after it.
    let m = run_ok("bsr.b sub\nnop\nsub: nop", |_| {});
    assert_eq!(m.cpu.a[7], STACK - 4);
    assert_eq!(m.mem_long(STACK - 4), BASE + 2);
    let m = run_ok("bsr.w sub\nnop\nsub: nop", |_| {});
    assert_eq!(m.mem_long(STACK - 4), BASE + 4);
    let m = run_ok(".word 0x61ff\n.long sub-.\nnop\nsub: nop", |_| {});
    assert_eq!(m.mem_long(STACK - 4), BASE + 6);
}

/// Low RAM for the absolute-short case: the subroutine lives at `LOW`.
const LOW: u32 = 0x1000;

#[test]
fn jsr_through_every_control_mode() {
    // `sub` is at BASE + 0x40 in the code image (`.org`); each case calls it a different way and checks that the
    // return address is the instruction after the JSR and that the subroutine ran.
    let sub = BASE + 0x40;
    let cases: [(&str, usize); 7] = [
        ("jsr (%a2)", 2),
        ("jsr 8(%a2)", 4),
        ("jsr (4,%a2,%d3.l)", 4),
        ("jsr 0x40100040", 6),
        ("jsr %pc@(sub)", 4),
        ("jsr %pc@(sub,%d3.l*4)", 4),
        ("jsr 0x1000:w", 4),
    ];
    for (i, (jsr, len)) in cases.iter().enumerate() {
        // d3 = 4 offsets the indexed forms: the base is chosen so that the target is `sub`.
        let src =
            format!("{jsr}\nmoveq #1,%d1\nbra.w done\n.org 0x40\nsub: moveq #7,%d0\nrts\ndone:");
        let src = if jsr.contains("pc@(sub,") {
            // The scaled index adds d3*4 = 16 to the base, so aim 16 bytes short of `sub`.
            src.replace("pc@(sub,", "pc@(sub-16,")
        } else {
            src
        };
        let mut m = Machine::new();
        let code = common::assemble(&format!("{src}\n.balign 2\nend:\n.long end\n"), BASE);
        let (code, tail) = code.split_at(code.len() - 4);
        let end = u32::from_be_bytes(tail.try_into().unwrap());
        m.mem.map_ram(0x4010_0000, 0x0100_0000);
        m.mem.map_ram(0, 0x8000);
        m.mem.load(BASE, code).unwrap();
        m.mem.load(LOW, &[0x70, 0x07, 0x4e, 0x75]).unwrap(); // moveq #7,%d0 ; rts
        m.cpu.pc = BASE;
        m.cpu.a[7] = STACK;
        m.cpu.d[3] = 4;
        m.cpu.a[2] = sub; // (A2)
        if i == 1 {
            m.cpu.a[2] = sub - 8;
        } else if i == 2 {
            m.cpu.a[2] = sub - 4 - 4;
        }
        let mut pushed = 0;
        // Step the JSR alone first: it must push the return address and land on the target.
        let r = m.run(Limit {
            max_instructions: Some(1),
            ..Limit::default()
        });
        assert_eq!(r.kind, Stop::InstructionLimit, "{jsr}: {r:?}");
        let target = if i == 6 { LOW } else { sub };
        assert_eq!(m.cpu.pc, target, "{jsr}");
        assert_eq!(m.cpu.a[7], STACK - 4, "{jsr}");
        pushed |= m.mem_long(STACK - 4);
        assert_eq!(pushed, BASE + *len as u32, "{jsr}: return address");
        let r = m.run(Limit {
            until_pc: Some(end),
            ..Limit::default()
        });
        assert_eq!(r.kind, Stop::UntilPc, "{jsr}: {r:?}");
        assert_eq!((m.cpu.d[0], m.cpu.d[1], m.cpu.a[7]), (7, 1, STACK), "{jsr}");
    }
}

#[test]
fn jmp_through_control_modes() {
    for (jmp, a2) in [
        ("jmp (%a2)", 0),
        ("jmp 6(%a2)", 6),
        ("jmp 0x40100040", 0),
        ("jmp %pc@(there)", 0),
    ] {
        let src = format!("{jmp}\nmoveq #1,%d1\n.org 0x40\nthere: moveq #7,%d0");
        let m = run_ok(&src, |m| m.cpu.a[2] = BASE + 0x40 - a2);
        assert_eq!((m.cpu.d[0], m.cpu.d[1], m.cpu.a[7]), (7, 0, STACK), "{jmp}");
    }
}

#[test]
fn lea_and_pea() {
    let m = run_ok(
        "lea (%a2),%a3\nlea -4(%a2),%a4\nlea (8,%a2,%d2.l*2),%a5\nlea 0x40100040,%a6\nlea %pc@(here),%a0\nhere:",
        |m| (m.cpu.a[2], m.cpu.d[2]) = (MEM + 0x10, 3),
    );
    assert_eq!(m.cpu.a[3], MEM + 0x10);
    assert_eq!(m.cpu.a[4], MEM + 0xC);
    assert_eq!(m.cpu.a[5], MEM + 0x10 + 8 + 6);
    assert_eq!(m.cpu.a[6], 0x4010_0040);
    assert_eq!(m.cpu.a[0], m.cpu.pc);
    let m = run_ok("pea 12(%a2)\npea (%a2)", |m| m.cpu.a[2] = MEM);
    assert_eq!(m.cpu.a[7], STACK - 8);
    assert_eq!(m.mem_long(STACK - 4), MEM + 12);
    assert_eq!(m.mem_long(STACK - 8), MEM);
    // The flags are not touched.
    let m = run_ok("lea (%a2),%a3\npea (%a2)", |m| m.cpu.set_ccr(0x1F));
    assert_eq!(m.cpu.ccr(), 0x1F);
}

#[test]
fn link_unlk_frame() {
    let m = run_ok("link %a6,#-8", |m| m.cpu.a[6] = 0xAAAA_0001);
    assert_eq!(m.cpu.a[7], STACK - 4 - 8);
    assert_eq!(m.cpu.a[6], STACK - 4);
    assert_eq!(m.mem_long(STACK - 4), 0xAAAA_0001);
    let m = run_ok("link %a6,#-8\nunlk %a6", |m| m.cpu.a[6] = 0xAAAA_0001);
    assert_eq!(m.cpu.a[7], STACK);
    assert_eq!(m.cpu.a[6], 0xAAAA_0001);
    // A positive displacement and a nested frame.
    let m = run_ok("link %a5,#-4\nlink %a6,#-16\nunlk %a6\nunlk %a5", |m| {
        (m.cpu.a[5], m.cpu.a[6]) = (0x1111_1111, 0x2222_2222)
    });
    assert_eq!(
        (m.cpu.a[5], m.cpu.a[6], m.cpu.a[7]),
        (0x1111_1111, 0x2222_2222, STACK)
    );
    let m = run_ok("link %a6,#4", |m| m.cpu.a[6] = 1);
    assert_eq!(m.cpu.a[7], STACK - 4 + 4);
}

#[test]
fn link_and_unlk_with_a7() {
    // guess: CFPRM ch. 4 gives LINK as "SP - 4 -> SP; Ay -> (SP); SP -> Ay; SP + dn -> SP" and does not call out
    // Ay = A7. Read in that order the value pushed is the decremented SP, A7 ends at that SP plus the displacement.
    let m = run_ok("link %a7,#-8", |_| {});
    assert_eq!(m.mem_long(STACK - 4), STACK - 4);
    assert_eq!(m.cpu.a[7], STACK - 4 - 8);

    // guess: UNLK does not call out Ax = A7 either; A7 + 4 is overwritten by the popped longword, which wins.
    let m = run_ok("unlk %a7", |m| {
        m.mem
            .write(STACK, mcf5441x_emu::Size::Long, 0x4012_3450)
            .unwrap()
    });
    assert_eq!(m.cpu.a[7], 0x4012_3450);
}

#[test]
fn movem_store_load_order() {
    // The lowest register at the lowest address, D0-D7 then A0-A7.
    let all = "movem.l %d0-%d7/%a0-%a7,(%a1)";
    let m = run_ok(all, |m| {
        for i in 0..8 {
            m.cpu.d[i] = 0x1000 + i as u32;
            m.cpu.a[i] = 0x2000 + i as u32;
        }
        m.cpu.a[1] = MEM;
    });
    for i in 0..8u32 {
        assert_eq!(m.mem_long(MEM + 4 * i), 0x1000 + i);
    }
    assert_eq!(m.mem_long(MEM + 32), 0x2000);
    assert_eq!(m.mem_long(MEM + 36), MEM, "A1 is stored as it was");
    assert_eq!(m.mem_long(MEM + 60), 0x2007, "A7 is the last longword");
    // A subset with (d16,An), including a negative displacement.
    let m = run_ok("movem.l %d1/%d3/%a0/%a6,-8(%a2)", |m| {
        (m.cpu.d[1], m.cpu.d[3], m.cpu.a[0], m.cpu.a[6]) = (0xD1, 0xD3, 0xA0, 0xA6);
        m.cpu.a[2] = MEM + 0x100;
    });
    let at = MEM + 0x100 - 8;
    assert_eq!(
        [0, 4, 8, 12].map(|o| m.mem_long(at + o)),
        [0xD1, 0xD3, 0xA0, 0xA6]
    );
    assert_eq!(m.mem_long(at + 16), 0, "nothing past the last register");
    // Loading, with the registers outside the mask kept; no flag changes.
    let m = run_ok("movem.l 4(%a2),%d4/%d5/%a3", |m| {
        m.cpu.a[2] = MEM;
        for (i, v) in [9u32, 0x44, 0x55, 0xA3].iter().enumerate() {
            m.mem
                .write(MEM + 4 * i as u32, mcf5441x_emu::Size::Long, *v)
                .unwrap();
        }
        m.cpu.d[6] = 0x66;
        m.cpu.set_ccr(0x15);
    });
    assert_eq!(
        (m.cpu.d[4], m.cpu.d[5], m.cpu.a[3], m.cpu.d[6]),
        (0x44, 0x55, 0xA3, 0x66)
    );
    assert_eq!(m.cpu.ccr(), 0x15);
    let m = run_ok("movem.l (%a2),%d0-%d7/%a0-%a7", |m| {
        m.cpu.a[2] = MEM;
        for i in 0..16u32 {
            m.mem
                .write(MEM + 4 * i, mcf5441x_emu::Size::Long, 0x100 + i)
                .unwrap();
        }
    });
    assert_eq!(
        m.cpu.d,
        [0x100, 0x101, 0x102, 0x103, 0x104, 0x105, 0x106, 0x107]
    );
    assert_eq!(m.cpu.a[7], 0x10F);
    assert_eq!(
        m.cpu.a[2], 0x10A,
        "A2 is loaded from its slot like any register"
    );
}

#[test]
fn nop_changes_nothing_but_pc() {
    let m = run_ok("nop\nnop", |m| m.cpu.set_ccr(0x1F));
    assert_eq!(m.cpu.ccr(), 0x1F);
    assert_eq!(m.instructions(), 2);
    assert_eq!(m.cpu.pc, BASE + 4);
}

#[test]
fn jump_to_an_odd_address_is_an_address_error() {
    // RM 3.3.4.2: the jump takes vector 3 and the run stops on it, PC on the jump.
    let mut m = Machine::new();
    m.mem.map_ram(0x4010_0000, 0x0100_0000);
    m.mem.load(BASE, &[0x4e, 0xd2]).unwrap(); // jmp (%a2)
    m.cpu.pc = BASE;
    m.cpu.a[2] = BASE + 0x201;
    let r = m.run(Limit::default());
    assert_eq!((r.kind, r.pc), (Stop::Exception(3), BASE), "{r:?}");
    assert_eq!(m.cpu.pc, BASE);
}

#[test]
fn btst_flags_survive_a_block_split_at_a_page_end() {
    // BTST is the last instruction before a 4 KiB boundary and BEQ the first after it. Z comes from the bit tested
    // (set bit: Z clear, no branch; clear bit: Z set, branch), whatever the flags were before.
    let src = ".org 0xffe\nbtst %d1,%d0\nbeq 1f\nmoveq #1,%d2\nbra.b 2f\n1: moveq #2,%d2\n2:";
    for (d0, before, want) in [
        (0x08u32, Z, 1u32),
        (0x08, 0, 1),
        (0x00, 0, 2),
        (0x00, Z | N | V | C, 2),
    ] {
        let (m, r) = common::run(src, |m| {
            m.cpu.pc = BASE + 0xFFE;
            m.cpu.d[0] = d0;
            m.cpu.d[1] = 3;
            m.cpu.set_ccr(before);
        });
        assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
        assert_eq!(m.cpu.d[2], want, "d0 {d0:#x}, ccr before {before:#x}");
    }
}
