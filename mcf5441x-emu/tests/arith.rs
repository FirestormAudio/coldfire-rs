mod common;

use mcf5441x_emu::{Limit, Machine, Size, Stop};

const BASE: u32 = 0x4010_0000;
const STACK: u32 = 0x4100_0000;

const X: u8 = 0x10;
const N: u8 = 0x08;
const Z: u8 = 0x04;
const V: u8 = 0x02;
const C: u8 = 0x01;

/// Assemble `src` at `BASE`, set D0, D1, A0 and the CCR, run to the `end` label and return the machine.
fn exec(src: &str, d0: u32, d1: u32, a0: u32, ccr: u8) -> Machine {
    let code = common::assemble(&format!("{src}\n.balign 2\nend:\n.long end\n"), BASE);
    let (code, tail) = code.split_at(code.len() - 4);
    let end = u32::from_be_bytes(tail.try_into().unwrap());
    let mut m = Machine::new();
    m.mem.map_ram(0x4010_0000, 0x0100_0000);
    m.mem.load(BASE, code).unwrap();
    m.cpu.pc = BASE;
    m.cpu.a[7] = STACK;
    m.cpu.d[0] = d0;
    m.cpu.d[1] = d1;
    m.cpu.a[0] = a0;
    m.cpu.set_sr(0x2700 | ccr as u16);
    let r = m.run(Limit {
        until_pc: Some(end),
        ..Limit::default()
    });
    assert_eq!(r.kind, Stop::UntilPc, "{src}: {r:?}");
    m
}

/// One row: operands D0 (destination) and D1 (source), the expected D0 and the expected CCR, all from the CFPRM
/// definition of the instruction.
struct Row(u32, u32, u32, u8);

fn table(insn: &str, ccr_in: u8, rows: &[Row]) {
    for &Row(d0, d1, want, ccr) in rows {
        let m = exec(insn, d0, d1, 0, ccr_in);
        assert_eq!(m.cpu.d[0], want, "{insn} d0={d0:#x} d1={d1:#x}");
        assert_eq!(
            m.cpu.ccr(),
            ccr,
            "{insn} d0={d0:#x} d1={d1:#x}: ccr {:#07b}",
            m.cpu.ccr()
        );
    }
}

#[test]
fn add_l_flags() {
    let rows = [
        Row(0, 0, 0, Z),
        Row(0, 1, 1, 0),
        Row(1, 0xFFFF_FFFF, 0, X | Z | C),
        Row(0x7FFF_FFFF, 1, 0x8000_0000, N | V),
        Row(0x8000_0000, 0x8000_0000, 0, X | Z | V | C),
        Row(0xFFFF_FFFF, 0xFFFF_FFFF, 0xFFFF_FFFE, X | N | C),
        Row(0x7FFF_FFFF, 0x7FFF_FFFF, 0xFFFF_FFFE, N | V),
        Row(0x8000_0000, 0xFFFF_FFFF, 0x7FFF_FFFF, X | V | C),
    ];
    // X is written by ADD: the result must not depend on the X going in.
    table("add.l %d1,%d0", 0, &rows);
    table("add.l %d1,%d0", X | N | Z | V | C, &rows);
    // The same through ADDI (immediates are the same values) and ADD Dn,<ea>.
    table(
        "addi.l #0x7FFFFFFF,%d0",
        0,
        &[
            Row(1, 0, 0x8000_0000, N | V),
            Row(0, 0, 0x7FFF_FFFF, 0),
            Row(0x8000_0000, 0, 0xFFFF_FFFF, N),
            Row(0xFFFF_FFFF, 0, 0x7FFF_FFFE, X | C),
        ],
    );
    let mut m = exec("move.l %d0,-(%a7)\nadd.l %d1,(%a7)", 0x7FFF_FFFF, 1, 0, 0);
    assert_eq!(m.mem.read(STACK - 4, Size::Long).unwrap(), 0x8000_0000);
    assert_eq!(m.cpu.ccr(), N | V);
}

#[test]
fn sub_l_flags() {
    let rows = [
        Row(0, 0, 0, Z),
        Row(1, 1, 0, Z),
        Row(0, 1, 0xFFFF_FFFF, X | N | C),
        Row(0x8000_0000, 1, 0x7FFF_FFFF, V),
        Row(0x7FFF_FFFF, 0xFFFF_FFFF, 0x8000_0000, X | N | V | C),
        Row(0x8000_0000, 0x8000_0000, 0, Z),
        Row(0xFFFF_FFFF, 0x7FFF_FFFF, 0x8000_0000, N),
        Row(0, 0x8000_0000, 0x8000_0000, X | N | V | C),
        Row(0x7FFF_FFFF, 0x8000_0000, 0xFFFF_FFFF, X | N | V | C),
    ];
    table("sub.l %d1,%d0", 0, &rows);
    table("sub.l %d1,%d0", X | N | Z | V | C, &rows);
    table(
        "subi.l #1,%d0",
        0,
        &[
            Row(0, 0, 0xFFFF_FFFF, X | N | C),
            Row(1, 0, 0, Z),
            Row(0x8000_0000, 0, 0x7FFF_FFFF, V),
            Row(0x7FFF_FFFF, 0, 0x7FFF_FFFE, 0),
        ],
    );
}

#[test]
fn addq_subq_flags() {
    table(
        "addq.l #1,%d0",
        0,
        &[
            Row(0x7FFF_FFFF, 0, 0x8000_0000, N | V),
            Row(0xFFFF_FFFF, 0, 0, X | Z | C),
            Row(0, 0, 1, 0),
        ],
    );
    // A quick count of 8 is encoded as 0.
    table(
        "addq.l #8,%d0",
        0,
        &[
            Row(0xFFFF_FFF9, 0, 1, X | C),
            Row(0xFFFF_FFF8, 0, 0, X | Z | C),
        ],
    );
    table(
        "subq.l #1,%d0",
        X,
        &[
            Row(0, 0, 0xFFFF_FFFF, X | N | C),
            Row(0x8000_0000, 0, 0x7FFF_FFFF, V),
            Row(1, 0, 0, Z),
        ],
    );
    table(
        "subq.l #8,%d0",
        0,
        &[Row(8, 0, 0, Z), Row(7, 0, 0xFFFF_FFFF, X | N | C)],
    );
    // An destination: the whole register, condition codes untouched.
    let m = exec("addq.l #1,%a0", 0, 0, 0xFFFF_FFFF, N | V | X);
    assert_eq!(m.cpu.a[0], 0);
    assert_eq!(m.cpu.ccr(), X | N | V);
    let m = exec("subq.l #3,%a0", 0, 0, 1, Z | C);
    assert_eq!(m.cpu.a[0], 0xFFFF_FFFE);
    assert_eq!(m.cpu.ccr(), Z | C);
}

#[test]
fn adda_suba_leave_ccr_alone() {
    let m = exec("adda.l %d1,%a0", 0, 1, 0x7FFF_FFFF, X | Z | C);
    assert_eq!(m.cpu.a[0], 0x8000_0000);
    assert_eq!(m.cpu.ccr(), X | Z | C);
    let m = exec("suba.l %d1,%a0", 0, 1, 0, 0);
    assert_eq!(m.cpu.a[0], 0xFFFF_FFFF);
    assert_eq!(m.cpu.ccr(), 0);
    let m = exec("adda.l %a0,%a0", 0, 0, 0x8000_0000, N);
    assert_eq!(m.cpu.a[0], 0);
    assert_eq!(m.cpu.ccr(), N);
}

#[test]
fn addx_keeps_z_when_result_zero() {
    // (D0, D1, X and Z going in) -> D0 and CCR. Z is cleared by a non-zero result and otherwise unchanged.
    let cases: [(u32, u32, u8, u32, u8); 10] = [
        (0, 0, Z, 0, Z),
        (0, 0, 0, 0, 0),
        (0, 0, X | Z, 1, 0),
        (0xFFFF_FFFF, 0, X | Z, 0, X | Z | C),
        (0xFFFF_FFFF, 0, X, 0, X | C),
        (1, 1, X | Z, 3, 0),
        (0x7FFF_FFFF, 0, X, 0x8000_0000, N | V),
        (0xFFFF_FFFF, 0xFFFF_FFFF, X, 0xFFFF_FFFF, X | N | C),
        (0x8000_0000, 0x7FFF_FFFF, X, 0, X | C),
        (0x8000_0000, 0x8000_0000, 0, 0, X | V | C),
    ];
    for (d0, d1, ccr, want, want_ccr) in cases {
        let m = exec("addx.l %d1,%d0", d0, d1, 0, ccr);
        assert_eq!(m.cpu.d[0], want, "addx {d0:#x}+{d1:#x} ccr={ccr:#x}");
        assert_eq!(m.cpu.ccr(), want_ccr, "addx {d0:#x}+{d1:#x} ccr={ccr:#x}");
    }
}

#[test]
fn subx_flags() {
    let cases: [(u32, u32, u8, u32, u8); 8] = [
        (0, 0, X, 0xFFFF_FFFF, X | N | C),
        (0, 0, Z, 0, Z),
        (0, 0, 0, 0, 0),
        (0x8000_0000, 0, X, 0x7FFF_FFFF, V),
        (0, 0xFFFF_FFFF, X | Z, 0, X | Z | C),
        (0xFFFF_FFFF, 0xFFFF_FFFF, X, 0xFFFF_FFFF, X | N | C),
        (5, 3, X | Z, 1, 0),
        (5, 5, Z, 0, Z),
    ];
    for (d0, d1, ccr, want, want_ccr) in cases {
        let m = exec("subx.l %d1,%d0", d0, d1, 0, ccr);
        assert_eq!(m.cpu.d[0], want, "subx {d0:#x}-{d1:#x} ccr={ccr:#x}");
        assert_eq!(m.cpu.ccr(), want_ccr, "subx {d0:#x}-{d1:#x} ccr={ccr:#x}");
    }
}

#[test]
fn subx_chain_64bit() {
    // D0:D1 - D2:D3 with sub.l on the low longwords and subx.l on the high ones.
    let src = "sub.l %d3,%d1\nsubx.l %d2,%d0";
    // (minuend, subtrahend, difference), each written out in hex.
    let cases: [(u64, u64, u64); 6] = [
        (0x0000_0001_0000_0000, 1, 0x0000_0000_FFFF_FFFF),
        (0x8000_0000_0000_0000, 1, 0x7FFF_FFFF_FFFF_FFFF),
        (0, 0xFFFF_FFFF_FFFF_FFFF, 1),
        (
            0x0000_0001_FFFF_FFFF,
            0x0000_0000_FFFF_FFFF,
            0x0000_0001_0000_0000,
        ),
        (
            0x1234_5678_9ABC_DEF0,
            0x0FED_CBA9_8765_4321,
            0x0246_8ACF_1357_9BCF,
        ),
        (5, 5, 0),
    ];
    for (a, b, want) in cases {
        let code = common::assemble(&format!("{src}\n.balign 2\nend:\n.long end\n"), BASE);
        let (code, tail) = code.split_at(code.len() - 4);
        let end = u32::from_be_bytes(tail.try_into().unwrap());
        let mut m = Machine::new();
        m.mem.map_ram(0x4010_0000, 0x0100_0000);
        m.mem.load(BASE, code).unwrap();
        m.cpu.pc = BASE;
        m.cpu.a[7] = STACK;
        (m.cpu.d[0], m.cpu.d[1]) = ((a >> 32) as u32, a as u32);
        (m.cpu.d[2], m.cpu.d[3]) = ((b >> 32) as u32, b as u32);
        let r = m.run(Limit {
            until_pc: Some(end),
            ..Limit::default()
        });
        assert_eq!(r.kind, Stop::UntilPc);
        let got = ((m.cpu.d[0] as u64) << 32) | m.cpu.d[1] as u64;
        assert_eq!(got, want, "{a:#x} - {b:#x}");
        // Z of the chain is set only when the whole 64-bit result is zero.
        assert_eq!(m.cpu.ccr() & Z != 0, want == 0, "{a:#x} - {b:#x}");
    }
}

#[test]
fn cmp_b_w_isa_b() {
    // D0 holds garbage above the compared part: only the byte or word takes part, and D0 is not written.
    // CMP computes D0 - D1; X is left alone, so every row runs with X clear and with X set.
    let rows: [(&str, u32, u32, u8); 14] = [
        ("cmp.b %d1,%d0", 0x1234_5680, 0xAB01, V),
        ("cmp.b %d1,%d0", 0x1234_5600, 0xFF01, N | C),
        ("cmp.b %d1,%d0", 0x1234_5601, 0xFF01, Z),
        ("cmp.b %d1,%d0", 0x1234_567F, 0xFF80, N | V | C),
        ("cmp.b %d1,%d0", 0x1234_56FF, 0x7F, N),
        ("cmp.w %d1,%d0", 0x1234_8000, 0xFFFF_0001, V),
        ("cmp.w %d1,%d0", 0x1234_0000, 0xFFFF_8000, N | V | C),
        ("cmp.w %d1,%d0", 0x1234_FFFF, 0x0000_FFFF, Z),
        ("cmp.w %d1,%d0", 0x1234_0000, 0x0000_0001, N | C),
        ("cmpi.b #1,%d0", 0x1234_5600, 0, N | C),
        ("cmpi.b #0x80,%d0", 0x1234_567F, 0, N | V | C),
        ("cmpi.w #0x8000,%d0", 0x1234_7FFF, 0, N | V | C),
        ("cmpi.w #0x7FFF,%d0", 0x1234_7FFF, 0, Z),
        ("cmpi.w #1,%d0", 0x1234_8000, 0, V),
    ];
    for (insn, d0, d1, want) in rows {
        for x in [0, X] {
            let m = exec(insn, d0, d1, 0, x);
            assert_eq!(m.cpu.d[0], d0, "{insn}: D0 must not change");
            assert_eq!(
                m.cpu.ccr(),
                want | x,
                "{insn} d0={d0:#x} d1={d1:#x} x={x:#x}"
            );
        }
    }
}

/// `cmpa.w %d1,%a0`. guess: the installed assembler rejects `cmpa.w` for the 54418 although CFPRM Table 3-16
/// lists it from ISA_B, so the opword (1011 000 011 000 001) is written out.
const CMPA_W_D1_A0: &str = ".word 0xB0C1";

#[test]
fn cmp_l_cmpa_cmpi_flags() {
    let rows = [
        Row(0, 0, 0, Z),
        Row(0, 1, 0, N | C),
        Row(0x8000_0000, 1, 0x8000_0000, V),
        Row(0x7FFF_FFFF, 0xFFFF_FFFF, 0x7FFF_FFFF, N | V | C),
        Row(0xFFFF_FFFF, 0xFFFF_FFFF, 0xFFFF_FFFF, Z),
        Row(1, 0xFFFF_FFFF, 1, C),
    ];
    for x in [0, X] {
        for &Row(d0, d1, _, ccr) in &rows {
            let m = exec("cmp.l %d1,%d0", d0, d1, 0, x);
            assert_eq!(m.cpu.d[0], d0);
            assert_eq!(m.cpu.ccr(), ccr | x, "cmp.l {d0:#x},{d1:#x}");
            let m = exec("cmpa.l %d1,%a0", 0, d1, d0, x);
            assert_eq!(m.cpu.a[0], d0);
            assert_eq!(m.cpu.ccr(), ccr | x, "cmpa.l {d0:#x},{d1:#x}");
        }
    }
    // CMPI.L #imm,Dx = Dx - imm.
    let m = exec("cmpi.l #0x80000000,%d0", 0x7FFF_FFFF, 0, 0, 0);
    assert_eq!(m.cpu.ccr(), N | V | C);
    let m = exec("cmpi.l #0,%d0", 0, 0, 0, X);
    assert_eq!(m.cpu.ccr(), X | Z);
    // CMPA.W sign-extends its source to a longword: A0 - 0xFFFFFFFF.
    let m = exec(CMPA_W_D1_A0, 0, 0xFFFF, 0, 0);
    assert_eq!(m.cpu.ccr(), C, "0 - (-1) = 1 with a borrow");
    let m = exec(CMPA_W_D1_A0, 0, 0x8000, 0xFFFF_8000, 0);
    assert_eq!(m.cpu.ccr(), Z);
    let m = exec(CMPA_W_D1_A0, 0, 0x7FFF, 0xFFFF_8000, X);
    assert_eq!(
        m.cpu.ccr(),
        X | N,
        "0xFFFF8000 - 0x7FFF = 0xFFFF0001: negative, no borrow, no overflow"
    );
}

#[test]
fn neg_negx() {
    table(
        "neg.l %d0",
        X | N | Z | V | C,
        &[
            Row(0, 0, 0, Z),
            Row(1, 0, 0xFFFF_FFFF, X | N | C),
            Row(0x7FFF_FFFF, 0, 0x8000_0001, X | N | C),
            Row(0x8000_0000, 0, 0x8000_0000, X | N | V | C),
            Row(0xFFFF_FFFF, 0, 1, X | C),
        ],
    );
    // NEGX: 0 - D0 - X, Z only cleared. Rows: (D0, unused, result, ccr) for each CCR going in.
    table(
        "negx.l %d0",
        Z,
        &[
            Row(0, 0, 0, Z),
            Row(1, 0, 0xFFFF_FFFF, X | N | C),
            Row(0x7FFF_FFFF, 0, 0x8000_0001, X | N | C),
        ],
    );
    table("negx.l %d0", 0, &[Row(0, 0, 0, 0)]);
    table(
        "negx.l %d0",
        X | Z,
        &[
            Row(0, 0, 0xFFFF_FFFF, X | N | C),
            Row(1, 0, 0xFFFF_FFFE, X | N | C),
            Row(0x8000_0000, 0, 0x7FFF_FFFF, X | C),
            Row(0xFFFF_FFFF, 0, 0, X | Z | C),
        ],
    );
}
