mod common;

use mcf5441x_emu::{Limit, Machine, Size, Stop};

const BASE: u32 = 0x4010_0000;
const STACK: u32 = 0x4100_0000;
const MEM: u32 = 0x4020_0000;

const X: u8 = 0x10;
const N: u8 = 0x08;
const Z: u8 = 0x04;
const V: u8 = 0x02;
const C: u8 = 0x01;

/// Run `src` with D0, D1, A1 = `MEM` and the CCR set; `MEM` holds `mem` as a longword.
fn exec(src: &str, d0: u32, d1: u32, mem: u32, ccr: u8) -> Machine {
    let code = common::assemble(&format!("{src}\n.balign 2\nend:\n.long end\n"), BASE);
    let (code, tail) = code.split_at(code.len() - 4);
    let end = u32::from_be_bytes(tail.try_into().unwrap());
    let mut m = Machine::new();
    m.mem.map_ram(0x4010_0000, 0x0100_0000);
    m.mem.load(BASE, code).unwrap();
    m.mem.write(MEM, Size::Long, mem).unwrap();
    m.cpu.pc = BASE;
    m.cpu.a[7] = STACK;
    m.cpu.a[1] = MEM;
    m.cpu.d[0] = d0;
    m.cpu.d[1] = d1;
    m.cpu.set_sr(0x2700 | ccr as u16);
    let r = m.run(Limit {
        until_pc: Some(end),
        ..Limit::default()
    });
    assert_eq!(r.kind, Stop::UntilPc, "{src}: {r:?}");
    m
}

/// (D0, D1, result, N and Z of the result).
type Row = (u32, u32, u32, u8);

/// Every row with each CCR going in: V and C always come out clear and X comes out as it went in.
fn check(insn: &str, rows: &[Row], result: impl Fn(&Machine) -> u32) {
    for &(d0, d1, want, nz) in rows {
        for ccr in [0, X, V | C, X | N | Z | V | C] {
            let m = exec(insn, d0, d1, 0, ccr);
            assert_eq!(result(&m), want, "{insn} d0={d0:#x} d1={d1:#x}");
            assert_eq!(
                m.cpu.ccr(),
                nz | (ccr & X),
                "{insn} d0={d0:#x} d1={d1:#x} in={ccr:#x}"
            );
        }
    }
}

fn d0(m: &Machine) -> u32 {
    m.cpu.d[0]
}

#[test]
fn logic_ops_clear_v_and_c_keep_x() {
    let and: [Row; 5] = [
        (0xFFFF_FFFF, 0x8000_0000, 0x8000_0000, N),
        (0x7FFF_FFFF, 0x8000_0000, 0, Z),
        (1, 1, 1, 0),
        (0, 0xFFFF_FFFF, 0, Z),
        (0xFFFF_FFFF, 0xFFFF_FFFF, 0xFFFF_FFFF, N),
    ];
    check("and.l %d1,%d0", &and, d0);
    let or: [Row; 5] = [
        (0, 0, 0, Z),
        (0x8000_0000, 1, 0x8000_0001, N),
        (0x7FFF_FFFF, 0x8000_0000, 0xFFFF_FFFF, N),
        (1, 0, 1, 0),
        (0x0000_FFFF, 0x7FFF_0000, 0x7FFF_FFFF, 0),
    ];
    check("or.l %d1,%d0", &or, d0);
    let eor: [Row; 5] = [
        (0xFFFF_FFFF, 0xFFFF_FFFF, 0, Z),
        (0x7FFF_FFFF, 0x8000_0000, 0xFFFF_FFFF, N),
        (1, 0, 1, 0),
        (0, 0, 0, Z),
        (0x8000_0000, 0xFFFF_FFFF, 0x7FFF_FFFF, 0),
    ];
    // EOR Dy,<ea>: D1 into D0 here.
    check("eor.l %d1,%d0", &eor, d0);
    // The immediate forms use D1's value as the immediate in the source text.
    for &(a, b, want, nz) in &and {
        check(&format!("andi.l #{b:#x},%d0"), &[(a, 0, want, nz)], d0);
    }
    for &(a, b, want, nz) in &or {
        check(&format!("ori.l #{b:#x},%d0"), &[(a, 0, want, nz)], d0);
    }
    for &(a, b, want, nz) in &eor {
        check(&format!("eori.l #{b:#x},%d0"), &[(a, 0, want, nz)], d0);
    }
    // Memory forms.
    for &(a, b, want, nz) in &and {
        let mut m = exec("and.l %d1,(%a1)", 0, b, a, X | V | C);
        assert_eq!(m.mem.read(MEM, Size::Long).unwrap(), want);
        assert_eq!(m.cpu.ccr(), nz | X);
        let m = exec("and.l (%a1),%d0", b, 0, a, V | C);
        assert_eq!(m.cpu.d[0], want);
        assert_eq!(m.cpu.ccr(), nz);
    }
    for &(a, b, want, nz) in &or {
        let mut m = exec("or.l %d1,(%a1)", 0, b, a, X | V | C);
        assert_eq!(m.mem.read(MEM, Size::Long).unwrap(), want);
        assert_eq!(m.cpu.ccr(), nz | X);
    }
    for &(a, b, want, nz) in &eor {
        let mut m = exec("eor.l %d1,(%a1)", 0, b, a, V | C);
        assert_eq!(m.mem.read(MEM, Size::Long).unwrap(), want);
        assert_eq!(m.cpu.ccr(), nz);
    }
    // NOT.
    let not: [Row; 5] = [
        (0, 0, 0xFFFF_FFFF, N),
        (0xFFFF_FFFF, 0, 0, Z),
        (0x7FFF_FFFF, 0, 0x8000_0000, N),
        (1, 0, 0xFFFF_FFFE, N),
        (0x8000_0000, 0, 0x7FFF_FFFF, 0),
    ];
    check("not.l %d0", &not, d0);
}

#[test]
fn ext_extb_swap() {
    // EXT.W: byte to word, the upper word untouched. guess: N and Z are those of the 16-bit result.
    check(
        "ext.w %d0",
        &[
            (0x1234_5680, 0, 0x1234_FF80, N),
            (0x1234_567F, 0, 0x1234_007F, 0),
            (0x1234_5600, 0, 0x1234_0000, Z),
            (0xABCD_00FF, 0, 0xABCD_FFFF, N),
        ],
        d0,
    );
    // EXT.L: word to longword.
    check(
        "ext.l %d0",
        &[
            (0x1234_8000, 0, 0xFFFF_8000, N),
            (0x1234_7FFF, 0, 0x0000_7FFF, 0),
            (0xFFFF_0000, 0, 0, Z),
            (0x0001_FFFF, 0, 0xFFFF_FFFF, N),
        ],
        d0,
    );
    // EXTB.L: byte to longword.
    check(
        "extb.l %d0",
        &[
            (0x1234_5680, 0, 0xFFFF_FF80, N),
            (0x1234_567F, 0, 0x0000_007F, 0),
            (0xFFFF_FF00, 0, 0, Z),
            (0x0000_00FF, 0, 0xFFFF_FFFF, N),
        ],
        d0,
    );
    // SWAP: exchange the halves; N and Z from the 32-bit result.
    check(
        "swap %d0",
        &[
            (0x1234_5678, 0, 0x5678_1234, 0),
            (0x0000_FFFF, 0, 0xFFFF_0000, N),
            (0, 0, 0, Z),
            (0xFFFF_0000, 0, 0x0000_FFFF, 0),
            (0x8000_0000, 0, 0x0000_8000, 0),
        ],
        d0,
    );
}

#[test]
fn tst_clr() {
    // TST: N and Z of the tested size, V and C cleared, X kept, operand unchanged.
    let rows: [(&str, u32, u8); 10] = [
        ("tst.b %d0", 0x1234_5680, N),
        ("tst.b %d0", 0x1234_5600, Z),
        ("tst.b %d0", 0x1234_567F, 0),
        ("tst.w %d0", 0x1234_8000, N),
        ("tst.w %d0", 0xFFFF_0000, Z),
        ("tst.w %d0", 0x0000_7FFF, 0),
        ("tst.l %d0", 0x8000_0000, N),
        ("tst.l %d0", 0, Z),
        ("tst.l %d0", 1, 0),
        ("tst.l %a0", 0, Z),
    ];
    for (insn, v, want) in rows {
        for ccr in [0, X | V | C] {
            let m = exec(insn, v, 0, 0, ccr);
            assert_eq!(m.cpu.d[0], v);
            assert_eq!(m.cpu.ccr(), want | (ccr & X), "{insn} {v:#x}");
        }
    }
    let m = exec("tst.b (%a1)", 0, 0, 0x8000_0000, V);
    assert_eq!(m.cpu.ccr(), N);
    let m = exec("tst.w (%a1)", 0, 0, 0x0000_FFFF, V);
    assert_eq!(m.cpu.ccr(), Z);
    let m = exec("tst.l (%a1)", 0, 0, 0x0000_FFFF, 0);
    assert_eq!(m.cpu.ccr(), 0);

    // CLR: the sized part becomes zero, N=0 Z=1 V=0 C=0, X kept.
    for (insn, want) in [
        ("clr.b %d0", 0x1234_5600),
        ("clr.w %d0", 0x1234_0000),
        ("clr.l %d0", 0),
    ] {
        for ccr in [N | V | C, X | N | V | C] {
            let m = exec(insn, 0x1234_5678, 0, 0, ccr);
            assert_eq!(m.cpu.d[0], want, "{insn}");
            assert_eq!(m.cpu.ccr(), Z | (ccr & X), "{insn}");
        }
    }
    let mut m = exec("clr.l (%a1)", 0, 0, 0xDEAD_BEEF, N);
    assert_eq!(m.mem.read(MEM, Size::Long).unwrap(), 0);
    assert_eq!(m.cpu.ccr(), Z);
    let mut m = exec("clr.w (%a1)+", 0, 0, 0xDEAD_BEEF, 0);
    assert_eq!(m.mem.read(MEM, Size::Long).unwrap(), 0x0000_BEEF);
    assert_eq!(m.cpu.a[1], MEM + 2);
    let mut m = exec("clr.b (1,%a1)", 0, 0, 0xDEAD_BEEF, 0);
    assert_eq!(m.mem.read(MEM, Size::Long).unwrap(), 0xDE00_BEEF);
}
