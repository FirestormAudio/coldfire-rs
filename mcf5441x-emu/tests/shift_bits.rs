mod common;

use common::{C, MEM, N, V, X, Z, run_ok};
use mcf5441x_emu::Size;

/// Run `insn` with D0 = `d0`, D1 = `d1` and the CCR set to `ccr`.
fn shift(insn: &str, d0: u32, d1: u32, ccr: u8) -> (u32, u8) {
    let m = run_ok(insn, |m| {
        m.cpu.d[0] = d0;
        m.cpu.d[1] = d1;
        m.cpu.set_sr(0x2700 | ccr as u16);
    });
    (m.cpu.d[0], m.cpu.ccr())
}

/// (D0, count in D1, result, CCR). With a nonzero count every flag is rewritten, so the CCR going in must not
/// matter: X is checked against both 0 and 1.
type Row = (u32, u32, u32, u8);

fn check_reg(insn: &str, rows: &[Row]) {
    for &(d0, n, want, ccr) in rows {
        for ccr_in in [0, X, V | C, X | N | Z | V | C] {
            assert_eq!(
                shift(insn, d0, n, ccr_in),
                (want, ccr),
                "{insn} d0={d0:#x} count={n} in={ccr_in:#x}"
            );
        }
    }
}

#[test]
fn shift_counts_mod_64_and_flags() {
    // Last bit out goes to C and X; V is always clear (CFPRM ch. 4, "unlike the 68K family").
    let lsl: [Row; 8] = [
        (0x8000_0001, 1, 0x0000_0002, X | C),
        (0x1234_5678, 4, 0x2345_6780, X | C),
        (0x0000_0001, 32, 0, X | Z | C),
        (0x0000_0002, 32, 0, Z),
        (0xFFFF_FFFF, 33, 0, Z),
        (0x4000_0000, 65, 0x8000_0000, N),
        (0xFFFF_FFFF, 63, 0, Z),
        (0x0000_0001, 31, 0x8000_0000, N),
    ];
    check_reg("lsl.l %d1,%d0", &lsl);
    let lsr: [Row; 6] = [
        (0x0000_0003, 1, 0x0000_0001, X | C),
        (0x8000_0000, 31, 0x0000_0001, 0),
        (0x8000_0000, 32, 0, X | Z | C),
        (0x8000_0000, 33, 0, Z),
        (0xFFFF_FFFF, 63, 0, Z),
        (0xFFFF_FFFF, 0x41, 0x7FFF_FFFF, X | C),
    ];
    check_reg("lsr.l %d1,%d0", &lsr);
    let asr: [Row; 8] = [
        (0x8000_0000, 1, 0xC000_0000, N),
        (0x8000_0001, 1, 0xC000_0000, X | N | C),
        (0x8000_0000, 31, 0xFFFF_FFFF, N),
        (0x8000_0000, 32, 0xFFFF_FFFF, X | N | C),
        (0x8000_0000, 40, 0xFFFF_FFFF, X | N | C),
        (0x4000_0000, 32, 0, Z),
        (0x7FFF_FFFF, 63, 0, Z),
        (0x0000_0001, 1, 0, X | Z | C),
    ];
    check_reg("asr.l %d1,%d0", &asr);
    let asl: [Row; 7] = [
        (0x4000_0000, 1, 0x8000_0000, N),
        (0x8000_0000, 1, 0, X | Z | C),
        (0xC000_0000, 2, 0, X | Z | C),
        (0x0000_0001, 31, 0x8000_0000, N),
        (0x0000_0001, 32, 0, X | Z | C),
        (0x0000_0001, 33, 0, Z),
        (0x0000_0003, 0x1000_0001, 0x0000_0006, 0),
    ];
    check_reg("asl.l %d1,%d0", &asl);
}

#[test]
fn immediate_counts_are_1_to_8() {
    // The count field holds 0 for a count of 8.
    let rows: [(&str, u32, u32, u8); 8] = [
        ("lsl.l #8,%d0", 0x00FF_0001, 0xFF00_0100, N),
        ("lsl.l #8,%d0", 0x0100_0000, 0, X | Z | C),
        ("lsl.l #1,%d0", 0x4000_0000, 0x8000_0000, N),
        ("lsr.l #3,%d0", 0x0000_000C, 1, X | C),
        ("lsr.l #8,%d0", 0x0000_0080, 0, X | Z | C),
        ("asr.l #1,%d0", 0xFFFF_FFFF, 0xFFFF_FFFF, X | N | C),
        ("asr.l #8,%d0", 0x8000_0000, 0xFF80_0000, N),
        ("asl.l #8,%d0", 0x0100_0000, 0, X | Z | C),
    ];
    for (insn, d0, want, ccr) in rows {
        for ccr_in in [0, X | N | Z | V | C] {
            assert_eq!(shift(insn, d0, 0, ccr_in), (want, ccr), "{insn} d0={d0:#x}");
        }
    }
}

#[test]
fn shift_by_zero_clears_c_keeps_x() {
    // A zero count (or one that is a multiple of 64) leaves the value and X; C and V are cleared; N and Z follow
    // the unchanged value.
    for insn in [
        "lsl.l %d1,%d0",
        "lsr.l %d1,%d0",
        "asl.l %d1,%d0",
        "asr.l %d1,%d0",
    ] {
        for count in [0, 64, 128, 0xFFFF_FFC0] {
            for (d0, nz) in [(0u32, Z), (0x8000_0000, N), (1, 0)] {
                for ccr_in in [0, X, V | C, X | N | Z | V | C] {
                    assert_eq!(
                        shift(insn, d0, count, ccr_in),
                        (d0, nz | (ccr_in & X)),
                        "{insn} d0={d0:#x} count={count:#x} in={ccr_in:#x}"
                    );
                }
            }
        }
    }
}

/// Run `insn` with D0, D1, the byte at `MEM` and A1 = `MEM`; returns the machine.
fn bit(insn: &str, d0: u32, d1: u32, byte: u8, ccr: u8) -> mcf5441x_emu::Machine {
    run_ok(insn, |m| {
        m.cpu.d[0] = d0;
        m.cpu.d[1] = d1;
        m.mem.write(MEM, Size::Byte, byte as u32).unwrap();
        m.mem.write(MEM + 1, Size::Byte, 0xA5).unwrap();
        m.cpu.set_sr(0x2700 | ccr as u16);
    })
}

#[test]
fn btst_register_mod_32_memory_mod_8() {
    // Z is set when the tested bit is zero; the other flags and the operand are not touched.
    for ccr_in in [0, X | N | V | C, Z, X | N | Z | V | C] {
        let keep = ccr_in & !Z;
        // Data register: the bit number is taken modulo 32.
        for (d1, set) in [
            (8u32, true),
            (40, true),
            (0xFFFF_FFE8, true),
            (9, false),
            (7, false),
            (0, false),
            (32, false),
        ] {
            let m = bit("btst %d1,%d0", 0x0000_0100, d1, 0, ccr_in);
            assert_eq!(m.cpu.d[0], 0x0000_0100);
            assert_eq!(m.cpu.ccr(), keep | if set { 0 } else { Z }, "d1={d1:#x}");
        }
        // Memory: a byte, the bit number taken modulo 8.
        for (d1, set) in [(2u32, true), (10, true), (34, true), (3, false), (0, false)] {
            let m = bit("btst %d1,(%a1)", 0, d1, 0x04, ccr_in);
            assert_eq!(m.cpu.ccr(), keep | if set { 0 } else { Z }, "mem d1={d1}");
        }
    }
    // Immediate bit numbers.
    let m = bit("btst #33,%d0", 0x0000_0002, 0, 0, 0);
    assert_eq!(m.cpu.ccr(), 0, "#33 is bit 1 of a register");
    let m = bit("btst #5,(%a1)", 0, 0, 0x04, 0);
    assert_eq!(m.cpu.ccr(), Z);
    // The byte operand moves (An)+ by one.
    let m = bit("btst %d1,(%a1)+", 0, 2, 0x04, 0);
    assert_eq!(m.cpu.a[1], MEM + 1);
    assert_eq!(m.cpu.ccr(), 0);
}

#[test]
fn bset_bclr_bchg_write_back() {
    // (insn, D0, bit number in D1, new D0, Z)
    let regs: [(&str, u32, u32, u32, bool); 8] = [
        ("bset %d1,%d0", 0, 33, 0x0000_0002, true),
        ("bset %d1,%d0", 0x0000_0002, 1, 0x0000_0002, false),
        ("bclr %d1,%d0", 0xFFFF_FFFF, 31, 0x7FFF_FFFF, false),
        ("bclr %d1,%d0", 0, 31, 0, true),
        ("bchg %d1,%d0", 0x8000_0000, 63, 0, false),
        ("bchg %d1,%d0", 0, 0, 1, true),
        ("bset #31,%d0", 0, 0, 0x8000_0000, true),
        ("bclr #0,%d0", 0x0000_0101, 0, 0x0000_0100, false),
    ];
    for (insn, d0, d1, want, z) in regs {
        for ccr_in in [0, X | N | V | C, Z] {
            let m = bit(insn, d0, d1, 0, ccr_in);
            assert_eq!(m.cpu.d[0], want, "{insn} d0={d0:#x} d1={d1}");
            assert_eq!(m.cpu.ccr(), (ccr_in & !Z) | if z { Z } else { 0 }, "{insn}");
        }
    }
    // Memory: one byte changes, its neighbour does not.
    let mem: [(&str, u32, u8, bool); 6] = [
        ("bset %d1,(%a1)", 9, 0x06, true),
        ("bset %d1,(%a1)", 3, 0x0C, true),
        ("bclr %d1,(%a1)", 2, 0x00, false),
        ("bclr %d1,(%a1)", 0, 0x04, true),
        ("bchg %d1,(%a1)", 10, 0x00, false),
        ("bchg %d1,(%a1)", 0, 0x05, true),
    ];
    for (insn, d1, want, z) in mem {
        let mut m = bit(insn, 0, d1, 0x04, X);
        assert_eq!(
            m.mem.read(MEM, Size::Byte).unwrap(),
            want as u32,
            "{insn} d1={d1}"
        );
        assert_eq!(m.mem.read(MEM + 1, Size::Byte).unwrap(), 0xA5);
        assert_eq!(m.cpu.ccr(), X | if z { Z } else { 0 }, "{insn}");
    }
    let mut m = bit("bset #7,(%a1)+", 0, 0, 0x01, 0);
    assert_eq!(m.mem.read(MEM, Size::Byte).unwrap(), 0x81);
    assert_eq!(m.cpu.a[1], MEM + 1);
}

#[test]
fn scc_all_conditions() {
    // CCR states: A none, B N V, C Z C, D N C. Rows are written out from the tests in CFPRM ch. 4 Scc:
    // each is the outcome in states A B C D.
    let states = [0, N | V, Z | C, N | C];
    let rows: [(&str, [bool; 4]); 16] = [
        ("st", [true, true, true, true]),
        ("sf", [false, false, false, false]),
        ("shi", [true, true, false, false]),
        ("sls", [false, false, true, true]),
        ("scc", [true, true, false, false]),
        ("scs", [false, false, true, true]),
        ("sne", [true, true, false, true]),
        ("seq", [false, false, true, false]),
        ("svc", [true, false, true, true]),
        ("svs", [false, true, false, false]),
        ("spl", [true, false, true, false]),
        ("smi", [false, true, false, true]),
        ("sge", [true, true, true, false]),
        ("slt", [false, false, false, true]),
        ("sgt", [true, true, false, false]),
        ("sle", [false, false, true, true]),
    ];
    for (insn, want) in rows {
        for (i, &ccr_in) in states.iter().enumerate() {
            for x in [0, X] {
                let m = run_ok(&format!("{insn} %d0"), |m| {
                    m.cpu.d[0] = 0xA5A5_A5A5;
                    m.cpu.set_sr(0x2700 | (ccr_in | x) as u16);
                });
                let low = if want[i] { 0xFF } else { 0x00 };
                assert_eq!(m.cpu.d[0], 0xA5A5_A500 | low, "{insn} state {i}");
                assert_eq!(m.cpu.ccr(), ccr_in | x, "{insn} leaves the CCR");
            }
        }
    }
}
