mod common;

use common::{C, MEM, N, V, X, Z, run, run_ok};
use mcf5441x_emu::{Size, Stop};

/// Run `insn` with D0, D1, D2 and the longword at `MEM`; returns (D0, D1, D2, CCR).
fn go(insn: &str, d0: u32, d1: u32, d2: u32, ccr: u8) -> [u32; 4] {
    let m = run_ok(insn, |m| {
        m.cpu.d = [d0, d1, d2, 0, 0, 0, 0, 0];
        m.cpu.set_sr(0x2700 | ccr as u16);
    });
    [m.cpu.d[0], m.cpu.d[1], m.cpu.d[2], m.cpu.ccr() as u32]
}

/// (D0, D1, result in D0, N and Z): V and C come out clear, X as it went in.
type Row = (u32, u32, u32, u8);

fn check(insn: &str, rows: &[Row]) {
    for &(d0, d1, want, nz) in rows {
        for ccr_in in [0, X, V | C, X | N | Z | V | C] {
            let [r0, r1, _, ccr] = go(insn, d0, d1, 0, ccr_in);
            assert_eq!(r0, want, "{insn} d0={d0:#x} d1={d1:#x}");
            assert_eq!(r1, d1);
            assert_eq!(
                ccr,
                (nz | (ccr_in & X)) as u32,
                "{insn} d0={d0:#x} d1={d1:#x} in={ccr_in:#x}"
            );
        }
    }
}

#[test]
fn muls_mulu_word_long() {
    // Word: the low words multiply, the upper word of Dx is ignored, all 32 bits of the product are kept.
    let muls_w: [Row; 6] = [
        (0xFFFF_0003, 0x0000_FFFE, 0xFFFF_FFFA, N),
        (0x0000_7FFF, 0xABCD_7FFF, 0x3FFF_0001, 0),
        (0x0000_8000, 0x0000_8000, 0x4000_0000, 0),
        (0x1234_0000, 0x0000_0005, 0, Z),
        (0x0000_0007, 0x0000_FFFF, 0xFFFF_FFF9, N),
        (0x0000_8000, 0x0000_0001, 0xFFFF_8000, N),
    ];
    check("muls.w %d1,%d0", &muls_w);
    let mulu_w: [Row; 4] = [
        (0xFFFF_0003, 0x0000_FFFE, 0x0002_FFFA, 0),
        (0x0000_FFFF, 0x0000_FFFF, 0xFFFE_0001, N),
        (0x0000_0000, 0x0000_FFFF, 0, Z),
        (0x0001_0002, 0xFFFF_0003, 0x0000_0006, 0),
    ];
    check("mulu.w %d1,%d0", &mulu_w);
    // Long: the low 32 bits; V stays clear however much is discarded.
    let muls_l: [Row; 5] = [
        (0x0001_0000, 0x0001_0000, 0, Z),
        (0xFFFF_FFFD, 5, 0xFFFF_FFF1, N),
        (0x8000_0000, 0xFFFF_FFFF, 0x8000_0000, N),
        (3, 4, 12, 0),
        (0x4000_0000, 2, 0x8000_0000, N),
    ];
    check("muls.l %d1,%d0", &muls_l);
    let mulu_l: [Row; 4] = [
        (0xFFFF_FFFF, 2, 0xFFFF_FFFE, N),
        (0x1234_5678, 0x10, 0x2345_6780, 0),
        (0x0001_0000, 0x0001_0000, 0, Z),
        (0xFFFF_FFFF, 0xFFFF_FFFF, 1, 0),
    ];
    check("mulu.l %d1,%d0", &mulu_l);
    // A memory source: the word at the lower address is the high half of the longword.
    let m = run_ok("muls.w (%a1),%d0", |m| {
        m.cpu.d[0] = 7;
        m.mem.write(MEM, Size::Long, 0xFFFE_0000).unwrap();
    });
    assert_eq!(m.cpu.d[0], 0xFFFF_FFF2);
    let m = run_ok("mulu.l (%a1),%d0", |m| {
        m.cpu.d[0] = 7;
        m.mem.write(MEM, Size::Long, 0xFFFE_0000).unwrap();
    });
    assert_eq!(m.cpu.d[0], 0xFFFE_0000u32.wrapping_mul(7));
}

#[test]
fn divs_divu_word_remainder() {
    // Word: 32-bit Dx / 16-bit source; remainder in the upper word, quotient in the lower. (D0, D1, D0, CCR)
    let rows: [(&str, u32, u32, u32, u8); 14] = [
        ("divu.w %d1,%d0", 100, 7, 0x0002_000E, 0),
        ("divu.w %d1,%d0", 5, 10, 0x0005_0000, Z),
        ("divu.w %d1,%d0", 0x0001_FFFF, 2, 0x0001_FFFF, N),
        ("divu.w %d1,%d0", 7, 0xABCD_0002, 0x0001_0003, 0),
        // Overflow: the destination is unchanged, V set, N and Z cleared.
        ("divu.w %d1,%d0", 0xFFFF_FFFF, 0x10, 0xFFFF_FFFF, V),
        ("divu.w %d1,%d0", 0x0001_0000, 1, 0x0001_0000, V),
        ("divs.w %d1,%d0", 0xFFFF_FFF9, 2, 0xFFFF_FFFD, N),
        ("divs.w %d1,%d0", 7, 0xFFFE, 0x0001_FFFD, N),
        ("divs.w %d1,%d0", 0xFFFF_8000, 1, 0x0000_8000, N),
        ("divs.w %d1,%d0", 0, 3, 0, Z),
        ("divs.w %d1,%d0", 0x0000_8000, 1, 0x0000_8000, V),
        ("divs.w %d1,%d0", 0x8000_0000, 0xFFFF, 0x8000_0000, V),
        ("divs.w %d1,%d0", 0xFFFF_8001, 1, 0x0000_8001, N),
        // -32769 does not fit in 16 bits.
        ("divs.w %d1,%d0", 0xFFFF_7FFF, 1, 0xFFFF_7FFF, V),
    ];
    for (insn, d0, d1, want, ccr) in rows {
        for ccr_in in [0, X, V | C, X | N | Z | V | C] {
            let [r0, r1, ..] = go(insn, d0, d1, 0, ccr_in);
            assert_eq!(r0, want, "{insn} d0={d0:#x} d1={d1:#x}");
            assert_eq!(r1, d1);
            let got = go(insn, d0, d1, 0, ccr_in)[3] as u8;
            assert_eq!(got, ccr | (ccr_in & X), "{insn} d0={d0:#x} d1={d1:#x}");
        }
    }
    // Long: the quotient goes to Dx (CFPRM ch. 4: "Dw = Dx" is DIVS/DIVU).
    let long: [(&str, u32, u32, u32, u8); 8] = [
        ("divu.l %d1,%d0", 100, 7, 14, 0),
        ("divu.l %d1,%d0", 0xFFFF_FFFF, 1, 0xFFFF_FFFF, N),
        ("divu.l %d1,%d0", 0xFFFF_FFFF, 0x1_0000, 0xFFFF, 0),
        ("divu.l %d1,%d0", 3, 5, 0, Z),
        ("divs.l %d1,%d0", 0xFFFF_FF9C, 7, 0xFFFF_FFF2, N),
        ("divs.l %d1,%d0", 0x8000_0000, 0xFFFF_FFFF, 0x8000_0000, V),
        ("divs.l %d1,%d0", 0x8000_0000, 2, 0xC000_0000, N),
        ("divs.l %d1,%d0", 100, 0xFFFF_FFF9, 0xFFFF_FFF2, N),
    ];
    for (insn, d0, d1, want, ccr) in long {
        let [r0, _, _, got] = go(insn, d0, d1, 0, X);
        assert_eq!(r0, want, "{insn} d0={d0:#x} d1={d1:#x}");
        assert_eq!(got as u8, ccr | X, "{insn} d0={d0:#x} d1={d1:#x}");
    }
    // REMU/REMS: the remainder goes to Dw and Dx keeps the dividend. N and Z follow the quotient, as the
    // manual's condition codes say. (insn, D0, D1, D2 out, CCR)
    let rem: [(&str, u32, u32, u32, u8); 7] = [
        ("remu.l %d1,%d2:%d0", 100, 7, 2, 0),
        ("remu.l %d1,%d2:%d0", 3, 5, 3, Z),
        ("remu.l %d1,%d2:%d0", 0xFFFF_FFFF, 0x10, 0xF, 0),
        ("rems.l %d1,%d2:%d0", 0xFFFF_FF9C, 7, 0xFFFF_FFFE, N),
        ("rems.l %d1,%d2:%d0", 100, 0xFFFF_FFF9, 2, N),
        ("rems.l %d1,%d2:%d0", 14, 7, 0, 0),
        // Overflow: neither register is written.
        (
            "rems.l %d1,%d2:%d0",
            0x8000_0000,
            0xFFFF_FFFF,
            0x5555_5555,
            V,
        ),
    ];
    for (insn, d0, d1, d2, ccr) in rem {
        let [r0, r1, r2, got] = go(insn, d0, d1, 0x5555_5555, C | X);
        assert_eq!(r0, d0, "{insn}: Dx keeps the dividend");
        assert_eq!(r1, d1);
        assert_eq!(r2, d2, "{insn} d0={d0:#x} d1={d1:#x}");
        assert_eq!(got as u8, ccr | X, "{insn} d0={d0:#x} d1={d1:#x}");
    }
}

#[test]
fn divide_by_zero_raises_vector_5() {
    // The exception stops at the divide instruction; no register or flag changes (CFPRM ch. 4 DIVS). The word
    // forms divide by the low word of D1 only, so a nonzero upper word does not save them.
    let rows: [(&str, u32); 8] = [
        ("divu.w %d1,%d0", 0),
        ("divu.w %d1,%d0", 0xFFFF_0000),
        ("divs.w %d1,%d0", 0xFFFF_0000),
        ("divs.w %d1,%d0", 0),
        ("divu.l %d1,%d0", 0),
        ("divs.l %d1,%d0", 0),
        ("remu.l %d1,%d2:%d0", 0),
        ("rems.l %d1,%d2:%d0", 0),
    ];
    for (insn, d1) in rows {
        let (m, r) = run(&format!("tpf\n{insn}\nmoveq #2,%d3"), |m| {
            m.cpu.d = [0x1234_5678, d1, 0x5555_5555, 0, 0, 0, 0, 0];
            m.cpu.set_sr(0x2700 | (X | N | V) as u16);
        });
        assert_eq!(r.kind, Stop::Exception(5), "{insn} d1={d1:#x}");
        assert_eq!(r.pc, common::BASE + 2, "{insn}: PC is the divide");
        assert_eq!(m.cpu.pc, common::BASE + 2);
        assert_eq!(m.cpu.d[0], 0x1234_5678, "{insn}");
        assert_eq!(m.cpu.d[1], d1, "{insn}");
        assert_eq!(m.cpu.d[2], 0x5555_5555, "{insn}");
        assert_eq!(m.cpu.d[3], 0, "{insn}: the next instruction did not run");
        assert_eq!(m.cpu.ccr(), X | N | V, "{insn}: the flags are as they were");
        assert_eq!(m.instructions(), 1, "{insn}");
    }
}
