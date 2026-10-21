mod common;

use common::{C, MEM, N, V, X, Z, run_ok};
use mcf5441x_emu::Size;

/// Each case is the instruction lines of one function that was run on an MCF54415 (without its closing HALT),
/// the register values it was given and the D0 it left. Every value here was measured on an MCF54415.
#[test]
fn measured_on_mcf54415() {
    // (instruction lines, D0, D1, D2 in; D0 out)
    let cases: [(&str, [u32; 3], u32); 8] = [
        // mvs.b of 0x80 -> 0xffffff80
        ("mvs.b\t%d0, %d0", [0x80, 0, 0], 0xFFFF_FF80),
        // mvz.w of 0xffff8001 -> 0x00008001
        ("mvz.w\t%d0, %d0", [0xFFFF_8001, 0, 0], 0x0000_8001),
        // remu.l: d0 % d1 (remainder into %d2, then moved to %d0), here 3 % 5
        ("remu.l\t%d1, %d2:%d0\n\tmove.l\t%d2, %d0", [3, 5, 0x77], 3),
        // ff1.l of 0x00010000 -> 15
        ("ff1.l\t%d0", [0x0001_0000, 0, 0], 15),
        // bitrev.l of 1 -> 0x80000000
        ("bitrev.l\t%d0", [1, 0, 0], 0x8000_0000),
        // byterev.l of 0x11223344 -> 0x44332211
        ("byterev.l\t%d0", [0x1122_3344, 0, 0], 0x4433_2211),
        // mov3q.l #-1 -> 0xffffffff
        ("mov3q.l\t#-1, %d0", [0, 0, 0], 0xFFFF_FFFF),
        // sats.l after an overflowing add: 0x7fffffff + 1 -> 0x7fffffff
        (
            "add.l\t%d1, %d0\n\tsats.l\t%d0",
            [0x7FFF_FFFF, 1, 0],
            0x7FFF_FFFF,
        ),
    ];
    for (src, [d0, d1, d2], want) in cases {
        let m = run_ok(src, |m| {
            m.cpu.d[0] = d0;
            m.cpu.d[1] = d1;
            m.cpu.d[2] = d2;
        });
        assert_eq!(m.cpu.d[0], want, "{src}");
    }
}

/// Run `insn` with D0 and the CCR; returns (D0, CCR).
fn one(insn: &str, d0: u32, ccr: u8) -> (u32, u8) {
    let m = run_ok(insn, |m| {
        m.cpu.d[0] = d0;
        m.cpu.set_sr(0x2700 | ccr as u16);
    });
    (m.cpu.d[0], m.cpu.ccr())
}

#[test]
fn sats_only_when_v_set() {
    // With V set the register saturates by its (wrong) sign: positive -> 0x80000000 is wrong-signed, so a
    // register that reads positive becomes the most negative value and a negative one the most positive.
    // guess: the manual page gives no condition-code box; "set according to the result" is read as N and Z from
    // the result, V and C cleared, X unchanged, as for MOVE.
    let rows: [(u32, u8, u32, u8); 8] = [
        (0x8000_0000, V, 0x7FFF_FFFF, 0),
        (0xFFFF_FFFF, V | C, 0x7FFF_FFFF, 0),
        (0x0000_0000, V, 0x8000_0000, N),
        (0x1234_5678, V | X, 0x8000_0000, N | X),
        // V clear: nothing changes, the flags are those of the value.
        (0x8000_0000, 0, 0x8000_0000, N),
        (0x7FFF_FFFF, C, 0x7FFF_FFFF, 0),
        (0, X | N, 0, Z | X),
        (0x1234_5678, 0, 0x1234_5678, 0),
    ];
    for (d0, ccr_in, want, ccr) in rows {
        assert_eq!(
            one("sats.l %d0", d0, ccr_in),
            (want, ccr),
            "d0={d0:#x} in={ccr_in:#x}"
        );
    }
}

#[test]
fn ff1_of_zero_is_32_and_sets_z() {
    // N is the msb of the source, Z is set when the source is zero, V and C clear, X unchanged.
    let rows: [(u32, u32, u8); 7] = [
        (0, 32, Z),
        (0x8000_0000, 0, N),
        (0x4000_0000, 1, 0),
        (0x0001_0000, 15, 0),
        (0x0000_0001, 31, 0),
        (0xFFFF_FFFF, 0, N),
        (0x0000_0002, 30, 0),
    ];
    for (d0, want, ccr) in rows {
        for ccr_in in [0, X | V | C, X | N | Z | V | C] {
            assert_eq!(
                one("ff1.l %d0", d0, ccr_in),
                (want, ccr | (ccr_in & X)),
                "d0={d0:#x} in={ccr_in:#x}"
            );
        }
    }
}

#[test]
fn bitrev_byterev_leave_the_ccr() {
    for ccr_in in [0, X | N | Z | V | C] {
        assert_eq!(
            one("bitrev.l %d0", 0x1234_5678, ccr_in),
            (0x1E6A_2C48, ccr_in)
        );
        assert_eq!(one("bitrev.l %d0", 0x8000_0000, ccr_in), (1, ccr_in));
        assert_eq!(
            one("byterev.l %d0", 0xAABB_CCDD, ccr_in),
            (0xDDCC_BBAA, ccr_in)
        );
        assert_eq!(
            one("byterev.l %d0", 0x0000_00FF, ccr_in),
            (0xFF00_0000, ccr_in)
        );
    }
}

#[test]
fn mov3q_sign_extends_minus_one_to_seven() {
    let rows: [(&str, u32, u8); 5] = [
        ("mov3q.l #-1,%d0", 0xFFFF_FFFF, N),
        ("mov3q.l #1,%d0", 1, 0),
        ("mov3q.l #7,%d0", 7, 0),
        ("mov3q.l #4,%d0", 4, 0),
        ("mov3q.l #2,%d0", 2, 0),
    ];
    for (insn, want, ccr) in rows {
        for ccr_in in [0, X | Z | V | C] {
            assert_eq!(
                one(insn, 0x5555_5555, ccr_in),
                (want, ccr | (ccr_in & X)),
                "{insn}"
            );
        }
    }
    let mut m = run_ok("mov3q.l #-1,(%a1)", |_| {});
    assert_eq!(m.mem.read(MEM, Size::Long).unwrap(), 0xFFFF_FFFF);
    let mut m = run_ok("mov3q.l #3,-(%a1)", |_| {});
    assert_eq!(m.mem.read(MEM - 4, Size::Long).unwrap(), 3);
    assert_eq!(m.cpu.a[1], MEM - 4);
}

#[test]
fn tpf_skips_its_extension_words() {
    // The words after TPF belong to it: 0x7000 is MOVEQ #0,D0 if it were executed.
    let m = run_ok("tpf\ntpf.w #0x7000\ntpf.l #0x70007000", |m| {
        m.cpu.d[0] = 5;
        m.cpu.set_sr(0x2700 | (X | N | Z | V | C) as u16);
    });
    assert_eq!(m.cpu.d[0], 5);
    assert_eq!(m.cpu.ccr(), X | N | Z | V | C);
    assert_eq!(m.instructions(), 3);
}
