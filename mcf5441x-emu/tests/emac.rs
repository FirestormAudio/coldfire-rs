//! The EMAC (CFPRM ch. 6, RM ch. 5). Expected values are worked out by hand from the CFPRM instruction pages and
//! the RM 5.3.5 multiply-accumulate algorithm, and written out as literals.
//!
//! The complete 48-bit accumulator is `{ACCextn[15:0], ACCn}` in integer mode and `{ACCextn[15:8], ACCn,
//! ACCextn[7:0]}` in fractional mode (RM 5.3). The tests look at the raw registers: `acc[n]` is ACCn and `ext[n]`
//! is ACCextn.

mod common;

use common::{BASE, MEM, X};
use mcf5441x_emu::{Machine, Size, Stop};

// MACSR bits (RM 5.2.1, Figure 5-2).
const PAV0: u32 = 0x100;
const PAV2: u32 = 0x400;
const PAV3: u32 = 0x800;
const OMC: u32 = 0x80;
const SU: u32 = 0x40;
const FI: u32 = 0x20;
const RT: u32 = 0x10;
const N: u32 = 0x08;
const Z: u32 = 0x04;
const V: u32 = 0x02;
const EV: u32 = 0x01;

/// Run `src` after setting D0-D7 from `d`.
fn run(src: &str, d: [u32; 8]) -> Machine {
    common::run_ok(src, |m| m.cpu.d = d)
}

/// One multiply-accumulate case: MACSR is loaded with `macsr`, ACC0 with `acc` (which also sets its extension the
/// way MOVE to ACC does in that mode), ACCext01 with `ext01` when given, then `insn` runs with D0 and D1.
struct Case {
    insn: &'static str,
    macsr: u32,
    acc: u32,
    ext01: Option<u32>,
    d0: u32,
    d1: u32,
    /// ACC0, ACCext0 and MACSR afterwards.
    want: (u32, u16, u32),
}

fn case(c: &Case) -> Machine {
    let ext = if c.ext01.is_some() {
        "move.l %d4,%accext01\n"
    } else {
        ""
    };
    let src = format!("move.l %d2,%macsr\nmove.l %d3,%acc0\n{ext}{}", c.insn);
    let m = run(
        &src,
        [c.d0, c.d1, c.macsr, c.acc, c.ext01.unwrap_or(0), 0, 0, 0],
    );
    let e = &m.cpu.emac;
    assert_eq!(
        (e.acc[0], e.ext[0], e.macsr),
        c.want,
        "{} macsr={:#x} acc={:#x} ext01={:x?} d0={:#x} d1={:#x}: got acc {:#x} ext {:#x} macsr {:#x}",
        c.insn,
        c.macsr,
        c.acc,
        c.ext01,
        c.d0,
        c.d1,
        e.acc[0],
        e.ext[0],
        e.macsr
    );
    m
}

fn cases(cs: &[Case]) {
    for c in cs {
        case(c);
    }
}

const fn row(
    insn: &'static str,
    macsr: u32,
    acc: u32,
    d0: u32,
    d1: u32,
    want: (u32, u16, u32),
) -> Case {
    Case {
        insn,
        macsr,
        acc,
        ext01: None,
        d0,
        d1,
        want,
    }
}

const fn row_ext(
    insn: &'static str,
    macsr: u32,
    acc: u32,
    ext01: u32,
    d0: u32,
    d1: u32,
    want: (u32, u16, u32),
) -> Case {
    Case {
        insn,
        macsr,
        acc,
        ext01: Some(ext01),
        d0,
        d1,
        want,
    }
}

/// Load MACSR and ACC0 (and ACCext01 when `ext01` is given) from D2, D3 and D4, then store ACC0 into D5.
fn store(macsr: u32, acc: u32, ext01: Option<u32>) -> u32 {
    let ext = if ext01.is_some() {
        "move.l %d4,%accext01\n"
    } else {
        ""
    };
    let src = format!("move.l %d2,%macsr\nmove.l %d3,%acc0\n{ext}move.l %acc0,%d5");
    run(&src, [0, 0, macsr, acc, ext01.unwrap_or(0), 0, 0, 0])
        .cpu
        .d[5]
}

#[test]
fn measured_mac_on_mcf54415() {
    // Measured on an MCF54415: D0 = 6, D1 = 7 gives D0 = 42.
    let m = run(
        "move.l #0,%macsr\nmove.l #0,%acc0\nmac.l %d0,%d1,%acc0\nmove.l %acc0,%d0",
        [6, 7, 0, 0, 0, 0, 0, 0],
    );
    assert_eq!(m.cpu.d[0], 42);
}

#[test]
fn mac_signed_integer_long() {
    // CFPRM ch. 6 MAC; RM 5.3.5 signed integers: the product is sign-extended to 48 bits and added; N is ACC[47],
    // Z the whole 48 bits, EV set when ACC[47:31] is not all zeros or all ones.
    let i = "mac.l %d0,%d1,%acc0";
    cases(&[
        row(i, 0, 0, 6, 7, (42, 0, 0)),
        row(i, 0, 10, 0xFFFF_FFFD, 5, (0xFFFF_FFFB, 0xFFFF, N)),
        row(i, 0, 0x7FFF_FFFF, 1, 1, (0x8000_0000, 0, EV)),
        row(
            i,
            0,
            0x8000_0000,
            0xFFFF_FFFF,
            1,
            (0x7FFF_FFFF, 0xFFFF, N | EV),
        ),
        row(i, 0, 5, 5, 0xFFFF_FFFF, (0, 0, Z)),
        // -2^31 * 2^8 = -2^39 still fits the 40-bit product.
        row(i, 0, 0, 0x8000_0000, 0x100, (0, 0xFF80, N | EV)),
    ]);
}

#[test]
fn mac_signed_integer_word_halves() {
    // CFPRM ch. 6 MAC: U/Lx and U/Ly pick the upper or lower word, sign-extended in signed mode (RM 5.3.5).
    // D0 = (-2, 3), D1 = (5, 7) as (upper, lower).
    let (d0, d1) = (0xFFFE_0003, 0x0005_0007);
    cases(&[
        row(
            "mac.w %d0u,%d1l,%acc0",
            0,
            0,
            d0,
            d1,
            (0xFFFF_FFF2, 0xFFFF, N),
        ),
        row("mac.w %d0l,%d1u,%acc0", 0, 0, d0, d1, (15, 0, 0)),
        row(
            "mac.w %d0u,%d1u,%acc0",
            0,
            0,
            d0,
            d1,
            (0xFFFF_FFF6, 0xFFFF, N),
        ),
        row("mac.w %d0l,%d1l,%acc0", 0, 0, d0, d1, (21, 0, 0)),
        row(
            "mac.w %d0l,%d1l,%acc0",
            0,
            0,
            0x8000_8000,
            0x8000_8000,
            (0x4000_0000, 0, 0),
        ),
    ]);
}

#[test]
fn mac_unsigned_integer() {
    // RM 5.3.5 unsigned integers: operands and product zero-extended; EV set when ACC[47:32] is not zero; N is
    // ACC[47]. MOVE to ACC clears the extension in this mode (CFPRM ch. 6 MOVE to ACC).
    let l = "mac.l %d0,%d1,%acc0";
    cases(&[
        row(l, SU, 0, 0xFFFF_FFFF, 2, (0xFFFF_FFFE, 0x0001, SU | EV)),
        row(
            "mac.w %d0u,%d1l,%acc0",
            SU,
            0,
            0xFFFF_0000,
            0x0000_FFFF,
            (0xFFFE_0001, 0, SU),
        ),
        row(l, SU, 0x8000_0000, 0x8000_0000, 1, (0, 0x0001, SU | EV)),
        row(
            "mac.w %d0l,%d1l,%acc0",
            SU,
            0,
            0xFFFF,
            2,
            (0x0001_FFFE, 0, SU),
        ),
    ]);
}

#[test]
fn mac_fractional_rounds_to_nearest() {
    // RM 5.3.5 signed fractionals with R/T = 1: product = (Y * X) << 1, rounded to nearest (even) at bit 24, then
    // product[71:24] is added. Fractional ACC0 = {ext[15:8], ACC0, ext[7:0]}, so a sum of 2 shows as ext 0x0002.
    // RM 5.3.1.1 gives the rounding rule; RM 5.2.1 Table 5-3 the mode.
    let i = "mac.l %d0,%d1,%acc0";
    let m = FI | RT;
    cases(&[
        // 0x800000 exactly, bit 24 clear: round down.
        row(i, m, 0, 0x0040_0000, 1, (0, 0, m | Z)),
        // 0x1800000: 0x800000 with bit 24 set: round up to 2.
        row(i, m, 0, 0x00C0_0000, 1, (0, 0x0002, m)),
        // Above one half: round up.
        row(i, m, 0, 0x0040_0001, 1, (0, 0x0001, m)),
        // -0x800000: product[71:24] = -1 and the low bits are exactly one half with bit 24 set: up to 0.
        row(i, m, 0, 0xFFC0_0000, 1, (0, 0, m | Z)),
        // (1 - 2^-31)^2: low bits 0x000002, no rounding.
        row(i, m, 0, 0x7FFF_FFFF, 0x7FFF_FFFF, (0x7FFF_FFFE, 0, m)),
        // -1 * -1: product[71:64] is zero-filled, so +1.0 = 0x0080_0000_0000, beyond 40 bits: EV.
        row(i, m, 0, 0x8000_0000, 0x8000_0000, (0x8000_0000, 0, m | EV)),
        // Word operands are {Rn.half, 0x0000}: 0.5 * 0.5 = 0.25.
        row(
            "mac.w %d0u,%d1u,%acc0",
            m,
            0,
            0x4000_0000,
            0x4000_0000,
            (0x2000_0000, 0, m),
        ),
    ]);

    // Stores (CFPRM ch. 6 MOVE from ACC): S/U = 0, R/T = 1 rounds ACC[39:8] by ACC[7:0] (the low extension byte).
    assert_eq!(store(m, 0x1234, Some(0x80)), 0x1234, "tie, even stays");
    assert_eq!(store(m, 0x1234, Some(0x81)), 0x1235);
    assert_eq!(store(m, 0x1235, Some(0x80)), 0x1236, "tie, odd rounds up");
    assert_eq!(
        store(m, 0x7FFF_FFFF, Some(0x80)),
        0x8000_0000,
        "no saturation: wraps"
    );
    assert_eq!(
        store(m | OMC, 0x7FFF_FFFF, Some(0x80)),
        0x7FFF_FFFF,
        "saturated after rounding"
    );
    // S/U = 1: ACC[39:24] rounded by ACC[23:0] into the low word, upper word zero.
    let s16 = FI | SU;
    assert_eq!(store(s16, 0x1234_8000, Some(0)), 0x1234);
    assert_eq!(store(s16, 0x1235_8000, Some(0)), 0x1236);
    assert_eq!(store(s16, 0x1234_8000, Some(0x0001)), 0x1235);
    assert_eq!(
        store(s16, 0xFFFF_8000, Some(0xFF00)),
        0,
        "-1/2 lsb rounds up to 0"
    );
    assert_eq!(store(s16, 0x7FFF_8000, Some(0)), 0x8000, "no saturation");
    assert_eq!(
        store(s16 | OMC, 0x7FFF_8000, Some(0)),
        0x7FFF,
        "saturates positive"
    );
    assert_eq!(
        store(s16 | OMC, 0, Some(0xFF00)),
        0x8000,
        "saturates negative"
    );
    assert_eq!(
        store(s16 | RT, 0x1234_8000, Some(0)),
        0x1234,
        "R/T does not change 16-bit rounding"
    );
}

#[test]
fn mac_fractional_truncates_when_t() {
    // RM 5.3.5 signed fractionals with R/T = 0: product[71:24] with the low 24 bits dropped (RM 5.2.1 R/T).
    let i = "mac.l %d0,%d1,%acc0";
    let m = FI;
    cases(&[
        row(i, m, 0, 0x00C0_0000, 1, (0, 0x0001, m)),
        row(i, m, 0, 0x0040_0001, 1, (0, 0, m | Z)),
        // -0x800000 truncates towards minus infinity: -1 over all 48 bits.
        row(i, m, 0, 0xFFC0_0000, 1, (0xFFFF_FFFF, 0xFFFF, m | N)),
        row(i, m, 0, 0x7FFF_FFFF, 0x7FFF_FFFF, (0x7FFF_FFFE, 0, m)),
        row(
            "mac.w %d0u,%d1u,%acc0",
            m,
            0,
            0x4000_0000,
            0x4000_0000,
            (0x2000_0000, 0, m),
        ),
    ]);
    // Stores truncate the low extension byte.
    assert_eq!(store(m, 0x1234, Some(0xFF)), 0x1234);
    assert_eq!(store(m, 0x8000_0000, Some(0)), 0x8000_0000);
}

#[test]
fn msac_subtracts() {
    // CFPRM ch. 6 MSAC: ACCx - (Ry * Rx) -> ACCx.
    let l = "msac.l %d0,%d1,%acc0";
    cases(&[
        row(l, 0, 100, 6, 7, (58, 0, 0)),
        row(l, 0, 0, 6, 7, (0xFFFF_FFD6, 0xFFFF, N)),
        // Unsigned 10 - 20 borrows out of the 48-bit accumulator: an accumulation overflow (RM 5.3.5), which
        // without OMC wraps.
        row(
            l,
            SU,
            10,
            4,
            5,
            (0xFFFF_FFF6, 0xFFFF, SU | PAV0 | N | V | EV),
        ),
        // Fractional 0 - 0.25 = 0xFFE0_0000_0000.
        row(
            "msac.w %d0u,%d1u,%acc0",
            FI,
            0,
            0x4000_0000,
            0x4000_0000,
            (0xE000_0000, 0xFF00, FI | N),
        ),
    ]);
}

#[test]
fn scale_factor_shifts() {
    // RM 5.3.5: SF << 1 shifts a zero in; >> 1 shifts the sign in (signed) or a zero (unsigned); SF is ignored in
    // fractional mode (RM 5.3.1.4); the reserved encoding 10 does not scale.
    cases(&[
        row("mac.l %d0,%d1,<<,%acc0", 0, 0, 6, 7, (84, 0, 0)),
        row("mac.l %d0,%d1,>>,%acc0", 0, 0, 5, 7, (17, 0, 0)),
        row(
            "mac.l %d0,%d1,>>,%acc0",
            0,
            0,
            0xFFFF_FFF9,
            5,
            (0xFFFF_FFEE, 0xFFFF, N),
        ),
        row(
            "mac.l %d0,%d1,<<,%acc0",
            0,
            0,
            0x7FFF_FFFF,
            2,
            (0xFFFF_FFFC, 0x0001, EV),
        ),
        // 0xFF * 0xFFFFFFFF = 0xFE_FFFF_FF01, >> 1 = 0x7F_7FFF_FF80.
        row(
            "mac.l %d0,%d1,>>,%acc0",
            SU,
            0,
            0xFFFF_FFFF,
            0xFF,
            (0x7FFF_FF80, 0x007F, SU | EV),
        ),
        row(
            "mac.w %d0u,%d1u,<<,%acc0",
            FI,
            0,
            0x4000_0000,
            0x4000_0000,
            (0x2000_0000, 0, FI),
        ),
        row(
            "mac.w %d0u,%d1u,>>,%acc0",
            FI,
            0,
            0x4000_0000,
            0x4000_0000,
            (0x2000_0000, 0, FI),
        ),
        // mac.l %d0,%d1,%acc0 with the scale factor field 10.
        row(".short 0xa200,0x0c00", 0, 0, 6, 7, (42, 0, 0)),
    ]);
}

#[test]
fn omc_saturates_and_sets_pav() {
    // RM 5.3.5 with OMC: a product or 48-bit accumulation overflow sets PAVn and V and saturates the accumulator
    // (signed integer to the 32-bit limits, unsigned to all ones or zero, fractional to the 40-bit limits).
    let l = "mac.l %d0,%d1,%acc0";
    let ms = "msac.l %d0,%d1,%acc0";
    let big = 0x7FFF_FFFF;
    cases(&[
        // 0x7FFF_FFFF_FFFF + 1 overflows 48 bits: 0x0000_7FFF_FFFF.
        row_ext(
            l,
            OMC,
            0xFFFF_FFFF,
            0x7FFF,
            1,
            1,
            (0x7FFF_FFFF, 0, OMC | PAV0 | V),
        ),
        // -2^47 - 1: 0xFFFF_8000_0000.
        row_ext(
            ms,
            OMC,
            0,
            0x8000,
            1,
            1,
            (0x8000_0000, 0xFFFF, OMC | PAV0 | N | V),
        ),
        // A positive product that does not fit 40 bits: MAC saturates positive, MSAC negative.
        row(l, OMC, 0, big, big, (0x7FFF_FFFF, 0, OMC | PAV0 | V)),
        row(
            ms,
            OMC,
            0,
            big,
            big,
            (0x8000_0000, 0xFFFF, OMC | PAV0 | N | V),
        ),
        // A negative product that does not fit: the other way round.
        row(
            l,
            OMC,
            0,
            0x8000_0000,
            0xFFFF_FF00,
            (0x7FFF_FFFF, 0, OMC | PAV0 | V),
        ),
        // Beyond 32 bits but inside 48: EV only, no saturation.
        row(l, OMC, 0x7FFF_FFFF, 1, 1, (0x8000_0000, 0, OMC | EV)),
        // Unsigned: product overflow saturates MAC to all ones, MSAC to zero; a borrow saturates to zero.
        row(
            l,
            OMC | SU,
            0,
            0xFFFF_FFFF,
            0xFFFF_FFFF,
            (0xFFFF_FFFF, 0xFFFF, OMC | SU | PAV0 | N | V | EV),
        ),
        row(
            ms,
            OMC | SU,
            0,
            0xFFFF_FFFF,
            0xFFFF_FFFF,
            (0, 0, OMC | SU | PAV0 | Z | V),
        ),
        row(ms, OMC | SU, 10, 4, 5, (0, 0, OMC | SU | PAV0 | Z | V)),
        // Fractional: 0x7FFF_FFFF_FFFF + 0.25 saturates to 0x007F_FFFF_FF00, -2^47 - 0.25 to 0xFF80_0000_0000.
        row_ext(
            "mac.w %d0u,%d1u,%acc0",
            OMC | FI,
            0xFFFF_FFFF,
            0x7FFF,
            0x4000_0000,
            0x4000_0000,
            (0x7FFF_FFFF, 0, OMC | FI | PAV0 | V),
        ),
        row_ext(
            "msac.w %d0u,%d1u,%acc0",
            OMC | FI,
            0,
            0x8000,
            0x4000_0000,
            0x4000_0000,
            (0x8000_0000, 0xFF00, OMC | FI | PAV0 | N | V),
        ),
        // -1 * -1 = +1.0 needs the extension but does not overflow 48 bits.
        row(
            l,
            OMC | FI,
            0,
            0x8000_0000,
            0x8000_0000,
            (0x8000_0000, 0, OMC | FI | EV),
        ),
    ]);

    // Stores saturate what does not fit 32 bits (CFPRM ch. 6 MOVE from ACC).
    let st = |macsr: u32, d0: u32, d1: u32| {
        run(
            "move.l %d2,%macsr\nmove.l %d3,%acc0\nmac.l %d0,%d1,%acc0\nmove.l %acc0,%d5",
            [d0, d1, macsr, 0, 0, 0, 0, 0],
        )
        .cpu
        .d[5]
    };
    assert_eq!(
        st(OMC, 0x8000_0000, 0x100),
        0x8000_0000,
        "-2^39 saturates negative"
    );
    assert_eq!(st(0, 0x8000_0000, 0x100), 0, "without OMC: ACC[31:0]");
    assert_eq!(st(OMC | SU, 0xFFFF_FFFF, 2), 0xFFFF_FFFF);
    assert_eq!(st(SU, 0xFFFF_FFFF, 2), 0xFFFF_FFFE);
    assert_eq!(
        st(OMC | FI, 0x8000_0000, 0x8000_0000),
        0x7FFF_FFFF,
        "+1.0 saturates"
    );
    assert_eq!(
        st(FI, 0x8000_0000, 0x8000_0000),
        0x8000_0000,
        "+1.0 without OMC: ACC[39:8]"
    );
}

#[test]
fn pav_is_sticky_until_cleared() {
    // RM 5.2.1: PAVn stays set until MACSR is written or ACCn is loaded; with OMC a saturated accumulator ignores
    // MAC and MSAC until then (RM 5.3.5: nothing happens when OMC and PAVn are both set), and V follows PAVn.
    let src = "
        move.l #0x80,%macsr
        move.l #-1,%acc0
        move.l #0x7fff,%accext01
        mac.l %d0,%d1,%acc0     | saturates: 0x0000_7FFF_FFFF, PAV0
        move.l %macsr,%d2
        msac.l %d0,%d1,%acc0    | ignored
        move.l %macsr,%d3
        move.l %acc0,%d4
        mac.l %d0,%d1,%acc1     | ACC1 has its own flag: V = PAV1 = 0
        move.l %macsr,%d5
        move.l %acc1,%d6
        move.l %acc0,%acc2      | copies PAV0 into PAV2
        move.l %macsr,%d7
        move.l %d0,%acc0        | a load clears PAV0 only
        move.l %macsr,%a2
        mac.l %d0,%d1,%acc0     | works again: 1 + 1
        move.l %acc0,%a3
        move.l #0x80,%macsr     | a MACSR write clears PAV2
        move.l %acc2,%acc3
        move.l %macsr,%a4
    ";
    let m = run(src, [1, 1, 0, 0, 0, 0, 0, 0]);
    let d = m.cpu.d;
    assert_eq!(d[2], OMC | PAV0 | V);
    assert_eq!(d[3], OMC | PAV0 | V, "N, Z, EV from the unchanged ACC0");
    assert_eq!(d[4], 0x7FFF_FFFF);
    assert_eq!(d[5], OMC | PAV0);
    assert_eq!(d[6], 1);
    assert_eq!(d[7], OMC | PAV0 | PAV2 | V);
    assert_eq!(m.cpu.a[2], OMC | PAV2);
    assert_eq!(m.cpu.a[3], 2);
    assert_eq!(m.cpu.a[4], OMC);

    // guess: without OMC RM 5.3.5's algorithm clears PAVn before each MAC, but RM Table 5-2 ("once set, each flag
    // remains set until V is cleared by a move.l, MACSR instruction or the accumulator is loaded directly") and
    // every CFPRM ch. 6 MAC/MSAC page ("PAVx: ... unchanged otherwise") make it sticky; this core follows those.
    // 0x7FFF_FFFF_FFFF + 1 wraps (accumulation overflow), then 0 * 0 keeps PAV0 and V.
    let src = "
        move.l #0,%macsr
        move.l #-1,%acc0
        move.l #0x7fff,%accext01
        mac.l %d0,%d0,%acc0
        move.l %macsr,%d2
        mac.l %d1,%d1,%acc0
        move.l %macsr,%d3
    ";
    let m = run(src, [1, 0, 0, 0, 0, 0, 0, 0]);
    assert_eq!(m.cpu.emac.acc[0], 0);
    assert_eq!(m.cpu.emac.ext[0], 0x8000);
    assert_eq!(m.cpu.d[2], PAV0 | N | V | EV);
    assert_eq!(m.cpu.d[3], PAV0 | N | V | EV);
}

#[test]
fn mac_product_overflow_without_omc() {
    // guess: RM 5.3.5 sets PAVn and V on a product overflow but assigns no result when OMC is clear; this core
    // accumulates the low 40 bits of the product, sign-extended, as for any integer product (RM 5.3: "the
    // low-order 40 bits of the product are formed and used with the destination accumulator").
    // 0x7FFF_FFFF^2 = 0x3FFF_FFFF_0000_0001; its low 40 bits 0xFF_0000_0001 read as -0xFFFF_FFFF.
    case(&row(
        "mac.l %d0,%d1,%acc0",
        0,
        0,
        0x7FFF_FFFF,
        0x7FFF_FFFF,
        (0x0000_0001, 0xFFFF, PAV0 | N | V | EV),
    ));
}

#[test]
fn mac_with_load_moves_and_masks_address() {
    // CFPRM ch. 6 MAC with load: the multiply-accumulate plus (<ea>y) -> Rw; ACC lsb is the inverse of opword bit
    // 7. RM 5.2.2: with `&`, (An) uses An & {0xFFFF, MASK}; (An)+ uses An and leaves (An + 4) & mask in An;
    // -(An) uses and leaves (An - 4) & mask; (d16,An) uses (An + d16) & mask.
    let prepare = |m: &mut Machine| {
        for (i, v) in [
            0x1111_1111u32,
            0x2222_2222,
            0x3333_3333,
            0x4444_4444,
            0,
            0x6666_6666,
        ]
        .into_iter()
        .enumerate()
        {
            m.mem.write(MEM + 4 * i as u32, Size::Long, v).unwrap();
        }
        m.mem.write(0x401F_FFEC, Size::Long, 0x5555_5555).unwrap();
        m.cpu.d[0] = 6;
        m.cpu.d[1] = 7;
    };
    let go = |insn: &str, a0: u32| {
        common::run_ok(&format!("move.l #0xffef,%mask\n{insn}"), |m| {
            prepare(m);
            m.cpu.a[0] = a0;
        })
    };

    let m = go("mac.l %d0,%d1,(%a0)+,%d2,%acc0", MEM);
    assert_eq!(
        (m.cpu.d[2], m.cpu.a[0], m.cpu.emac.acc[0]),
        (0x1111_1111, MEM + 4, 42)
    );

    // 0x4020000C + 4 = 0x40200010, & 0xFFFFFFEF = 0x40200000: a 16-byte circular buffer.
    let m = go("mac.l %d0,%d1,(%a0)+&,%d2,%acc1", MEM + 12);
    assert_eq!((m.cpu.d[2], m.cpu.a[0]), (0x4444_4444, MEM));
    assert_eq!(
        (m.cpu.emac.acc[0], m.cpu.emac.acc[1]),
        (0, 42),
        "ACC1 selected"
    );

    // (0x40200000 - 4) & 0xFFFFFFEF = 0x401FFFEC; word upper x lower, << 1, subtracted from ACC3.
    let m = common::run_ok(
        "move.l #0xffef,%mask\nmsac.w %d0u,%d1l,<<,-(%a0)&,%a2,%acc3",
        |m| {
            prepare(m);
            m.cpu.a[0] = MEM;
            m.cpu.d[0] = 0x0006_0000;
        },
    );
    assert_eq!((m.cpu.a[2], m.cpu.a[0]), (0x5555_5555, 0x401F_FFEC));
    assert_eq!(
        (m.cpu.emac.acc[3], m.cpu.emac.ext[3]),
        (0xFFFF_FFAC, 0xFFFF)
    );
    assert_eq!(m.cpu.emac.macsr, N);

    // 0x40200000 + 20 = 0x40200014, masked to 0x40200004.
    let m = go("mac.l %d0,%d1,20(%a0)&,%d2,%acc2", MEM);
    assert_eq!(
        (m.cpu.d[2], m.cpu.a[0], m.cpu.emac.acc[2]),
        (0x2222_2222, MEM, 42)
    );
    let m = go("mac.l %d0,%d1,(%a0)&,%d2,%acc0", MEM + 0x18);
    assert_eq!(m.cpu.d[2], 0x3333_3333);
    // Without `&` the mask is not used.
    let m = go("mac.l %d0,%d1,20(%a0),%d2,%acc0", MEM);
    assert_eq!(m.cpu.d[2], 0x6666_6666);

    // The multiply uses Rx as it was before the load replaces it.
    let m = go("mac.l %d0,%d1,(%a0),%d1,%acc0", MEM);
    assert_eq!((m.cpu.d[1], m.cpu.emac.acc[0]), (0x1111_1111, 42));

    // guess: the manuals do not say which wins when Rw is the address register stepped by (Ay)+; as for a 68K
    // MOVE.L (A0)+,A0, the loaded value does.
    let m = go("mac.l %d0,%d1,(%a0)+,%a0,%acc0", MEM);
    assert_eq!(m.cpu.a[0], 0x1111_1111);

    // A load that faults leaves the address register, Rw and the EMAC as they were.
    let (m, r) = common::run(
        "move.l #0xffef,%mask\nmac.l %d0,%d1,(%a0)+,%d2,%acc0",
        |m| {
            prepare(m);
            m.cpu.a[0] = 0x5000_0000;
        },
    );
    assert_eq!((r.kind, r.pc), (Stop::Unmapped, BASE + 6), "{r:?}");
    assert_eq!((m.cpu.a[0], m.cpu.d[2]), (0x5000_0000, 0));
    assert_eq!((m.cpu.emac.acc[0], m.cpu.emac.macsr), (0, 0));
}

#[test]
fn accext_round_trip() {
    // CFPRM ch. 6 MOVE to/from ACCext01 and ACCext23.
    // guess: the CFPRM tables put ACCext1 (ACCext3) in bits 31-16 and ACCext0 (ACCext2) in bits 15-0; RM Figure 5-5
    // draws ACC0U/ACC0L in bits 31-16 instead. This core follows the instruction pages.
    let m = run(
        "move.l %d0,%accext01\nmove.l %d1,%accext23\nmove.l %accext01,%d2\nmove.l %accext23,%d3",
        [0x1234_5678, 0x9ABC_DEF0, 0, 0, 0, 0, 0, 0],
    );
    assert_eq!((m.cpu.d[2], m.cpu.d[3]), (0x1234_5678, 0x9ABC_DEF0));
    assert_eq!(m.cpu.emac.ext, [0x5678, 0x1234, 0xDEF0, 0x9ABC]);
    assert_eq!(m.cpu.emac.macsr, 0, "MACSR not affected");

    // The extension's place in the 48-bit accumulator depends on the mode (RM 5.3): ext0 = 0x0001 is 2^32 in
    // integer mode (EV) and 1 in fractional mode.
    let src = "move.l %d2,%macsr\nmove.l #0,%acc0\nmove.l #1,%accext01\nmac.l %d0,%d0,%acc0";
    assert_eq!(run(src, [0; 8]).cpu.emac.macsr, EV);
    assert_eq!(run(src, [0, 0, FI, 0, 0, 0, 0, 0]).cpu.emac.macsr, FI);

    // RM 5.3.1.2's save and restore sequence reproduces every register exactly.
    let src = "
        move.l #0x1234,%mask
        move.l #0x80000001,%acc0
        move.l #0x7fffffff,%acc1
        move.l #3,%acc2
        move.l #-4,%acc3
        move.l #0x0102a0b0,%accext01
        move.l #0x0c0dfeed,%accext23
        move.l #0x0f3a,%macsr
        lea -32(%sp),%sp
        move.l %macsr,%d7
        clr.l %d0
        move.l %d0,%macsr
        move.l %acc0,%d0
        move.l %acc1,%d1
        move.l %acc2,%d2
        move.l %acc3,%d3
        move.l %accext01,%d4
        move.l %accext23,%d5
        move.l %mask,%d6
        movem.l %d0-%d7,(%sp)
        move.l #0,%acc0
        move.l #0,%acc1
        move.l #0,%acc2
        move.l #0,%acc3
        move.l #0,%accext01
        move.l #0,%accext23
        move.l #0,%mask
        movem.l (%sp),%d0-%d7
        move.l #0,%macsr
        move.l %d0,%acc0
        move.l %d1,%acc1
        move.l %d2,%acc2
        move.l %d3,%acc3
        move.l %d4,%accext01
        move.l %d5,%accext23
        move.l %d6,%mask
        move.l %d7,%macsr
    ";
    let e = run(src, [0; 8]).cpu.emac;
    assert_eq!(e.acc, [0x8000_0001, 0x7FFF_FFFF, 3, 0xFFFF_FFFC]);
    assert_eq!(e.ext, [0xA0B0, 0x0102, 0xFEED, 0x0C0D]);
    assert_eq!((e.macsr, e.mask), (0x0F3A, 0xFFFF_1234));
}

#[test]
fn move_to_acc_extends_by_mode() {
    // CFPRM ch. 6 MOVE to ACC: signed integer sign-extends into all 16 extension bits, unsigned clears them,
    // fractional sign-extends into the upper byte and clears the lower; PAVx, V and EV are cleared.
    // guess: "N set if the msb of the result is set" is read as ACC[47], as for MAC (RM 5.3.5), so an unsigned load
    // never sets N.
    let go = |macsr: u32, v: u32| {
        let m = run(
            "move.l %d2,%macsr\nmove.l %d3,%acc0",
            [0, 0, macsr, v, 0, 0, 0, 0],
        );
        (m.cpu.emac.acc[0], m.cpu.emac.ext[0], m.cpu.emac.macsr)
    };
    assert_eq!(go(0, 0x8000_0000), (0x8000_0000, 0xFFFF, N));
    assert_eq!(go(SU, 0x8000_0000), (0x8000_0000, 0, SU));
    assert_eq!(go(FI, 0x8000_0000), (0x8000_0000, 0xFF00, FI | N));
    assert_eq!(go(0, 0), (0, 0, Z));
    assert_eq!(go(PAV0 | N | Z | V | EV, 1), (1, 0, 0), "PAV0 V EV cleared");
    assert_eq!(go(0x200 | PAV0 | V, 1), (1, 0, 0x200), "PAV1 kept");

    // MOVE ACC to ACC copies all 48 bits and the PAV flag; V follows the copied flag (CFPRM ch. 6).
    let m = run(
        "move.l #-1,%acc0\nmove.l #0x100,%macsr\nmove.l %acc0,%acc3",
        [0; 8],
    );
    assert_eq!(
        (m.cpu.emac.acc[3], m.cpu.emac.ext[3]),
        (0xFFFF_FFFF, 0xFFFF)
    );
    assert_eq!(m.cpu.emac.macsr, PAV0 | PAV3 | N | V);
}

#[test]
fn movclr_clears_acc() {
    // CFPRM ch. 6 MOVCLR: the store of MOVE from ACC (here saturating), then ACCx, its extension and PAVx are
    // cleared; N Z V EV are not affected. MOVE from ACC leaves the accumulator alone.
    let src = "
        move.l #0x80,%macsr
        move.l #0x7fffffff,%acc0
        mac.l %d0,%d0,%acc0
        move.l #0x181,%macsr
        move.l %acc0,%d4
        movclr.l %acc0,%d3
    ";
    let m = run(src, [1, 0, 0, 0, 0, 0, 0, 0]);
    assert_eq!((m.cpu.d[3], m.cpu.d[4]), (0x7FFF_FFFF, 0x7FFF_FFFF));
    assert_eq!((m.cpu.emac.acc[0], m.cpu.emac.ext[0]), (0, 0));
    assert_eq!(m.cpu.emac.macsr, OMC | EV);

    let m = run("move.l #42,%acc1\nmovclr.l %acc1,%a3", [0; 8]);
    assert_eq!((m.cpu.a[3], m.cpu.emac.acc[1]), (42, 0));
}

#[test]
fn macsr_to_ccr() {
    // CFPRM ch. 6 MOVE MACSR to CCR: X cleared, N Z V from MACSR, C from EV.
    for (macsr, ccr) in [(0x0F, 0x0F), (0x0A, 0x0A), (0x05, 0x05), (0x1F0, 0)] {
        let m = common::run_ok("move.l %d2,%macsr\nmove.l %macsr,%ccr", |m| {
            m.cpu.d[2] = macsr;
            m.cpu.set_ccr(X | 0x0F);
        });
        assert_eq!(m.cpu.ccr(), ccr, "macsr {macsr:#x}");
    }
    // MOVE from MACSR clears bits 31-12; MOVE from MASK sets bits 31-16 (CFPRM ch. 6, RM 5.2.2).
    let m = run(
        "move.l %mask,%d4\nmove.l #-1,%macsr\nmove.l %macsr,%d2\nmove.l #0x12345678,%mask\nmove.l %mask,%d3",
        [0; 8],
    );
    assert_eq!((m.cpu.d[2], m.cpu.d[3]), (0x0000_0FFF, 0xFFFF_5678));
    assert_eq!(m.cpu.d[4], 0xFFFF_FFFF, "MASK resets to all ones");
}

#[test]
fn dual_accumulate_encodings_are_illegal() {
    // guess: extension bits 1-0 select the EMAC_B dual-accumulate forms (CFPRM ch. 6 MAAAC...), which RM ch. 5 does
    // not list for this core; they take the illegal-instruction exception.
    let (m, r) = common::run(".short 0xa200,0x0801", |m| m.cpu.d[0] = 6);
    assert_eq!((r.kind, r.pc), (Stop::Illegal, BASE), "{r:?}");
    assert_eq!(m.cpu.emac.acc[0], 0);
}
