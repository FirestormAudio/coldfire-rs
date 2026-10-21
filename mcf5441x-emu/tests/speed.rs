//! Interpreter speed, on a bitwise (table-less) CRC-32. Run it in a release build:
//! `cargo test --release -p mcf5441x-emu --test speed -- --ignored --nocapture`.

mod common;

use common::{prepare, until};
use mcf5441x_emu::Stop;

/// Where the buffer is.
const BUF: u32 = 0x4030_0000;
/// Its length.
const LEN: usize = 1024;
/// Bytes fed through the CRC: the buffer over and over, one loop iteration per byte.
const STEPS: u32 = 2_000_000;

/// The reflected CRC-32 (polynomial 0xEDB88320) of `STEPS` bytes taken round-robin from the buffer, bit by bit:
/// per byte 3 + 8 x 4.5 (taking the XOR half the time) + 4 instructions, about 86 million in all.
const CRC: &str = "\
        lea 0x40300000,%a1\n\
        move.l #2000000,%d3\n\
        moveq #-1,%d0\n\
        move.l #0xedb88320,%d4\n\
    outer: move.l %a1,%a0\n\
        move.l #1024,%d2\n\
    byte: mvz.b (%a0)+,%d1\n\
        eor.l %d1,%d0\n\
        moveq #7,%d5\n\
    bit: lsr.l #1,%d0\n\
        bcc.s 1f\n\
        eor.l %d4,%d0\n\
    1:  subq.l #1,%d5\n\
        bpl.s bit\n\
        subq.l #1,%d3\n\
        beq.s done\n\
        subq.l #1,%d2\n\
        bne.s byte\n\
        bra.s outer\n\
    done: not.l %d0\n\
    end: nop";

fn buffer() -> Vec<u8> {
    (0..LEN).map(|i| (i * 37 + 11) as u8).collect()
}

fn crc32_reference(buf: &[u8]) -> u32 {
    let mut crc = !0u32;
    for i in 0..STEPS as usize {
        crc ^= buf[i % buf.len()] as u32;
        for _ in 0..8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0xEDB8_8320
            } else {
                crc >> 1
            };
        }
    }
    !crc
}

#[test]
#[ignore = "a release-build benchmark"]
fn core_runs_at_least_150_mips() {
    let (mut m, l) = prepare(CRC, &["end"]);
    let buf = buffer();
    m.mem.load(BUF, &buf).unwrap();
    let t = std::time::Instant::now();
    let r = m.run(until(l[0]));
    let secs = t.elapsed().as_secs_f64();
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.cpu.d[0], crc32_reference(&buf));
    let mips = m.instructions() as f64 / secs / 1e6;
    println!(
        "CRC-32: {} instructions in {secs:.3} s: {mips:.1} MIPS",
        m.instructions()
    );
    assert!(mips >= 150.0, "{mips:.1} MIPS");
}
