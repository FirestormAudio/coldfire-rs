mod common;

use std::collections::BTreeMap;
use std::process::Command;

use mcf5441x_emu::Size;
use mcf5441x_emu::isa::{self, Isa, Op, Table};

const BASE: u32 = 0x4010_0000;

/// Read big-endian words out of an assembled program loaded at `BASE`.
fn words(bytes: &[u8]) -> impl Fn(u32) -> Option<u16> + '_ {
    move |addr: u32| {
        let off = addr.checked_sub(BASE)? as usize;
        let b = bytes.get(off..off + 2)?;
        Some(u16::from_be_bytes([b[0], b[1]]))
    }
}

#[test]
fn table_has_no_ambiguous_opwords() {
    // The shipped description builds: every opword matches at most one form after the most-specific rule.
    let table = Table::parse(isa::SOURCE).unwrap_or_else(|e| panic!("{e}"));
    // Every `Op` variant is described by at least one line.
    for op in Op::ALL {
        assert!(
            table.forms().iter().any(|f| f.id == *op),
            "{op:?} has no line in the description"
        );
    }
    // Two forms with the same number of fixed bits that both accept an opword are rejected loudly.
    let overlapping = "\
nop  A  -  0100111001110001  none  -  Nop  # CFPRM ch. 4 NOP, p. 4-63
pulse  A  -  0100111001110001  none  -  Pulse  # CFPRM ch. 4 PULSE, p. 4-69
";
    let err = Table::parse(overlapping).unwrap_err();
    assert!(err.contains("ambiguous"), "{err}");
    // A line without a manual citation is rejected.
    let uncited = "nop  A  -  0100111001110001  none  -  Nop\n";
    assert!(Table::parse(uncited).unwrap_err().contains("CFPRM"));
}

#[test]
fn known_opwords_decode() {
    let lines: &[(&str, &str, Option<Size>)] = &[
        ("nop", "nop", None),
        ("rts", "rts", None),
        ("moveq #1,%d0", "moveq", Some(Size::Long)),
        ("mov3q #-1,%d0", "mov3q", Some(Size::Long)),
        ("mvs.b %d1,%d0", "mvs", Some(Size::Byte)),
        ("mvz.w %d1,%d0", "mvz", Some(Size::Word)),
        ("ff1 %d0", "ff1", Some(Size::Long)),
        ("bitrev %d0", "bitrev", Some(Size::Long)),
        ("byterev %d0", "byterev", Some(Size::Long)),
        ("sats %d0", "sats", Some(Size::Long)),
        ("remu.l %d1,%d2:%d0", "div", Some(Size::Long)),
        ("mac.l %d1,%d2,%acc0", "mac", None),
        ("movem.l %d0-%d7,(%a7)", "movem", Some(Size::Long)),
        ("movec %d0,%vbr", "movec", Some(Size::Long)),
        ("stop #0x2700", "stop", None),
    ];
    let src: String = lines.iter().map(|(l, _, _)| format!("    {l}\n")).collect();
    let bin = common::assemble(&src, BASE);
    let mem = words(&bin);
    let mut pc = BASE;
    for (line, mnemonic, size) in lines {
        let opword = mem(pc).unwrap();
        let form =
            isa::decode(opword).unwrap_or_else(|| panic!("{line}: {opword:#06x} does not decode"));
        assert_eq!(form.mnemonic, *mnemonic, "{line}: {opword:#06x}");
        assert_eq!(form.size, *size, "{line}: {opword:#06x}");
        pc += isa::length(&mem, pc).unwrap_or_else(|| panic!("{line}: no length"));
    }
    assert_eq!(
        pc,
        BASE + bin.len() as u32,
        "lengths walk the program exactly"
    );

    // Field extraction without per-instruction bit twiddling.
    let moveq = isa::decode(0x7e85).unwrap(); // moveq #-123,%d7
    assert_eq!(moveq.id, Op::Moveq);
    assert_eq!(moveq.fields.get('R', 0x7e85), Some(7));
    assert_eq!(moveq.fields.get('q', 0x7e85), Some(0x85));
    let movea = isa::decode(0x2c6a).unwrap(); // movea.l (d16,%a2),%a6
    assert_eq!(movea.id, Op::Movea);
    assert_eq!(movea.fields.ea(0x2c6a), Some((5, 2)));
    assert_eq!(movea.fields.get('R', 0x2c6a), Some(6));
}

#[test]
fn fpu_and_cau_opwords_are_recognised_as_such() {
    let fpu = [
        "fmove.d %fp0,%fp1",
        "fadd.d (%a0),%fp1",
        "fmovem.d %fp0-%fp7,(%a7)",
        "fsave (%a0)",
        "frestore (%a0)",
    ];
    for line in fpu {
        let bin = common::assemble_cpu(&format!("    {line}\n"), BASE, "5475");
        let opword = words(&bin)(BASE).unwrap();
        let form =
            isa::decode(opword).unwrap_or_else(|| panic!("{line}: {opword:#06x} does not decode"));
        assert_eq!(form.isa, Isa::Fpu, "{line}: {opword:#06x}");
    }
    let bin = common::assemble_cpu("top: fbeq top\n    fbne.l top\n", BASE, "5475");
    let mem = words(&bin);
    assert_eq!(isa::decode(mem(BASE).unwrap()).unwrap().isa, Isa::Fpu);
    assert_eq!(isa::length(&mem, BASE), Some(4));
    assert_eq!(isa::decode(mem(BASE + 4).unwrap()).unwrap().isa, Isa::Fpu);
    assert_eq!(isa::length(&mem, BASE + 4), Some(6));

    // The CAU's coprocessor load and store (MCF54418 RM ch. 26), longword only.
    let cau = [
        "cp0ldl %d1,%d2,#1,#0x13",
        "cp0ldl (%a1),%d0,#1,#0x01",
        "cp0stl %d1,%d2,#1,#2",
        "cp0ldl 4(%a1),%d2,#1,#3",
    ];
    for line in cau {
        let bin = common::assemble(&format!("    {line}\n"), BASE);
        let mem = words(&bin);
        let opword = mem(BASE).unwrap();
        let form =
            isa::decode(opword).unwrap_or_else(|| panic!("{line}: {opword:#06x} does not decode"));
        assert_eq!(form.isa, Isa::Cau, "{line}: {opword:#06x}");
        assert_eq!(isa::length(&mem, BASE), Some(bin.len() as u32), "{line}");
    }
}

/// An opword class where the description follows the CFPRM and `m68k-elf-objdump -m m68k:isa-c:emac` does not.
struct Known {
    what: &'static str,
    matches: fn(u16) -> bool,
    reason: &'static str,
    cfprm: &'static str,
}

/// The (mode, register) fields of bits 5-0.
fn ea(op: u16) -> (u16, u16) {
    ((op >> 3) & 7, op & 7)
}

/// Every disagreement between the description and objdump, by form. The manual wins each one; each entry
/// matches exactly the opwords that disagree.
const KNOWN_OBJDUMP_DIFFERENCES: &[Known] = &[
    Known {
        what: "ORI/ANDI/SUBI/ADDI/EORI/CMPI #imm,Dx with bits 5-3 not 000",
        matches: |op| {
            matches!(
                op & 0xffc0,
                0x0080 | 0x0280 | 0x0480 | 0x0680 | 0x0a80 | 0x0c00 | 0x0c40 | 0x0c80
            ) && ea(op).0 != 0
        },
        reason: "objdump takes Dx from bits 2-0 whatever bits 5-3 hold; the formats fix bits 5-3 at 000",
        cfprm: "CFPRM ch. 4 ORI p. 4-67, ANDI p. 4-10, SUBI p. 4-78, ADDI p. 4-5, EORI p. 4-36, CMPI p. 4-30",
    },
    Known {
        what: "BTST Dy,#imm",
        matches: |op| op & 0xf1ff == 0x013c,
        reason: "the dynamic BTST destination table has no #<data> entry",
        cfprm: "CFPRM ch. 4 BTST, p. 4-25",
    },
    Known {
        what: "MOVE.B #imm,Ax",
        matches: |op| op & 0xf1ff == 0x107c,
        reason: "MOVEA has no byte size: size 0x is reserved",
        cfprm: "CFPRM ch. 4 MOVEA, p. 4-49",
    },
    Known {
        what: "NEGX/NEG/NOT Dx with bits 5-3 not 000",
        matches: |op| matches!(op & 0xffc0, 0x4080 | 0x4480 | 0x4680) && ea(op).0 != 0,
        reason: "objdump takes Dx from bits 2-0 whatever bits 5-3 hold; the formats fix bits 5-3 at 000",
        cfprm: "CFPRM ch. 4 NEGX p. 4-62, NEG p. 4-61, NOT p. 4-64",
    },
    Known {
        what: "STRLDSR",
        matches: |op| op == 0x40e7,
        reason: "objdump leaves STRLDSR out of isa-c; the CFPRM lists it for ISA_C",
        cfprm: "CFPRM ch. 8 STRLDSR, p. 8-16; Table 3-16, p. 3-24",
    },
    Known {
        what: "MOVE <ea>,CCR and MOVE <ea>,SR with an EA other than Dy or #imm",
        matches: |op| matches!(op & 0xffc0, 0x44c0 | 0x46c0) && ea(op).0 != 0 && ea(op) != (7, 4),
        reason: "objdump takes Dy from bits 2-0 whatever bits 5-3 hold; only Dy and #<data> are listed",
        cfprm: "CFPRM ch. 4 MOVE to CCR, p. 4-54; ch. 8 MOVE to SR, p. 8-11",
    },
    Known {
        what: "TAS Dx",
        matches: |op| op & 0xfff8 == 0x4ac0,
        reason: "unlike the 68K, Dx is not a TAS addressing mode",
        cfprm: "CFPRM ch. 4 TAS, p. 4-82",
    },
    Known {
        what: "0x4AFD",
        matches: |op| op == 0x4afd,
        reason: "objdump prints it as its swbeg.l marker; the CFPRM defines only 0x4AFC (ILLEGAL) there, and mode 7 register 5 is no addressing mode",
        cfprm: "CFPRM ch. 4 ILLEGAL, p. 4-39; Table 2-3, p. 2-10",
    },
    Known {
        what: "Scc with bits 5-3 not 000 (conditions F, HI, LS, CC, CS, EQ, MI, GE, LT, GT, LE)",
        matches: |op| {
            op & 0xf0c0 == 0x50c0
                && ea(op).0 != 0
                && !(0x51fa..=0x51fc).contains(&op)
                && !matches!((op >> 8) & 0xf, 0x0 | 0x6 | 0x8 | 0x9 | 0xa)
        },
        reason: "objdump takes Dx from bits 2-0 whatever bits 5-3 hold (for T, NE, VC, VS and PL it rejects them); the format fixes bits 5-3 at 000",
        cfprm: "CFPRM ch. 4 Scc, p. 4-74",
    },
    Known {
        what: "BRA.L (0x60FF)",
        matches: |op| op == 0x60ff,
        reason: "objdump decodes it as BRA.B with displacement -1 for isa-c; the 8-bit field 0xFF selects a 32-bit displacement from ISA_B on (a guess in the description for ISA_C)",
        cfprm: "CFPRM ch. 4 BRA, p. 4-20",
    },
    Known {
        what: "MVS.B, MVZ.B and CMP.B with an Ay source",
        matches: |op| matches!(op & 0xf1f8, 0x7108 | 0x7188 | 0xb008),
        reason: "objdump accepts Ay as a byte operand; address registers cannot be byte-size operands (a guess in the description)",
        cfprm: "CFPRM 1.9.1, p. 1-20; ch. 4 MVS p. 4-59, MVZ p. 4-60, CMP p. 4-28",
    },
    Known {
        what: "MOVE <ea>,ACCext01 and MOVE <ea>,ACCext23 with an EA other than Dy or Ay",
        matches: |op| op & 0xfbc0 == 0xab00 && ea(op).0 >= 2,
        reason: "objdump reads bits 3-0 as Ry whatever bits 5-4 hold (so #imm loses its immediate); only Dy, Ay and #<data> are listed",
        cfprm: "CFPRM ch. 6 MOVE to ACCext01 p. 6-23, MOVE to ACCext23 p. 6-24",
    },
    Known {
        what: "CMPA.W",
        matches: |op| op & 0xf1c0 == 0xb0c0 && !(ea(op).0 == 7 && ea(op).1 > 4),
        reason: "objdump leaves CMPA.W out of isa-c; the CFPRM lists it from ISA_B and for ISA_C",
        cfprm: "CFPRM ch. 4 CMPA, p. 4-29; Table 3-16, p. 3-20",
    },
    Known {
        what: "WDEBUG",
        matches: |op| op & 0xffc0 == 0xfbc0 && matches!(ea(op).0, 2 | 5),
        reason: "objdump requires the extension word 0x0003 and this check supplies zero words; the opword alone is a WDEBUG",
        cfprm: "CFPRM ch. 8 WDEBUG, p. 8-18",
    },
    Known {
        what: "CAU cp0ld.l and cp0st.l",
        matches: |op| op & 0xfec0 == 0xfc80 && ea(op).0 <= 5,
        reason: "objdump decodes the coprocessor instructions in isa-c; this check counts the recognised CAU forms as not executable",
        cfprm: "RM 26.3.2; CFPRM does not list the coprocessor instructions",
    },
    Known {
        what: "Coprocessor forms other than the CAU's cp0ld.l/cp0st.l (byte and word sizes, the busy branch, coprocessor 1)",
        matches: |op| {
            op & 0xfc00 == 0xfc00
                && if (op >> 6) & 3 == 3 {
                    op & 0x013f < 8
                } else {
                    ea(op).0 <= 5
                }
                && !(op & 0xfec0 == 0xfc80)
        },
        reason: "objdump decodes generic coprocessor encodings; this core's only coprocessor is the CAU (coprocessor 0), longword only",
        cfprm: "RM 26.3.2; CFPRM does not list the coprocessor instructions",
    },
];

/// (valid, length in bytes) for each opword, as objdump sees it with five zero extension words after it.
fn objdump_view() -> Vec<(bool, u32)> {
    const STRIDE: usize = 12;
    let dir = std::env::temp_dir().join(format!("mcf5441x-emu-objdump-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let bin = dir.join("all.bin");
    let mut bytes = vec![0u8; 65536 * STRIDE];
    for op in 0..=0xffffu32 {
        bytes[op as usize * STRIDE..][..2].copy_from_slice(&(op as u16).to_be_bytes());
    }
    std::fs::write(&bin, &bytes).unwrap();
    let out = Command::new("m68k-elf-objdump")
        .args([
            "-D",
            "-z",
            "-b",
            "binary",
            "-m",
            "m68k:isa-c:emac",
            "--insn-width=12",
        ])
        .arg(&bin)
        .output()
        .expect("m68k-elf-objdump");
    let _ = std::fs::remove_dir_all(&dir);
    assert!(
        out.status.success(),
        "{}",
        String::from_utf8_lossy(&out.stderr)
    );
    let text = String::from_utf8(out.stdout).unwrap();
    // address -> valid?
    let mut lines = BTreeMap::new();
    for line in text.lines() {
        let Some((addr, rest)) = line.split_once(":\t") else {
            continue;
        };
        let Ok(addr) = u32::from_str_radix(addr.trim(), 16) else {
            continue;
        };
        let insn = rest.split_once('\t').map_or("", |(_, i)| i);
        lines.insert(addr, !insn.starts_with(".short"));
    }
    let addrs: Vec<u32> = lines.keys().copied().collect();
    (0..65536u32)
        .map(|op| {
            let at = op * STRIDE as u32;
            let i = addrs
                .binary_search(&at)
                .unwrap_or_else(|_| panic!("objdump lost sync at opword {op:#06x}"));
            let next = addrs.get(i + 1).copied().unwrap_or(65536 * STRIDE as u32);
            (lines[&at], next - at)
        })
        .collect()
}

#[test]
fn lengths_and_validity_match_objdump() {
    let theirs = objdump_view();
    let mut unexplained = Vec::new();
    let mut hits = vec![0usize; KNOWN_OBJDUMP_DIFFERENCES.len()];
    let mut agreeing_but_listed = Vec::new();
    for op in 0..=0xffffu16 {
        let mut words = [0u16; 6];
        words[0] = op;
        let mem = |addr: u32| words.get(addr as usize / 2).copied();
        // FPU and CAU forms are recognised but are not instructions of isa-c:emac (objdump rejects them there).
        let ours = match isa::decode(op) {
            Some(f) if !matches!(f.isa, Isa::Fpu | Isa::Cau) => {
                (true, isa::length(&mem, 0).unwrap())
            }
            _ => (false, 2),
        };
        let theirs = theirs[op as usize];
        let known: Vec<usize> = (0..KNOWN_OBJDUMP_DIFFERENCES.len())
            .filter(|&i| (KNOWN_OBJDUMP_DIFFERENCES[i].matches)(op))
            .collect();
        assert!(known.len() <= 1, "{op:#06x} is listed twice: {known:?}");
        match (ours == theirs, known.first()) {
            (true, None) => {}
            (true, Some(&i)) => agreeing_but_listed.push((op, KNOWN_OBJDUMP_DIFFERENCES[i].what)),
            (false, Some(&i)) => hits[i] += 1,
            (false, None) => unexplained.push(format!(
                "{op:#06x}: ours {ours:?} ({}), objdump {theirs:?}",
                isa::decode(op).map_or("-", |f| f.mnemonic)
            )),
        }
    }
    for (k, n) in KNOWN_OBJDUMP_DIFFERENCES.iter().zip(&hits) {
        assert!(*n > 0, "stale entry: {}", k.what);
        assert!(
            !k.reason.is_empty() && (k.cfprm.starts_with("CFPRM") || k.cfprm.starts_with("RM")),
            "{}",
            k.what
        );
    }
    assert!(
        agreeing_but_listed.is_empty(),
        "listed but agreeing: {:x?}",
        &agreeing_but_listed[..agreeing_but_listed.len().min(20)]
    );
    assert!(
        unexplained.is_empty(),
        "{} unexplained disagreements, first ones:\n{}",
        unexplained.len(),
        unexplained[..unexplained.len().min(60)].join("\n")
    );
}
