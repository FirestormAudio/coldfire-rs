//! An instruction that does not complete leaves the core as it was before it (RM 3.3.4.16): every form of the
//! instruction set is run with its operands where an access fails, and the whole programmer-visible state is
//! compared. This guards every handler's rule of writing registers only after its last access that can fault.

use mcf5441x_emu::isa::{self, Ext, Mode, Op};
use mcf5441x_emu::{Device, DeviceCx, Limit, Machine, Size, Stop};

/// Where the instruction is placed: near the end of RAM, so that `(d16,PC)` with a displacement of 0x7FF0 is
/// unmapped.
const AT: u32 = 0x40FF_FF00;
const RAM: u32 = 0x40FF_0000;
const RAM_END: u32 = 0x4100_0000;
/// A value for every 16-bit operand: as a displacement or absolute word it leads out of RAM.
const W: u16 = 0x7FF0;
/// A brief index extension word: D7.L * 1, d8 = 0.
const INDEX: u16 = 0x7800;
/// Where the address registers point in the unmapped pass.
const UNMAPPED: u32 = 0x3000_0000;

/// Reads 1 from anywhere in its window and refuses every write, as a model's tripwire would.
struct ReadOnly;

impl Device for ReadOnly {
    fn read(&mut self, _: u32, _: Size, _: &mut DeviceCx) -> u32 {
        1
    }
    fn write(&mut self, _: u32, _: Size, _: u32, cx: &mut DeviceCx) {
        cx.refuse(Stop::Unmodelled, "read-only");
    }
}

/// How the address registers are set up.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Pass {
    /// All of A0-A7 point at unmapped memory: the first access faults.
    Unmapped,
    /// All of A0-A7 point into a device that serves reads and refuses writes: a read-modify-write faults on its
    /// write, after its read.
    ReadOnly,
    /// All of A0-A7 point at the last longword of RAM: a multi-register transfer faults after its first access.
    Straddle,
    /// A0 points into RAM, A1-A7 at unmapped memory: an instruction with two memory operands faults on the
    /// second after the first succeeds.
    FirstOk,
}

/// The programmer-visible state, with the condition codes evaluated.
fn snapshot(m: &Machine) -> String {
    let c = &m.cpu;
    format!(
        "d {:08x?} a {:08x?} pc {:08x} sr {:04x} other_sp {:08x} vbr {:08x} cacr {:08x} asid {:08x} acr {:08x?} \
         mmubar {:08x} rgpiobar {:08x} rambar {:08x} emac {:x?}",
        c.d,
        c.a,
        c.pc,
        c.sr(),
        c.other_sp,
        c.vbr,
        c.cacr,
        c.asid,
        c.acr,
        c.mmubar,
        c.rgpiobar,
        c.rambar,
        c.emac
    )
}

/// The words of one instance of the form at `opword`: the opword, the form's extension words, then the EA
/// extension words (source first).
fn encode(opword: u16) -> Vec<u16> {
    let form = isa::decode(opword).unwrap();
    let mut words = vec![opword];
    let imm = |words: &mut Vec<u16>| match form.size {
        Some(Size::Long) => words.extend([W, W]),
        _ => words.push(W),
    };
    for e in form.ext.iter() {
        match e {
            Ext::Word => words.push(match form.id {
                // STRLDSR's immediate keeps the core in supervisor mode.
                Op::Strldsr => 0x2700,
                _ => W,
            }),
            Ext::Long => words.extend([W, W]),
            Ext::Ext => words.push(match form.id {
                // Every register: the transfer runs on after its first access.
                Op::MovemEaRegs | Op::MovemRegsEa => 0xFFFF,
                // MAC with load: Rx = D3, Ry = D2, word operands, no shift, MAC, the load through MASK.
                Op::MacLoad => 0x3022,
                // DIVx.L / REMx.L / MULx.L: Dx = D3, Dw = D4, signed long.
                Op::DivL | Op::MulL => 0x3804,
                _ => 0x0000,
            }),
            Ext::Imm => imm(&mut words),
        }
    }
    for ea in [form.fields.ea(opword), form.fields.ea2(opword)] {
        let Some(mode) = ea.and_then(|(m, r)| Mode::from_fields(m, r)) else {
            continue;
        };
        match mode {
            Mode::Disp | Mode::AbsW | Mode::PcDisp => words.push(W),
            Mode::AbsL => words.extend([W, W]),
            Mode::Index | Mode::PcIndex => words.push(INDEX),
            Mode::Imm => imm(&mut words),
            _ => {}
        }
    }
    words
}

/// Whether `mode` names memory.
fn memory(mode: Option<Mode>) -> bool {
    !matches!(mode, None | Some(Mode::Dn | Mode::An | Mode::Imm))
}

/// One machine with the instruction at [`AT`] and every register set to a distinct value.
fn machine(words: &[u16], pass: Pass) -> Machine {
    let mut m = Machine::new();
    m.mem.map_ram(RAM, RAM_END - RAM);
    if pass == Pass::ReadOnly {
        m.add_device(UNMAPPED, 0x1000_0000, Box::new(ReadOnly));
    }
    let bytes: Vec<u8> = words.iter().flat_map(|w| w.to_be_bytes()).collect();
    m.mem.load(AT, &bytes).unwrap();
    m.policy.stop_on = [true; 256];
    let c = &mut m.cpu;
    c.pc = AT;
    for k in 0..8 {
        // Distinct; as an index added to any base here, they lead out of RAM and inside the read-only device.
        c.d[k] = 0x0100_0000 + 0x104 * k as u32;
        c.a[k] = match pass {
            Pass::Unmapped | Pass::ReadOnly => UNMAPPED + 0x100 * k as u32,
            Pass::Straddle => RAM_END - 4,
            Pass::FirstOk if k == 0 => RAM + 0x100,
            Pass::FirstOk => UNMAPPED + 0x100 * k as u32,
        };
    }
    c.set_sr(0x2715);
    c.other_sp = 0x3000_0F00;
    c.vbr = 0x4100_0000;
    c.emac.acc = [0x1111_1111, 0x2222_2222, 0x3333_3333, 0x4444_4444];
    c.emac.ext = [0x0102, 0x0304, 0x0506, 0x0708];
    c.emac.macsr = 0x0005;
    c.emac.mask = 0xFFFF_5A5A;
    m
}

/// How one instance ended.
struct Outcome {
    kind: Stop,
    /// Whether it did not complete (and so must have left the core as it was).
    rolled_back: bool,
}

/// Run the one instruction cached or not; panics when an instruction that did not complete changed the core.
fn run(opword: u16, pass: Pass, cached: bool) -> Outcome {
    let words = encode(opword);
    let mut m = machine(&words, pass);
    let before = snapshot(&m);
    let limit = Limit {
        max_instructions: Some(1),
        ..Limit::default()
    };
    let r = if cached {
        m.run(limit)
    } else {
        m.run_uncached(limit)
    };
    let rolled_back = m.instructions() == 0;
    if rolled_back {
        let form = isa::decode(opword).unwrap();
        assert_eq!(
            snapshot(&m),
            before,
            "{} (line {}) {words:04x?} in the {pass:?} pass ({}): stopped with {r:?} but changed the core",
            form.mnemonic,
            form.line,
            if cached { "cached" } else { "uncached" }
        );
        assert_eq!(r.pc, AT, "{r:?}");
    }
    Outcome {
        kind: r.kind,
        rolled_back,
    }
}

/// Ops with a memory operand that is not required to fault on it. Every instance of them is still run in every
/// pass, and when it does not complete it must leave the core as it was:
/// - LEA and JMP compute an address without accessing it (JMP's target faults at the next fetch, after it
///   completed);
/// - the CAU's loads and stores (cp0ld, cp0st) stop with `Unmodelled` before any access;
/// - the FPU forms (FPU general, FSAVE, FRESTORE) take the line-F exception: the chip has no FPU.
///
/// PEA and JSR access no operand either, but their push faults (A7 is unmapped in that pass), so they are held to
/// the rule like the others.
const NO_ACCESS: &[Op] = &[
    Op::Lea,
    Op::Jmp,
    Op::Cp0ld,
    Op::Cp0st,
    Op::FpuOp,
    Op::Fsave,
    Op::Frestore,
];

#[test]
fn every_memory_operand_instruction_rolls_back_on_a_fault() {
    // One instance per (form, source mode, destination mode): the first opword and the last that decode to it, so
    // the register fields take their lowest and highest values.
    let mut found: std::collections::HashMap<(u32, Option<Mode>, Option<Mode>), (u16, u16)> =
        std::collections::HashMap::new();
    for opword in 0..=0xFFFFu16 {
        let Some(form) = isa::decode(opword) else {
            continue;
        };
        let mode = |ea: Option<(u32, u32)>| ea.and_then(|(m, r)| Mode::from_fields(m, r));
        let key = (
            form.line,
            mode(form.fields.ea(opword)),
            mode(form.fields.ea2(opword)),
        );
        found.entry(key).or_insert((opword, opword)).1 = opword;
    }
    let mut instances: Vec<((u32, Option<Mode>, Option<Mode>), u16)> = found
        .into_iter()
        .flat_map(|(k, (first, last))| [(k, first), (k, last)])
        .collect();
    instances.sort_by_key(|&(_, op)| op);
    instances.dedup_by_key(|&mut (_, op)| op);

    let mut unrolled: Vec<String> = Vec::new();
    for &((_, ea, ea2), opword) in &instances {
        let form = isa::decode(opword).unwrap();
        let mut faulted = false;
        for pass in [
            Pass::Unmapped,
            Pass::ReadOnly,
            Pass::Straddle,
            Pass::FirstOk,
        ] {
            for cached in [true, false] {
                let o = run(opword, pass, cached);
                if pass == Pass::Unmapped && o.rolled_back && o.kind == Stop::Unmapped {
                    faulted = true;
                }
            }
        }
        if (memory(ea) || memory(ea2)) && !NO_ACCESS.contains(&form.id) && !faulted {
            unrolled.push(format!(
                "{} (line {}) opword {opword:#06x}",
                form.mnemonic, form.line
            ));
        }
    }
    assert!(
        unrolled.is_empty(),
        "memory-operand instances that did not fault on an unmapped operand:\n{}",
        unrolled.join("\n")
    );

    // Every op with a memory operand was covered.
    let covered: Vec<Op> = instances
        .iter()
        .filter(|((_, ea, ea2), _)| memory(*ea) || memory(*ea2))
        .map(|(_, op)| isa::decode(*op).unwrap().id)
        .collect();
    for form in isa::forms() {
        if !form.ea.ea.is_empty() || !form.ea.ea2.is_empty() {
            let has_memory = [
                Mode::Ind,
                Mode::PostInc,
                Mode::PreDec,
                Mode::Disp,
                Mode::Index,
            ]
            .into_iter()
            .chain([Mode::AbsW, Mode::AbsL, Mode::PcDisp, Mode::PcIndex])
            .any(|m| form.ea.ea.contains(m) || form.ea.ea2.contains(m));
            if has_memory {
                assert!(covered.contains(&form.id), "{} not covered", form.mnemonic);
            }
        }
    }
}
