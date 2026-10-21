mod common;

use common::{STACK, until};
use mcf5441x_emu::elf::{ElfError, load_elf};
use mcf5441x_emu::{Machine, Stop};
use std::process::Command;
use std::sync::atomic::{AtomicUsize, Ordering};

static COUNTER: AtomicUsize = AtomicUsize::new(0);

/// Assemble `src` and link it as an ELF with `.text` at 0x40100000, `.data` at 0x40200000 and `.bss` right after it in the same segment.
fn link(src: &str) -> Vec<u8> {
    link_with(src, &["-n", "-Ttext=0x40100000", "-Tdata=0x40200000"])
}

/// As [`link`] with the given linker layout arguments.
fn link_with(src: &str, layout: &[&str]) -> Vec<u8> {
    let dir = std::env::temp_dir().join(format!(
        "mcf5441x-emu-elf-{}-{}",
        std::process::id(),
        COUNTER.fetch_add(1, Ordering::Relaxed)
    ));
    std::fs::create_dir_all(&dir).unwrap();
    let (s, o, e) = (dir.join("x.s"), dir.join("x.o"), dir.join("x.elf"));
    std::fs::write(&s, src).unwrap();
    let a = Command::new("m68k-elf-as")
        .args(["-mcpu=54418", "-o"])
        .arg(&o)
        .arg(&s)
        .output()
        .expect("m68k-elf-as");
    assert!(a.status.success(), "{}", String::from_utf8_lossy(&a.stderr));
    let l = Command::new("m68k-elf-ld")
        .args(layout)
        .args(["-e", "start", "-o"])
        .arg(&e)
        .arg(&o)
        .output()
        .expect("m68k-elf-ld");
    assert!(l.status.success(), "{}", String::from_utf8_lossy(&l.stderr));
    let bytes = std::fs::read(&e).unwrap();
    let _ = std::fs::remove_dir_all(&dir);
    bytes
}

const PROGRAM: &str = "
        .text
        .globl start
start:  move.l value,%d0
        lea buf,%a0
        move.l %d0,(%a0)
        add.l (%a0),%d0
done:   nop
        .data
value:  .long 0x12345678
        .bss
buf:    .space 16
";

fn machine() -> Machine {
    let mut m = Machine::new();
    m.mem.map_ram(0x4010_0000, 0x0100_0000);
    m
}

#[test]
fn loads_segments_and_returns_entry() {
    let elf = link(PROGRAM);
    let mut m = machine();
    // Dirty the bss first: loading must zero it.
    m.mem.load(0x4020_0004, &[0xff; 16]).unwrap();
    let entry = load_elf(&mut m, &elf).unwrap();
    assert_eq!(entry, 0x4010_0000);
    assert_eq!(
        m.mem.peek(0x4020_0000, 4).unwrap(),
        0x1234_5678u32.to_be_bytes()
    );
    assert_eq!(m.mem.peek(0x4020_0004, 16).unwrap(), vec![0; 16]);
    assert_eq!(m.mem.peek(0x4010_0000, 2).unwrap(), elf_text_head(&elf));
}

/// The first two bytes of the text in the file: the first PT_LOAD's data (offset in its program header).
fn elf_text_head(elf: &[u8]) -> Vec<u8> {
    let phoff = u32::from_be_bytes(elf[28..32].try_into().unwrap()) as usize;
    let off = u32::from_be_bytes(elf[phoff + 4..phoff + 8].try_into().unwrap()) as usize;
    elf[off..off + 2].to_vec()
}

#[test]
fn rejects_little_endian_and_wrong_machine() {
    let elf = link(PROGRAM);
    let mut m = machine();

    let mut le = elf.clone();
    le[5] = 1;
    assert_eq!(load_elf(&mut m, &le), Err(ElfError::NotBigEndian));

    let mut arm = elf.clone();
    arm[18..20].copy_from_slice(&40u16.to_be_bytes());
    assert_eq!(load_elf(&mut m, &arm), Err(ElfError::WrongMachine(40)));

    let mut elf64 = elf.clone();
    elf64[4] = 2;
    assert_eq!(load_elf(&mut m, &elf64), Err(ElfError::NotElf32));

    let mut bad_magic = elf.clone();
    bad_magic[1] = b'X';
    assert_eq!(load_elf(&mut m, &bad_magic), Err(ElfError::NotElf));
}

#[test]
fn malformed_input_is_an_error_not_a_panic() {
    let elf = link(PROGRAM);
    let mut m = machine();
    assert_eq!(load_elf(&mut m, &[]), Err(ElfError::Truncated));
    assert_eq!(load_elf(&mut m, &elf[..3]), Err(ElfError::Truncated));
    for n in 0..0x7c {
        // Every prefix that cuts into the headers or the segment data is a typed error.
        assert!(load_elf(&mut m, &elf[..n]).is_err(), "prefix {n}");
    }
    // A program header table pointing outside the file.
    let mut far = elf.clone();
    far[28..32].copy_from_slice(&0xffff_fff0u32.to_be_bytes());
    assert_eq!(load_elf(&mut m, &far), Err(ElfError::Truncated));
    // Garbage after a valid identification.
    let mut garbage = vec![0xa5u8; 200];
    garbage[..4].copy_from_slice(b"\x7fELF");
    garbage[4] = 1;
    garbage[5] = 2;
    assert!(load_elf(&mut m, &garbage).is_err());
    // A segment whose file range is outside the file.
    let mut big = elf.clone();
    let phoff = u32::from_be_bytes(big[28..32].try_into().unwrap()) as usize;
    big[phoff + 16..phoff + 24].copy_from_slice(&[0xff; 8]);
    assert_eq!(load_elf(&mut m, &big), Err(ElfError::Truncated));
}

#[test]
fn segment_in_unmapped_memory_is_an_error() {
    let elf = link(PROGRAM);
    let mut m = Machine::new();
    m.mem.map_ram(0x4010_0000, 0x1000); // text fits, data does not
    assert_eq!(
        load_elf(&mut m, &elf),
        Err(ElfError::Unmapped { addr: 0x4020_0000 })
    );
}

#[test]
fn runs_a_linked_program_to_its_exit_label() {
    let elf = link(PROGRAM);
    let mut m = machine();
    let entry = load_elf(&mut m, &elf).unwrap();
    m.cpu.pc = entry;
    m.cpu.a[7] = STACK;
    // `done` is the last instruction: move.l abs (6) + lea abs (6) + move.l (2) + add.l (2) = 16 bytes.
    let r = m.run(until(0x4010_0000 + 16));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.cpu.d[0], 0x1234_5678u32.wrapping_mul(2));
    assert_eq!(
        m.mem.peek(0x4020_0004, 4).unwrap(),
        0x1234_5678u32.to_be_bytes()
    );
}

#[test]
fn separate_bss_segment_is_zeroed_over_dirty_ram() {
    // Without -n and with -Tbss the linker gives .bss its own segment with no file data.
    let elf = link_with(
        PROGRAM,
        &["-Ttext=0x40100000", "-Tdata=0x40200000", "-Tbss=0x40210000"],
    );
    let mut m = Machine::new();
    // The first segment also covers the ELF headers, below the text.
    m.mem.map_ram(0x4000_0000, 0x0300_0000);
    m.mem.load(0x4021_0000, &[0xff; 16]).unwrap();
    load_elf(&mut m, &elf).unwrap();
    assert_eq!(m.mem.peek(0x4021_0000, 16).unwrap(), vec![0; 16]);
    assert_eq!(
        m.mem.peek(0x4020_0000, 4).unwrap(),
        0x1234_5678u32.to_be_bytes()
    );
}
