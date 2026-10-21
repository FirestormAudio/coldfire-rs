//! Booting from reset: the serial boot facility (RM ch. 11), FlexBus CS0 (RM ch. 20) and the NFC's boot (RM ch. 22),
//! and reading an ELF's sections and symbols. The tests that boot a real program build coldfire-rs's
//! `examples/cold` with `toolchain/cargo-cf`, one build per boot source; they are skipped (with a message) when the
//! ColdFire toolchain is not built.

use std::collections::{BTreeMap, BTreeSet};
use std::path::PathBuf;
use std::process::Command;
use std::sync::Mutex;

use mcf5441x_emu::boot::{BootError, BootSource, boot, nand_boot_view, nand_pages, sbf_image};
use mcf5441x_emu::elf::{sections, symbols};
use mcf5441x_emu::parts::SpiNor;
use mcf5441x_emu::soc::{Chip, ChipOptions, Ddrmc, Dspi, Mcf5441x, NandArray, Pll, StubBlock};
use mcf5441x_emu::{Limit, Machine, Size, Stop, StopReason};

/// DDR as examples/cold's memory.x has it: 128 MB at 0x40000000, its stack at the top.
const DDR: (u32, u32) = (0x4000_0000, 0x0800_0000);
/// The SRAM: 64 KB where the program's RAMBAR puts it (RM ch. 7).
const SRAM: (u32, u32) = (0x8000_0000, 0x1_0000);
/// A 16 MB SPI NOR with a 64 KB-sector ID.
const JEDEC: [u8; 6] = [0x01, 0x20, 0x18, 0x4D, 0x01, 0x80];
const FLASH_SIZE: u32 = 16 << 20;
/// examples/cold's PLL (cold-hello's `PLL`): FBKDIV 19 and REFDIV 0 in PLL_CR; OUTDIV1 1, OUTDIV2 3 and OUTDIV5 7
/// in PLL_DR (RM Figures 8-3 and 8-4).
const PLL_CR: u32 = 19;
const PLL_DR: u32 = 1 | 3 << 5 | 7 << 21;
/// The boot part's halt markers, in D0 at its HALT (embassy-mcf5441x's boot ops).
const MARK_PLL_MISMATCH: u32 = 0xB007_0001;
const MARK_NAND_ECC: u32 = 0xB007_0005;
/// A NAND boot-format page's payload (embassy-mcf5441x's boot fetch: 1056-byte pages, 60 ECC bytes).
const PAYLOAD: usize = 996;
/// The cold layout's sections (coldfire-rt's link-cold.x): the boot part, then the app part.
const BOOT_SECTIONS: [&str; 4] = [".boot.vectors", ".boot.head", ".boot", ".boot.rodata"];
const APP_SECTIONS: [&str; 3] = [".text", ".rodata", ".data"];
const LIMIT: u64 = 50_000_000;

fn root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..")
}

static BUILD: Mutex<()> = Mutex::new(());

/// examples/cold built for boot source `source` with `extra` features, as ELF bytes; `None` (with a message) when the
/// ColdFire toolchain is not built.
fn build(source: &str, extra: &[&str]) -> Option<Vec<u8>> {
    let root = root();
    let rcf =
        std::env::var_os("RCF").map_or_else(|| root.join("build/rust-coldfire"), PathBuf::from);
    if !rcf
        .join("rustc_codegen_gcc/target/debug/librustc_codegen_gcc.so")
        .exists()
    {
        eprintln!("skipped: the ColdFire toolchain is not built (toolchain/setup.sh, or set RCF)");
        return None;
    }
    let _one = BUILD.lock().unwrap_or_else(|e| e.into_inner());
    let cold = root.join("examples/cold");
    let name = std::iter::once(source)
        .chain(extra.iter().copied())
        .collect::<Vec<_>>()
        .join("-");
    let target = cold.join("target").join(&name);
    let features = std::iter::once(format!("boot-{source}"))
        .chain(std::iter::once("mcf54415".to_string()))
        .chain(extra.iter().map(|s| s.to_string()))
        .collect::<Vec<_>>()
        .join(",");
    let mut cmd = Command::new(root.join("toolchain/cargo-cf"));
    // the outer cargo's settings are not the ColdFire build's
    for (k, _) in std::env::vars_os() {
        let k = k.to_string_lossy().into_owned();
        if (k.starts_with("CARGO_") && k != "CARGO_HOME")
            || k.starts_with("RUSTC")
            || k.starts_with("RUSTFLAGS")
            || k == "RUSTDOCFLAGS"
            || k == "RUSTUP_TOOLCHAIN"
        {
            cmd.env_remove(k);
        }
    }
    let out = cmd
        .arg(&cold)
        .args([
            "--no-default-features",
            "--features",
            &features,
            "--target-dir",
        ])
        .arg(&target)
        .env("RCF", &rcf)
        .env("CF_MEMORY_DIR", cold.join("memory").join(source))
        .output()
        .expect("toolchain/cargo-cf runs");
    let err = String::from_utf8_lossy(&out.stderr);
    assert!(
        out.status.success(),
        "cargo-cf {name} failed:\n{}",
        &err[err.len().saturating_sub(4000)..]
    );
    Some(
        std::fs::read(target.join("m68k-coldfire-none-elf/release/cold-hello"))
            .expect("cold-hello built"),
    )
}

/// The named sections laid out by load address from the lowest one's, gaps zero (as `objcopy -O binary` does).
fn flat(elf: &[u8], names: &[&str]) -> (u32, Vec<u8>) {
    let all = sections(elf).unwrap();
    let parts: Vec<_> = all
        .iter()
        .filter(|s| names.contains(&s.name.as_str()) && !s.data.is_empty())
        .collect();
    let base = parts
        .iter()
        .map(|s| s.load)
        .min()
        .expect("sections present");
    let end = parts
        .iter()
        .map(|s| s.load + s.data.len() as u32)
        .max()
        .unwrap();
    let mut out = vec![0; (end - base) as usize];
    for s in parts {
        let at = (s.load - base) as usize;
        out[at..at + s.data.len()].copy_from_slice(&s.data);
    }
    (base, out)
}

/// A machine with the chip, DDR and SRAM.
fn machine(options: ChipOptions) -> (Machine, Chip) {
    let mut m = Machine::new();
    m.mem.map_ram(DDR.0, DDR.1);
    m.mem.map_ram(SRAM.0, SRAM.1);
    let chip = Mcf5441x::build(&mut m, options);
    (m, chip)
}

fn run_to(m: &mut Machine, pc: u32) -> StopReason {
    m.run(Limit {
        until_pc: Some(pc),
        max_instructions: Some(LIMIT),
    })
}

/// Serial boot of `elf` with PLL_CR `pll_cr`: the SBF loads the boot part (`__boot_size` bytes) into the SRAM.
fn serial(elf: &[u8], pll_cr: u32) -> (Machine, Chip, BTreeMap<String, u32>) {
    let sym = symbols(elf).unwrap();
    let (base, image) = flat(elf, &[&BOOT_SECTIONS[..], &APP_SECTIONS[..]].concat());
    assert_eq!(base, sym["__image_start"]);
    let flash = SpiNor::new(
        FLASH_SIZE,
        JEDEC,
        &sbf_image(&image, sym["__boot_size"] as usize, 0),
    );
    let (mut m, chip) = machine(ChipOptions::default().with_pll(pll_cr, PLL_DR));
    let pc = boot(
        &mut m,
        &chip,
        BootSource::Serial {
            flash,
            sram_base: SRAM.0,
        },
    )
    .unwrap();
    assert_eq!(
        (m.cpu.a[7], pc),
        (sym["__boot_stack_top"], sym["__boot_entry"])
    );
    (m, chip, sym)
}

/// The NAND for `elf`: the boot part in the four boot pages, the app part from row 4.
fn nand_array(elf: &[u8], bad: &[u32]) -> NandArray {
    let (_, boot_part) = flat(elf, &BOOT_SECTIONS);
    let (_, app) = flat(elf, &APP_SECTIONS);
    let mut pages = nand_pages(&boot_part, PAYLOAD);
    pages.extend(app.chunks(PAYLOAD).map(<[u8]>::to_vec));
    NandArray {
        pages,
        bad: bad.iter().copied().collect(),
        payload: PAYLOAD,
    }
}

fn nand(elf: &[u8], bad: &[u32]) -> (Machine, Chip, BTreeMap<String, u32>) {
    let sym = symbols(elf).unwrap();
    let (mut m, chip) = machine(ChipOptions::default().with_nand(NandArray::default()));
    let pc = boot(
        &mut m,
        &chip,
        BootSource::Nand {
            array: nand_array(elf, bad),
        },
    )
    .unwrap();
    assert_eq!(
        (m.cpu.a[7], pc),
        (sym["__boot_stack_top"], sym["__boot_entry"])
    );
    (m, chip, sym)
}

fn nor_of(chip: &Chip, m: &Machine) -> Vec<u8> {
    let nor = m.device::<Dspi>(chip.dspi[0]).device::<SpiNor>(1);
    nor.log()
        .iter()
        .map(|&(_, op, _)| op)
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect()
}

fn stub_writes(m: &Machine, chip: &Chip, name: &str, offset: u32) -> Vec<u32> {
    let id = chip.stubs.iter().find(|(n, _)| *n == name).unwrap().1;
    m.device::<StubBlock>(id)
        .writes()
        .iter()
        .filter(|w| w.1 == offset)
        .map(|w| w.3)
        .collect()
}

#[test]
fn serial_boot_reaches_main() {
    let Some(elf) = build("serial", &[]) else {
        return;
    };
    let (mut m, chip, sym) = serial(&elf, PLL_CR);
    let r = run_to(&mut m, sym["main"]);
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    // the boot part only READs (0x03) the flash
    assert_eq!(nor_of(&chip, &m), [0x03]);
    // SBFSR holds BLL (RM 11.3.1)
    let bll = sym["__boot_size"].div_ceil(4) - 1;
    assert_eq!(m.access(0xEC09_0020, Size::Word, None), Ok(bll));
}

#[test]
fn nor_boot_reaches_main() {
    let Some(elf) = build("nor", &[]) else { return };
    let sym = symbols(&elf).unwrap();
    let (base, image) = flat(&elf, &[&BOOT_SECTIONS[..], &APP_SECTIONS[..]].concat());
    assert_eq!(base, 0);
    let (mut m, chip) = machine(ChipOptions::default());
    boot(&mut m, &chip, BootSource::Nor { image }).unwrap();
    let r = run_to(&mut m, sym["main"]);
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    // the program's CS0 (cold-hello's FlexBusCs0): CSCR0 (RM 20.3.3) WS 4 (bits 15:10), AA (bit 8), PS 1x 16-bit
    // (bits 7:6); CSMR0 (RM 20.3.2) BAM 0x00FF (bits 31:16, a 16 MB block) and V (bit 0)
    let cscr0 = 4 << 10 | 1 << 8 | 0b10 << 6;
    let csmr0 = ((16u32 << 20) - 1) & 0xFFFF_0000 | 1;
    assert_eq!(stub_writes(&m, &chip, "FlexBus", 0x08), [cscr0]);
    assert_eq!(stub_writes(&m, &chip, "FlexBus", 0x04), [csmr0]);
    // CSAR0 = 0 first, and CSMR0[V] last: CS0 stays the global chip select until then (RM 20.4.1.3)
    let id = chip.stubs.iter().find(|(n, _)| *n == "FlexBus").unwrap().1;
    let order: Vec<u32> = m
        .device::<StubBlock>(id)
        .writes()
        .iter()
        .map(|w| w.1)
        .collect();
    assert_eq!(order, [0x00, 0x08, 0x04]);
}

#[test]
fn nand_boot_reaches_main() {
    let Some(elf) = build("nand", &[]) else {
        return;
    };
    let (mut m, chip, sym) = nand(&elf, &[]);
    let r = run_to(&mut m, sym["main"]);
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    // the boot part read the app pages from row 4 on, in order
    let (_, app) = flat(&elf, &APP_SECTIONS);
    let rows: Vec<u32> = (4..4 + app.len().div_ceil(PAYLOAD) as u32).collect();
    assert_eq!(
        m.device::<mcf5441x_emu::soc::Nfc>(chip.nfc.unwrap())
            .reads(),
        rows
    );
}

#[test]
fn app_part_arrives_exactly() {
    // cold-hello's app part ends mid-page: the last NAND page is copied to the byte
    let Some(elf) = build("nand", &[]) else {
        return;
    };
    let (_, app) = flat(&elf, &APP_SECTIONS);
    assert_ne!(app.len() % PAYLOAD, 0);
    let (mut m, _chip, sym) = nand(&elf, &[]);
    assert_eq!(sym["__app_len"] as usize, app.len());
    let r = run_to(&mut m, sym["main"]);
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.mem.peek(sym["__app_vma"], app.len() as u32).unwrap(), app);
}

#[test]
fn nand_bad_page_halts_with_its_marker() {
    // the second app page (row 5) fails ECC: the boot part halts with its NAND ECC marker in D0
    let Some(elf) = build("nand", &[]) else {
        return;
    };
    let (_, app) = flat(&elf, &APP_SECTIONS);
    assert!(app.len() > PAYLOAD, "cold-hello's app part spans two pages");
    let (mut m, _chip, sym) = nand(&elf, &[5]);
    let r = run_to(&mut m, sym["main"]);
    assert_eq!(r.kind, Stop::Halt("halt"), "{r:?}");
    assert_eq!(m.cpu.d[0], MARK_NAND_ECC);
}

#[test]
fn ddr_sequence_order() {
    // RM 21.4: DDR_RCR 0, DDR_RCR[RST], DDR_PADCR, (the board's CRs: none in cold-hello), CR09 START last
    let Some(elf) = build("serial", &[]) else {
        return;
    };
    let (mut m, chip, sym) = serial(&elf, PLL_CR);
    assert_eq!(run_to(&mut m, sym["main"]).kind, Stop::UntilPc);
    let writes = m.device::<Ddrmc>(chip.ddrmc).writes();
    let offsets: Vec<u32> = writes.iter().map(|w| w.0).collect();
    assert_eq!(offsets[..3], [0x180, 0x180, 0x1AC]);
    assert_eq!((writes[0].2, writes[1].2), (0, 1 << 30));
    let last = writes.last().unwrap();
    assert_eq!(last.0, 0x24);
    assert_ne!(last.2 & 1 << 24, 0, "CR09 START");
}

#[test]
fn keep_mismatch_halts_before_ddr() {
    // pll-keep: the boot part checks the PLL the SBF set; a PLL_CR other than the program's halts before DDR
    let Some(elf) = build("serial", &["pll-keep"]) else {
        return;
    };
    let (mut m, chip, sym) = serial(&elf, PLL_CR + 1);
    let r = run_to(&mut m, sym["main"]);
    assert_eq!(r.kind, Stop::Halt("halt"), "{r:?}");
    assert_eq!(m.cpu.d[0], MARK_PLL_MISMATCH);
    assert!(m.device::<Ddrmc>(chip.ddrmc).writes().is_empty());
    assert!(m.device::<Pll>(chip.pll).writes().is_empty());
}

#[test]
fn keep_match_reaches_main() {
    let Some(elf) = build("serial", &["pll-keep"]) else {
        return;
    };
    let (mut m, chip, sym) = serial(&elf, PLL_CR);
    let r = run_to(&mut m, sym["main"]);
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert!(m.device::<Pll>(chip.pll).writes().is_empty());
}

#[test]
fn sections_carry_run_and_load_addresses() {
    // the app part runs in DDR and loads after the boot part (coldfire-rt's link-cold.x)
    let Some(elf) = build("serial", &[]) else {
        return;
    };
    let sym = symbols(&elf).unwrap();
    let text = sections(&elf)
        .unwrap()
        .into_iter()
        .find(|s| s.name == ".text")
        .unwrap();
    assert_eq!((text.addr, text.load), (sym["__app_vma"], sym["__app_lma"]));
    assert_eq!(sym["__app_lma"], sym["__image_start"] + sym["__boot_size"]);
    let all = sections(&elf).unwrap();
    assert!(
        all.iter()
            .all(|s| !s.name.starts_with(".debug") && s.name != ".bss")
    );
}

#[test]
fn boot_view_round_trips() {
    let data: Vec<u8> = (0..3968).map(|i| (i % 256) as u8).collect();
    let pages = nand_pages(&data, PAYLOAD);
    assert_eq!(pages.iter().map(Vec::len).collect::<Vec<_>>(), [PAYLOAD; 4]);
    // RM Figure 22-25: U0 page 0, U1 page 1, ..., U4 page 0's second unit
    assert_eq!(pages[1][..8], data[8..16]);
    assert_eq!(pages[0][8..16], data[32..40]);
    // guess: the view keeps to whole 8-byte units; each page's last 4 bytes are outside it
    assert_eq!(pages[3][992..], [0xFF; 4]);
    assert_eq!(nand_boot_view(&pages), data);
}

#[test]
fn sbf_image_follows_table_11_5() {
    // RM Table 11-5: {0000, BLDIV}, BLL[7:0], BLL[15:8], RCON[7:0]..RCON[31:24], then the code from byte 7
    let image: Vec<u8> = (1..=40).collect();
    let flash = sbf_image(&image, 18, 0x1234_5678);
    // 18 bytes: 5 longwords, BLL 4 (BLL + 1 longwords are loaded, RM Table 11-3)
    assert_eq!(flash[..7], [0x01, 4, 0, 0x78, 0x56, 0x34, 0x12]);
    assert_eq!(flash[7..], image[..]);
    assert_eq!(sbf_image(&image, 0, 0)[1..3], [0, 0]);
}

/// A serial-boot flash with a boot image of `longs` longwords whose first two are SP and PC.
fn handmade(longs: usize, skip: &[u8]) -> (Vec<u8>, Vec<u8>) {
    let mut image = Vec::new();
    image.extend_from_slice(&0x8001_0000u32.to_be_bytes());
    image.extend_from_slice(&0x8000_0008u32.to_be_bytes());
    image.extend((8..longs * 4 + 16).map(|i| i as u8));
    let mut flash = skip.to_vec();
    flash.extend(sbf_image(&image, longs * 4, 0));
    (image, flash)
}

#[test]
fn serial_boot_loads_bll_plus_one_longwords() {
    let (image, flash) = handmade(6, &[]);
    let (mut m, chip) = machine(ChipOptions::default());
    let pc = boot(
        &mut m,
        &chip,
        BootSource::Serial {
            flash: SpiNor::new(1 << 20, JEDEC, &flash),
            sram_base: SRAM.0,
        },
    );
    assert_eq!(pc, Ok(0x8000_0008));
    assert_eq!(
        (m.cpu.a[7], m.cpu.pc, m.cpu.sr()),
        (0x8001_0000, 0x8000_0008, 0x2700)
    );
    // 6 longwords copied, the SRAM after them untouched
    assert_eq!(m.mem.peek(SRAM.0, 24).unwrap(), image[..24]);
    assert_eq!(m.mem.peek(SRAM.0 + 24, 4).unwrap(), [0; 4]);
    // SBFSR BLL 5, SBFCR BLDIV 1 and FR clear (RM 11.3)
    assert_eq!(m.access(0xEC09_0020, Size::Word, None), Ok(5));
    assert_eq!(m.access(0xEC09_0022, Size::Word, None), Ok(1));
    // the flash is on DSPI0 PCS1
    assert_eq!(
        m.device::<Dspi>(chip.dspi[0]).device::<SpiNor>(1).memory()[..flash.len()],
        flash[..]
    );
}

#[test]
fn serial_boot_skips_bytes_until_bits_7_4_clear() {
    // RM 11.4.1 step 5: a byte with bits 7:4 not 0000 is skipped. guess: the 1-to-0 transition the SBF also needs
    // before the next byte is taken as given.
    let (image, flash) = handmade(4, &[0xFF, 0x80]);
    let (mut m, chip) = machine(ChipOptions::default());
    let pc = boot(
        &mut m,
        &chip,
        BootSource::Serial {
            flash: SpiNor::new(1 << 20, JEDEC, &flash),
            sram_base: SRAM.0,
        },
    );
    assert_eq!(pc, Ok(0x8000_0008));
    assert_eq!(m.mem.peek(SRAM.0, 16).unwrap(), image[..16]);
}

#[test]
fn serial_boot_refusals() {
    let (mut m, chip) = machine(ChipOptions::default());
    // BLL 0: nothing is loaded (RM 11.4.3), the reset vector would come from FlexBus or the NFC
    let flash = SpiNor::new(1 << 20, JEDEC, &sbf_image(&[0; 16], 0, 0));
    assert_eq!(
        boot(
            &mut m,
            &chip,
            BootSource::Serial {
                flash,
                sram_base: SRAM.0
            }
        ),
        Err(BootError::NoBootLoad)
    );
    // an erased flash never synchronises the SBF (RM 11.4.1)
    let flash = SpiNor::new(1 << 20, JEDEC, &[]);
    assert_eq!(
        boot(
            &mut m,
            &chip,
            BootSource::Serial {
                flash,
                sram_base: SRAM.0
            }
        ),
        Err(BootError::NoSbfHeader)
    );
    // no SRAM at the base
    let (_, flash) = handmade(4, &[]);
    let flash = SpiNor::new(1 << 20, JEDEC, &flash);
    assert_eq!(
        boot(
            &mut m,
            &chip,
            BootSource::Serial {
                flash,
                sram_base: 0x9000_0000
            }
        ),
        Err(BootError::Unmapped { addr: 0x9000_0000 })
    );
}

#[test]
fn nor_boot_maps_the_image_at_0() {
    let mut image = Vec::new();
    image.extend_from_slice(&0x8001_0000u32.to_be_bytes());
    image.extend_from_slice(&0x0000_0400u32.to_be_bytes());
    image.extend_from_slice(&[0x4E, 0x71]);
    let (mut m, chip) = machine(ChipOptions::default());
    assert_eq!(
        boot(
            &mut m,
            &chip,
            BootSource::Nor {
                image: image.clone()
            }
        ),
        Ok(0x400)
    );
    assert_eq!(m.cpu.a[7], 0x8001_0000);
    assert_eq!(m.mem.peek(0, image.len() as u32).unwrap(), image);
    // erased past the image, to 64 KB
    assert_eq!(m.mem.peek(0xFFFC, 4).unwrap(), [0xFF; 4]);
    assert!(m.mem.peek(0x1_0000, 1).is_none());
}

/// Four boot pages whose view starts with SP 0x80010000 and PC `pc`.
fn boot_pages(pc: u32) -> Vec<Vec<u8>> {
    let mut view = 0x8001_0000u32.to_be_bytes().to_vec();
    view.extend_from_slice(&pc.to_be_bytes());
    nand_pages(&view, PAYLOAD)
}

#[test]
fn nand_boot_falls_back_to_the_next_boot_block() {
    // RM 22.4.4: an ECC failure in the first boot block's pages moves the boot to the block at row 256
    let mut pages = vec![vec![0xFF; PAYLOAD]; 260];
    pages[..4].clone_from_slice(&boot_pages(0x10));
    pages[256..260].clone_from_slice(&boot_pages(0x20));
    let array = NandArray {
        pages,
        bad: BTreeSet::from([2]),
        payload: PAYLOAD,
    };
    let (mut m, chip) = machine(ChipOptions::default().with_nand(NandArray::default()));
    assert_eq!(boot(&mut m, &chip, BootSource::Nand { array }), Ok(0x20));
    // NFC_CFG[BTMD] is set after an NFC boot (RM Table 22-17)
    assert_ne!(m.access(0xFC0F_FF30, Size::Long, None).unwrap() & 1 << 6, 0);
    // the view is RAM at 0..0x1000, erased past 3968 bytes
    assert_eq!(m.mem.peek(0xF80, 0x80).unwrap(), [0xFF; 0x80]);
}

#[test]
fn nand_boot_refusals() {
    let pages = boot_pages(0x10);
    // every boot block has a failing page: the NFC boot is unsuccessful (RM 22.4.4)
    let array = NandArray {
        pages: pages.clone(),
        bad: BTreeSet::from([0, 257, 514, 771]),
        payload: PAYLOAD,
    };
    let (mut m, chip) = machine(ChipOptions::default().with_nand(NandArray::default()));
    assert_eq!(
        boot(&mut m, &chip, BootSource::Nand { array }),
        Err(BootError::NandBootFailed)
    );
    // a chip without a NAND array has no NFC model
    let array = NandArray {
        pages,
        bad: BTreeSet::new(),
        payload: PAYLOAD,
    };
    let (mut m, chip) = machine(ChipOptions::default());
    assert_eq!(
        boot(&mut m, &chip, BootSource::Nand { array }),
        Err(BootError::NoNfc)
    );
}
