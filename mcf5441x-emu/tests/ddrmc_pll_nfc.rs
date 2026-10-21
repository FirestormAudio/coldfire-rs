//! The DDR controller (RM ch. 21), the PLL (RM ch. 8) and the NAND flash controller (RM ch. 22), driven through
//! their registers the way boot code drives them: the DDR controller's parameters and START, the PLL's dividers,
//! and NFC page reads into the buffer.

use std::collections::BTreeSet;

use mcf5441x_emu::soc::{Ddrmc, NandArray, Nfc, Pll};
use mcf5441x_emu::{Device, DeviceId, Fault, Limit, Machine, Size, Stop};

const DDR: u32 = 0xFC0B_8000;
const PLL: u32 = 0xFC0C_0000;
const NFC: u32 = 0xFC0F_C000;
/// The NFC registers, after the buffer (RM Table 22-3).
const NFC_REGS: u32 = NFC + 0x3F00;

/// DDR_CR09 START, DDR_CR25 INTACK, DDR_CR27 INTSTATUS (RM Tables 21-14, 21-30, 21-32).
const CR09: u32 = 0x24;
const START: u32 = 1 << 24;
const CR25: u32 = 0x64;
const CR27: u32 = 0x6C;
const CR44: u32 = 0xB0;
const RCR: u32 = 0x180;
const PADCR: u32 = 0x1AC;
/// INTSTATUS bit 3, DRAM initialization complete, and bit 8, the OR of bits 7:0.
const INIT_DONE: u32 = 1 << 3;
const INT_ANY: u32 = 1 << 8;

/// PLL_CR, PLL_DR, PLL_SR (RM Table 8-2) and the status bits (RM Table 8-5).
const PLL_CR: u32 = 0x0;
const PLL_DR: u32 = 0x4;
const PLL_SR: u32 = 0x8;
const LOCK: u32 = 1 << 4;
const LOCKS: u32 = 1 << 5;
/// Test values: REFDIV 0, FBKDIV 26; OUTDIV5 3, OUTDIV4 2, OUTDIV3 3, OUTDIV2 2, OUTDIV1 1.
const TEST_CR: u32 = 0x0000_001A;
const TEST_DR: u32 = 0x0062_0C41;

/// The NFC registers (RM Table 22-3).
const CMD1: u32 = 0x00;
const CMD2: u32 = 0x04;
const CAR: u32 = 0x08;
const RAR: u32 = 0x0C;
const RPT: u32 = 0x10;
const RAI: u32 = 0x14;
const SR1: u32 = 0x18;
const SR2: u32 = 0x1C;
const DMA1: u32 = 0x20;
const DMACFG: u32 = 0x24;
const SWAP: u32 = 0x28;
const SECSZ: u32 = 0x2C;
const CFG: u32 = 0x30;
const DMA2: u32 = 0x34;
const ISR: u32 = 0x38;
/// NFC_ISR (RM Table 22-18).
const DONE: u32 = 1 << 30;
const IDLE: u32 = 1 << 29;
const DONEEN: u32 = 1 << 21;
const DONECLR: u32 = 1 << 18;
const IDLECLR: u32 = 1 << 17;
/// A page read (RM Figure 22-18): BYTE1 0x00, CODE 0x7EE0, BUFNO 0, START.
const READ_PAGE: u32 = 0x007E_E001;
/// NFC_CFG (RM Table 22-17): ECCAD at bits 30:22, ECCSRAM, DMAREQ, ECCMODE 32-error, AIAD, PAGECNT 1.
const ECCSRAM: u32 = 1 << 21;
const DMAREQ: u32 = 1 << 20;
const AIAD: u32 = 1 << 5;
const AIBN: u32 = 1 << 4;
const CFG_READ: u32 = 0x000E_0001 | ECCSRAM;
/// The ECC status byte goes to buffer byte 0x3F8 (ECCAD 0x7F).
const ECCAD: u32 = 0x7F;
const PAYLOAD: usize = 0x3E0;

struct Rig {
    m: Machine,
    ddr: DeviceId,
    pll: DeviceId,
    nfc: DeviceId,
}

fn array() -> NandArray {
    let pages = (0..6u8)
        .map(|p| (0..PAYLOAD - p as usize * 3).map(|i| p ^ i as u8).collect())
        .collect();
    NandArray {
        pages,
        bad: BTreeSet::from([4]),
        payload: PAYLOAD,
    }
}

/// The three blocks at their chip addresses.
fn rig() -> Rig {
    let mut m = Machine::new();
    let ddr = m.add_device(DDR, 0x4000, Box::new(Ddrmc::new()));
    let pll = m.add_device(PLL, 0x4000, Box::new(Pll::new(TEST_CR, TEST_DR)));
    let nfc = m.add_device(NFC, 0x4000, Box::new(Nfc::new(array())));
    Rig { m, ddr, pll, nfc }
}

impl Rig {
    fn read(&mut self, addr: u32, size: Size) -> u32 {
        let m = &mut self.m;
        m.access(addr, size, None)
            .unwrap_or_else(|f| panic!("read {addr:#x}: {f:?} {:?}", m.take_refusal()))
    }

    fn write(&mut self, addr: u32, size: Size, value: u32) {
        let m = &mut self.m;
        m.access(addr, size, Some(value))
            .unwrap_or_else(|f| panic!("write {addr:#x}: {f:?} {:?}", m.take_refusal()));
    }

    /// An access the block refuses: returns the detail.
    fn refused(&mut self, addr: u32, size: Size, write: Option<u32>) -> String {
        assert_eq!(
            self.m.access(addr, size, write),
            Err(Fault::Device(Stop::Unmodelled)),
            "{addr:#x} {size:?} {write:?}"
        );
        self.m.take_refusal().expect("a refusal").detail
    }

    fn nfc_r(&mut self, off: u32) -> u32 {
        self.read(NFC_REGS + off, Size::Long)
    }

    fn nfc_w(&mut self, off: u32, value: u32) {
        self.write(NFC_REGS + off, Size::Long, value)
    }

    fn buffer(&mut self, off: u32, len: u32) -> Vec<u8> {
        (0..len)
            .map(|i| self.read(NFC + off + i, Size::Byte) as u8)
            .collect()
    }

    /// Read `row` with the ECC status at ECCAD; returns ISR.
    fn read_row(&mut self, row: u32) -> u32 {
        self.nfc_w(CFG, ECCAD << 22 | CFG_READ);
        self.nfc_w(RAR, 0x1100_0000 | row);
        self.nfc_w(CMD2, READ_PAGE);
        self.nfc_r(ISR)
    }
}

#[test]
fn ddrmc_reset_values_follow_the_measurement() {
    // Measured on an MCF54415 with the controller's clock on: every CR is 0 except these. The RM (Table 21-4)
    // agrees on CR13, CR20, CR21, CR23 and CR44, and lists 0 for CR03 and CR22.
    let mut r = rig();
    let nonzero = [
        (0x0C, 0x100),
        (0x34, 0x0002_0000),
        (0x50, 0x0C00_0000),
        (0x54, 0x400),
        (0x58, 0xD00),
        (0x5C, 0x1000_0000),
        (CR44, 0x2040),
    ];
    for cr in 0..64 {
        let off = cr * 4;
        let want = nonzero
            .iter()
            .find(|&&(o, _)| o == off)
            .map_or(0, |&(_, v)| v);
        assert_eq!(r.read(DDR + off, Size::Long), want, "CR{cr:02}");
    }
    assert_eq!(r.read(DDR + RCR, Size::Long), 0, "RCR (RM Table 21-4)");
    // guess: RM Figure 21-56 has PADCR's reserved bit 9 reset to 1 where Table 21-4 lists 0.
    assert_eq!(r.read(DDR + PADCR, Size::Long), 0x200, "PADCR");
    assert!(r.m.device::<Ddrmc>(r.ddr).writes().is_empty());
}

#[test]
fn ddr_init_done_follows_start() {
    let mut r = rig();
    assert_eq!(r.read(DDR + CR27, Size::Long), 0);
    r.write(DDR + CR09, Size::Long, 0x0000_0100);
    assert_eq!(r.read(DDR + CR27, Size::Long), 0, "no START, no init");
    r.write(DDR + CR09, Size::Long, START | 0x100);
    // INTSTATUS bit 3, with bit 8 the OR of bits 7:0 (RM Table 21-32): 0x108, measured on an MCF54415 after
    // the controller's initialization.
    assert_eq!(r.read(DDR + CR27, Size::Long), INT_ANY | INIT_DONE);
    assert_eq!(r.read(DDR + CR09, Size::Long), START | 0x100);
    // CR27 is read-only: a write changes nothing.
    r.write(DDR + CR27, Size::Long, 0);
    assert_eq!(r.read(DDR + CR27, Size::Long), INT_ANY | INIT_DONE);
    // INTACK (CR25 bits 23:16, write-only) clears INTSTATUS bits; the rest of CR25 reads back.
    r.write(DDR + CR25, Size::Long, 0x0A08_0005);
    assert_eq!(r.read(DDR + CR27, Size::Long), 0);
    assert_eq!(r.read(DDR + CR25, Size::Long), 0x0A00_0005);
    // guess: rewriting CR09 with START still set does not restart the initialization.
    r.write(DDR + CR09, Size::Long, START);
    assert_eq!(r.read(DDR + CR27, Size::Long), 0);
    r.write(DDR + CR09, Size::Long, 0);
    r.write(DDR + CR09, Size::Byte, 1);
    assert_eq!(
        r.read(DDR + CR27, Size::Long),
        INT_ANY | INIT_DONE,
        "a byte write of START"
    );
}

#[test]
fn ddr_writes_are_recorded_in_order() {
    let mut r = rig();
    r.write(DDR + RCR, Size::Long, 0);
    r.write(DDR + RCR, Size::Long, 0x4000_0000);
    r.write(DDR + PADCR, Size::Long, 0x0001_0201);
    r.write(DDR + 0x08, Size::Word, 0x0102);
    r.write(DDR + 0x0B, Size::Byte, 0x7);
    r.write(DDR + CR09, Size::Long, START);
    assert_eq!(
        r.m.device::<Ddrmc>(r.ddr).writes(),
        [
            (RCR, Size::Long, 0),
            (RCR, Size::Long, 0x4000_0000),
            (PADCR, Size::Long, 0x0001_0201),
            (0x08, Size::Word, 0x0102),
            (0x0B, Size::Byte, 0x7),
            (CR09, Size::Long, START),
        ]
    );
    // RCR[RST] is write-only and reads 0 (RM Table 21-58); the other registers read back, a narrow write
    // landing in its byte lanes.
    assert_eq!(r.read(DDR + RCR, Size::Long), 0);
    assert_eq!(r.read(DDR + PADCR, Size::Long), 0x0001_0201);
    // guess: PADCR bit 9 reads 1 and cannot be cleared (RM Figure 21-56 shows it read-only).
    r.write(DDR + PADCR, Size::Long, 0x0003_0001);
    assert_eq!(r.read(DDR + PADCR, Size::Long), 0x0003_0201);
    assert_eq!(r.read(DDR + 0x08, Size::Long), 0x0102_0007);
    assert_eq!(r.read(DDR + 0x0A, Size::Word), 0x0007);
    assert_eq!(r.read(DDR + 0x09, Size::Byte), 0x02);
}

#[test]
fn ddr_read_only_registers_keep_their_values() {
    let mut r = rig();
    for off in [CR27, 0x78, CR44, 0xD4, 0xDC] {
        let before = r.read(DDR + off, Size::Long);
        r.write(DDR + off, Size::Long, 0xFFFF_FFFF);
        assert_eq!(r.read(DDR + off, Size::Long), before, "+{off:#x}");
    }
    assert_eq!(r.read(DDR + CR44, Size::Long), 0x2040, "VERSION");
}

#[test]
fn pll_reset_values_follow_the_measurement() {
    // CR and DR are what the boot configuration left (the constructor's); SR reads LOCK and LOCKS, measured on an
    // MCF54415 (RM Table 8-5: both set once the PLL has locked after reset).
    let mut r = rig();
    assert_eq!(r.read(PLL + PLL_CR, Size::Long), TEST_CR);
    assert_eq!(r.read(PLL + PLL_DR, Size::Long), TEST_DR);
    assert_eq!(r.read(PLL + PLL_SR, Size::Long), LOCK | LOCKS);
    assert_eq!(r.read(PLL + 0xC, Size::Long), 0);
}

#[test]
fn pll_reads_its_configuration_and_records_writes() {
    let mut r = rig();
    r.write(PLL + PLL_DR, Size::Long, 0x0041_0821);
    r.write(PLL + PLL_CR, Size::Long, 0x0000_0118);
    r.write(PLL + PLL_SR, Size::Long, 0x0000_0240);
    assert_eq!(r.read(PLL + PLL_DR, Size::Long), 0x0041_0821);
    assert_eq!(r.read(PLL + PLL_CR, Size::Long), 0x0000_0118);
    // Reserved bits read 0 (RM Figures 8-3 and 8-4).
    r.write(PLL + PLL_CR, Size::Long, 0xFFFF_FFFF);
    assert_eq!(r.read(PLL + PLL_CR, Size::Long), 0x0007_773F);
    r.write(PLL + PLL_DR, Size::Long, 0xFFFF_FFFF);
    assert_eq!(r.read(PLL + PLL_DR, Size::Long), 0x03FF_7FFF);
    assert_eq!(
        r.read(PLL + PLL_CR + 2, Size::Word),
        0x773F,
        "a narrow read"
    );
    assert_eq!(
        r.m.device::<Pll>(r.pll).writes(),
        [
            (PLL_DR, 0x0041_0821),
            (PLL_CR, 0x0000_0118),
            (PLL_SR, 0x0000_0240),
            (PLL_CR, 0xFFFF_FFFF),
            (PLL_DR, 0xFFFF_FFFF),
        ]
    );
}

#[test]
fn pll_status_flags_follow_the_rm() {
    // LOCK and LOCKS are read-only, LOCF and LOLF write-1-to-clear (RM Table 8-5); the model never loses lock or
    // clock, so no write changes SR.
    let mut r = rig();
    r.write(PLL + PLL_SR, Size::Long, 0xFFFF_FFFF);
    assert_eq!(r.read(PLL + PLL_SR, Size::Long), LOCK | LOCKS);
    r.write(PLL + PLL_SR, Size::Long, 0);
    assert_eq!(r.read(PLL + PLL_SR, Size::Long), LOCK | LOCKS);
    // guess: no relock time; a new CR leaves the PLL locked.
    r.write(PLL + PLL_CR, Size::Long, 0x0000_1018);
    assert_eq!(r.read(PLL + PLL_SR, Size::Long), LOCK | LOCKS);
}

#[test]
fn pll_narrow_divider_write_refuses() {
    // RM 8.2.2: PLL_DR takes a single 32-bit write.
    let mut r = rig();
    let d = r.refused(PLL + PLL_DR + 2, Size::Word, Some(0x0C41));
    assert!(d.contains("PLL") && d.contains("PLL_DR"), "{d}");
    assert_eq!(r.read(PLL + PLL_DR, Size::Long), TEST_DR);
    assert!(r.m.device::<Pll>(r.pll).writes().is_empty());
}

#[test]
fn nfc_reset_values_follow_the_rm() {
    // RM Table 22-3, all measured on an MCF54415.
    let mut r = rig();
    for (off, want) in [
        (CMD1, 0x30FF_0000),
        (CMD2, 0x007E_E000),
        (CAR, 0),
        (RAR, 0x1100_0000),
        (RPT, 0),
        (RAI, 1),
        (SR1, 0),
        (SR2, 0),
        (DMA1, 0),
        (DMACFG, 0),
        (SWAP, 0x0FFE_0FFE),
        (SECSZ, 0x420),
        (CFG, 0x000E_A631),
        (DMA2, 0),
        (ISR, 0x6000_0000),
    ] {
        assert_eq!(r.nfc_r(off), want, "+{off:#x}");
    }
    assert!(r.m.device::<Nfc>(r.nfc).reads().is_empty());
}

#[test]
fn nfc_page_read_fills_the_buffer() {
    let mut r = rig();
    r.write(NFC, Size::Long, 0xDEAD_BEEF);
    r.nfc_w(ISR, DONECLR | IDLECLR);
    assert_eq!(r.nfc_r(ISR), 0);
    let isr = r.read_row(2);
    assert_eq!(isr & (DONE | IDLE), DONE | IDLE);
    let page = array().pages[2].clone();
    assert!(page.len() < PAYLOAD);
    let got = r.buffer(0, PAYLOAD as u32);
    assert_eq!(&got[..page.len()], &page[..]);
    assert!(
        got[page.len()..].iter().all(|&b| b == 0xFF),
        "padded with 0xFF"
    );
    assert_eq!(r.buffer(ECCAD * 8, 1), [0], "ECC status: corrected");
    // The command is done: BUSY/START reads 0.
    assert_eq!(r.nfc_r(CMD2), READ_PAGE & !1);
    // A row past the array reads erased.
    r.read_row(0x20);
    assert!(r.buffer(0, 0x100).iter().all(|&b| b == 0xFF));
    assert_eq!(r.m.device::<Nfc>(r.nfc).reads(), [2, 0x20]);
}

#[test]
fn nfc_bad_row_reports_ecc_failure() {
    let mut r = rig();
    r.read_row(4);
    assert_eq!(r.buffer(ECCAD * 8, 1), [0x80], "CORFAIL (RM Table 22-19)");
    // Without ECCSRAM the status is not written (RM Table 22-17).
    r.write(NFC + ECCAD * 8, Size::Byte, 0x55);
    r.nfc_w(CFG, ECCAD << 22 | CFG_READ & !ECCSRAM);
    r.nfc_w(RAR, 0x1100_0004);
    r.nfc_w(CMD2, READ_PAGE);
    // Byte 0x3F8 lies past the payload, so the read leaves it alone.
    assert_eq!(r.buffer(ECCAD * 8, 1), [0x55]);
    assert_eq!(r.m.device::<Nfc>(r.nfc).reads(), [4, 4]);
    // guess: with ECCMODE 0 (bypass) nothing is corrected or found uncorrectable: the status byte is 0.
    r.nfc_w(CFG, ECCAD << 22 | ECCSRAM | 1);
    r.nfc_w(CMD2, READ_PAGE);
    assert_eq!(r.buffer(ECCAD * 8, 1), [0]);
}

#[test]
fn nfc_buffer_number_auto_increments() {
    // With CFG[AIBN] set (as at reset), a read moves CMD2[BUFNO] on by one (RM Figure 22-18), so restarting the
    // command reads into buffer 1, which the model refuses.
    let mut r = rig();
    r.nfc_w(CFG, CFG_READ | AIBN);
    r.nfc_w(CMD2, READ_PAGE);
    let cmd2 = r.nfc_r(CMD2);
    assert_eq!(cmd2, READ_PAGE & !1 | 1 << 1, "BUFNO 1");
    let d = r.refused(NFC_REGS + CMD2, Size::Long, Some(cmd2 | 1));
    assert!(d.contains("buffer 1"), "{d}");
    // Without AIBN, BUFNO stays.
    r.nfc_w(CFG, CFG_READ);
    r.nfc_w(CMD2, READ_PAGE);
    assert_eq!(r.nfc_r(CMD2), READ_PAGE & !1);
    assert_eq!(r.m.device::<Nfc>(r.nfc).reads(), [0, 0]);
}

#[test]
fn nfc_row_address_auto_increments() {
    // guess: with CFG[AIAD] set, RAR moves on by RAI after each page (RM Figure 22-18), also for a single page.
    let mut r = rig();
    r.nfc_w(CFG, CFG_READ | AIAD);
    r.nfc_w(RAI, 0x0000_0102);
    r.nfc_w(RAR, 0x1100_00FF);
    r.nfc_w(CMD2, READ_PAGE);
    assert_eq!(r.nfc_r(RAR), 0x1100_0201);
    assert_eq!(r.m.device::<Nfc>(r.nfc).reads(), [0xFF]);
}

#[test]
fn nfc_row_address_increment_wraps_within_the_row_bits() {
    // guess: as above; the row (RAR bits 23:0) wraps and the chip selects (bits 31:24) are kept
    let mut r = rig();
    r.nfc_w(CFG, CFG_READ | AIAD);
    r.nfc_w(RAI, 1);
    r.nfc_w(RAR, 0xFFFF_FFFF);
    r.nfc_w(CMD2, READ_PAGE);
    assert_eq!(r.nfc_r(RAR), 0xFF00_0000);
    assert_eq!(r.m.device::<Nfc>(r.nfc).reads(), [0xFF_FFFF]);
}

/// A longword read, or a write, at `off` made by a bus master straight into device `id`: the detail it halts with.
fn crossing<T: Device>(r: &mut Rig, id: DeviceId, off: u32, write: bool) -> String {
    r.m.with_device::<T, _>(id, |d, cx| {
        if write {
            d.write(off, Size::Long, 0x1234_5678, cx);
        } else {
            d.read(off, Size::Long, cx);
        }
    });
    let stop = r.m.run(Limit::default());
    assert_eq!(stop.kind, Stop::Unmodelled, "{}", stop.detail);
    stop.detail
}

#[test]
fn accesses_crossing_a_register_are_refused() {
    // only the core splits misaligned accesses (RM Table 3-12): a bus master's reaches the block whole
    let mut r = rig();
    let (ddr, pll, nfc) = (r.ddr, r.pll, r.nfc);
    for write in [false, true] {
        let d = crossing::<Ddrmc>(&mut r, ddr, 0x26, write);
        assert!(
            d.starts_with("DDR controller:") && d.contains("crosses"),
            "{d}"
        );
        let d = crossing::<Pll>(&mut r, pll, 0x3, write);
        assert!(d.starts_with("PLL:") && d.contains("crosses"), "{d}");
        let d = crossing::<Nfc>(&mut r, nfc, 0x3EFE, write);
        assert!(d.starts_with("NFC:") && d.contains("crosses"), "{d}");
    }
    assert_eq!(r.m.device::<Pll>(pll).writes(), []);
    assert_eq!(r.m.device::<Ddrmc>(ddr).writes(), []);
    assert_eq!(r.read(DDR + CR09, Size::Long), 0);
}

#[test]
fn nfc_isr_bits_clear_by_writing_one() {
    let mut r = rig();
    r.nfc_w(ISR, DONECLR);
    assert_eq!(r.nfc_r(ISR), IDLE);
    r.nfc_w(ISR, 0);
    assert_eq!(r.nfc_r(ISR), IDLE, "writing 0 clears nothing");
    r.nfc_w(ISR, IDLECLR);
    assert_eq!(r.nfc_r(ISR), 0);
    r.read_row(1);
    assert_eq!(r.nfc_r(ISR), DONE | IDLE);
    r.nfc_w(ISR, DONE | IDLE);
    assert_eq!(
        r.nfc_r(ISR),
        DONE | IDLE,
        "the status bits themselves are not w1c"
    );
}

#[test]
fn nfc_narrow_register_access_refuses() {
    let mut r = rig();
    let d = r.refused(NFC_REGS + ISR, Size::Byte, None);
    assert!(d.contains("NFC") && d.contains("1-byte"), "{d}");
    let d = r.refused(NFC_REGS + CMD2 + 2, Size::Word, Some(1));
    assert!(d.contains("NFC") && d.contains("2-byte"), "{d}");
    assert!(r.m.device::<Nfc>(r.nfc).reads().is_empty());
}

#[test]
fn nfc_buffer_accepts_any_size() {
    let mut r = rig();
    r.write(NFC + 0x100, Size::Long, 0x1122_3344);
    r.write(NFC + 0x104, Size::Word, 0x5566);
    r.write(NFC + 0x106, Size::Byte, 0x77);
    assert_eq!(r.read(NFC + 0x100, Size::Byte), 0x11, "big-endian");
    assert_eq!(r.read(NFC + 0x102, Size::Word), 0x3344);
    assert_eq!(r.read(NFC + 0x104, Size::Long), 0x5566_7700);
    assert_eq!(r.read(NFC + 0x3EFC, Size::Long), 0);
}

#[test]
fn nfc_unmodelled_commands_refuse() {
    let mut r = rig();
    // Page program (RM Figure 22-20), read ID, read status: only page reads are modelled.
    for (code, what) in [
        (0x807F_C001, "0x7fc0"),
        (0x9048_0401, "0x4804"),
        (0x7040_1801, "0x4018"),
    ] {
        let d = r.refused(NFC_REGS + CMD2, Size::Long, Some(code));
        assert!(d.contains("NFC") && d.contains(what), "{d}");
    }
    r.nfc_w(RPT, 2);
    let d = r.refused(NFC_REGS + CMD2, Size::Long, Some(READ_PAGE));
    assert!(d.contains("repeat"), "{d}");
    r.nfc_w(RPT, 1);
    r.nfc_w(CAR, 0x10);
    let d = r.refused(NFC_REGS + CMD2, Size::Long, Some(READ_PAGE));
    assert!(d.contains("column"), "{d}");
    r.nfc_w(CAR, 0);
    let d = r.refused(NFC_REGS + CMD2, Size::Long, Some(READ_PAGE | 2));
    assert!(d.contains("buffer 1"), "{d}");
    r.nfc_w(CFG, CFG_READ | DMAREQ);
    let d = r.refused(NFC_REGS + CMD2, Size::Long, Some(READ_PAGE));
    assert!(d.contains("DMA"), "{d}");
    r.nfc_w(CFG, CFG_READ | 2);
    let d = r.refused(NFC_REGS + CMD2, Size::Long, Some(READ_PAGE));
    assert!(d.contains("PAGECNT"), "{d}");
    r.nfc_w(CFG, CFG_READ | 1 << 6);
    let d = r.refused(NFC_REGS + CMD2, Size::Long, Some(READ_PAGE));
    assert!(d.contains("boot mode"), "{d}");
    r.nfc_w(CFG, CFG_READ);
    let d = r.refused(NFC_REGS + ISR, Size::Long, Some(DONEEN));
    assert!(d.contains("interrupt"), "{d}");
    let d = r.refused(NFC_REGS + SWAP, Size::Long, Some(0x0FFE_0100));
    assert!(d.contains("swap"), "{d}");
    assert_eq!(r.nfc_r(SWAP), 0x0FFE_0FFE);
    // Writing CMD2 without START only stores it.
    r.nfc_w(CMD2, 0x9048_0400);
    assert_eq!(r.nfc_r(CMD2), 0x9048_0400);
    assert!(r.m.device::<Nfc>(r.nfc).reads().is_empty());
}

#[test]
fn take_writes_drain_and_record_off_logs_nothing() {
    let mut r = rig();
    r.write(DDR + 8, Size::Long, 5);
    r.write(PLL + PLL_CR, Size::Long, TEST_CR);
    assert_eq!(
        r.m.device_mut::<Ddrmc>(r.ddr).take_writes(),
        [(8, Size::Long, 5)]
    );
    assert_eq!(
        r.m.device_mut::<Pll>(r.pll).take_writes(),
        [(PLL_CR, TEST_CR)]
    );
    assert!(r.m.device::<Ddrmc>(r.ddr).writes().is_empty());
    assert!(r.m.device::<Pll>(r.pll).writes().is_empty());
    r.m.device_mut::<Ddrmc>(r.ddr).record(false);
    r.m.device_mut::<Pll>(r.pll).record(false);
    r.write(DDR + CR09, Size::Long, START);
    r.write(PLL + PLL_CR, Size::Long, 0x1B);
    assert_eq!(r.read(DDR + CR27, Size::Long), INIT_DONE | INT_ANY);
    assert_eq!(r.read(PLL + PLL_CR, Size::Long), 0x1B);
    assert!(r.m.device::<Ddrmc>(r.ddr).writes().is_empty());
    assert!(r.m.device::<Pll>(r.pll).writes().is_empty());
}

#[test]
fn unlocked_pll_reads_no_lock() {
    let mut r = rig();
    assert_eq!(r.read(PLL + PLL_SR, Size::Long), LOCK | LOCKS);
    r.m.device_mut::<Pll>(r.pll).set_locked(false);
    assert_eq!(r.read(PLL + PLL_SR, Size::Long), 0);
    assert_eq!(r.read(PLL + PLL_SR, Size::Byte), 0);
    r.write(PLL + PLL_CR, Size::Long, TEST_CR);
    assert_eq!(
        r.read(PLL + PLL_CR, Size::Long),
        TEST_CR,
        "the rest is unchanged"
    );
    r.m.device_mut::<Pll>(r.pll).set_locked(true);
    assert_eq!(r.read(PLL + PLL_SR, Size::Long), LOCK | LOCKS);
}

#[test]
fn ddr_init_that_never_completes() {
    let mut r = rig();
    r.m.device_mut::<Ddrmc>(r.ddr).set_init_completes(false);
    r.write(DDR + CR09, Size::Long, START);
    assert_eq!(
        r.read(DDR + CR09, Size::Long),
        START,
        "START still reads back"
    );
    assert_eq!(r.read(DDR + CR27, Size::Long), 0, "no INIT_DONE, no OR bit");
    // guess: a START made while the knob was off does not complete when it is turned back on
    r.m.device_mut::<Ddrmc>(r.ddr).set_init_completes(true);
    r.write(DDR + CR09, Size::Long, START);
    assert_eq!(r.read(DDR + CR27, Size::Long), 0);
    r.write(DDR + CR09, Size::Long, 0);
    r.write(DDR + CR09, Size::Long, START);
    assert_eq!(r.read(DDR + CR27, Size::Long), INIT_DONE | INT_ANY);
}
