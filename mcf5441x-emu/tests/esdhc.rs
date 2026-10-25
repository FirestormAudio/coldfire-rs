//! The eSDHC (RM ch. 25) with an eMMC behind it, driven through its registers the way a read-only driver drives it:
//! commands through CMDARG and XFERTYP, block reads that eDMA channel 59 moves from DATPORT, completion on INTC2
//! source 31.

use mcf5441x_emu::parts::{Emmc, EmmcConfig};
use mcf5441x_emu::soc::{Edma, Esdhc, EsdhcConfig, Intc};
use mcf5441x_emu::{Device, DeviceId, Fault, Machine, Size, Stop};

const SD: u32 = 0xFC0C_C000;
const EDMA: u32 = 0xFC04_4000;
const INTC1: u32 = 0xFC04_C000;
const INTC2: u32 = 0xFC05_0000;
const RAM: u32 = 0x4000_0000;

const DSADDR: u32 = 0x00;
const BLKATTR: u32 = 0x04;
const CMDARG: u32 = 0x08;
const XFERTYP: u32 = 0x0C;
const CMDRSP0: u32 = 0x10;
const CMDRSP3: u32 = 0x1C;
const DATPORT: u32 = 0x20;
const PRSSTAT: u32 = 0x24;
const PROCTL: u32 = 0x28;
const SYSCTL: u32 = 0x2C;
const IRQSTAT: u32 = 0x30;
const IRQSTATEN: u32 = 0x34;
const IRQSIGEN: u32 = 0x38;
const AUTOC12ERR: u32 = 0x3C;
const HOSTCAPBLT: u32 = 0x40;
const WML: u32 = 0x44;
const ADMAESR: u32 = 0x54;
const ADMASAR: u32 = 0x58;
const VENDOR: u32 = 0xC0;
const HOSTVER: u32 = 0xFC;

/// IRQSTAT (RM Table 25-14).
const CC: u32 = 1 << 0;
const TC: u32 = 1 << 1;
const CTOE: u32 = 1 << 16;
const DTOE: u32 = 1 << 20;
const DCE: u32 = 1 << 21;
/// PRSSTAT (RM Table 25-11).
const CDIHB: u32 = 1 << 1;
const DLA: u32 = 1 << 2;
const BWEN: u32 = 1 << 10;
const BREN: u32 = 1 << 11;
const DAT0: u32 = 1 << 24;
/// SYSCTL (RM Table 25-13).
const RSTA: u32 = 1 << 24;
const RSTC: u32 = 1 << 25;
const RSTD: u32 = 1 << 26;
const INITA: u32 = 1 << 27;

/// The commands, as XFERTYP values.
const CMD0: u32 = 0x0000_0000;
const CMD1: u32 = 0x0102_0000;
const CMD3: u32 = 0x031A_0000;
const CMD6: u32 = 0x061B_0000;
const CMD8: u32 = 0x083A_0010;
const CMD9: u32 = 0x0909_0000;
const CMD12: u32 = 0x0C1B_0000;
const CMD14: u32 = 0x0E3A_0010;
const CMD16: u32 = 0x101A_0000;
const CMD18: u32 = 0x123A_0036;
const CMD19: u32 = 0x133A_0000;
const CMD13: u32 = 0x0D1A_0000;
/// CMD25 with DPSEL, MSBSEL, AC12EN and BCEN, DTDSEL clear (a write).
const CMD25: u32 = 0x193A_0026;
const CMD35: u32 = 0x231A_0000;
const CMD36: u32 = 0x241A_0000;
/// CMD38, R1b.
const CMD38: u32 = 0x261B_0000;
/// CMD38's TRIM argument, and the card status bits (JESD84 Table 68).
const TRIM: u32 = 1;
const OUT_OF_RANGE: u32 = 1 << 31;
const ERASE_PARAM: u32 = 1 << 27;
/// CMD6 SWITCH write-byte BUS_WIDTH (EXT_CSD 183) = 1: a 4-bit bus.
const BUS_WIDTH_4: u32 = 0x03B7_0100;
/// PROCTL with DTW 4-bit (EMODE address-invariant).
const PROCTL_4BIT: u32 = 0x22;

/// INTC2 source 31 (RM Table 17-17).
const SOURCE: u8 = 31;
const CH: u32 = 59;
const LATENCY: u64 = 1000;
const BUSY: u64 = 2000;
const WRITE_BUSY: u64 = 3000;
const ERASE_BUSY: u64 = 5000;
/// The card's size in sectors: a test value.
const SECTORS: u32 = 0x0076_8000;
const OCR: u32 = 0xC0FF_8080;
const CID: [u32; 4] = [0x0000_0001, 0, 0, 0x0015_0000];
const CSD: [u32; 4] = [0x1111_1111, 0x2222_2222, 0x3333_3333, 0x0044_4444];

struct Board {
    m: Machine,
    sd: DeviceId,
}

fn card(image: &[u8]) -> Emmc {
    Emmc::new(
        image.to_vec(),
        EmmcConfig {
            sec_count: SECTORS,
            cid: CID,
            csd: CSD,
            ext_csd: vec![(155, 1)],
            ocr: OCR,
        },
    )
}

/// RAM, INTC1 and INTC2, the eDMA and the eSDHC with `card` behind it.
fn board_with(card: Option<Emmc>) -> Board {
    let mut m = Machine::new();
    m.mem.map_ram(RAM, 0x0100_0000);
    let ([_, intc1, intc2], _) = Intc::chip();
    m.add_interrupt_controller(INTC1, 0x4000, Box::new(intc1));
    m.add_interrupt_controller(INTC2, 0x4000, Box::new(intc2));
    m.add_bus_master(EDMA, 0x2000, Box::new(Edma::new()));
    let sd = m.add_device(
        SD,
        0x4000,
        Box::new(Esdhc::new(
            EsdhcConfig {
                controller: 2,
                source: SOURCE,
                dma: CH as u8,
                data_latency: LATENCY,
                cmd6_busy: BUSY,
                write_busy: WRITE_BUSY,
                erase_busy: ERASE_BUSY,
            },
            card,
        )),
    );
    Board { m, sd }
}

fn board(image: &[u8]) -> Board {
    board_with(Some(card(image)))
}

impl Board {
    fn r(&mut self, off: u32) -> u32 {
        let m = &mut self.m;
        m.access(SD + off, Size::Long, None)
            .unwrap_or_else(|f| panic!("read +{off:#x}: {f:?} {:?}", m.take_refusal()))
    }

    fn w(&mut self, off: u32, value: u32) {
        let m = &mut self.m;
        m.access(SD + off, Size::Long, Some(value))
            .unwrap_or_else(|f| panic!("write +{off:#x}: {f:?} {:?}", m.take_refusal()));
    }

    /// A write the eSDHC refuses: returns the detail.
    fn refused(&mut self, off: u32, value: u32) -> String {
        assert_eq!(
            self.m.access(SD + off, Size::Long, Some(value)),
            Err(Fault::Device(Stop::Unmodelled)),
            "+{off:#x} = {value:#x}"
        );
        self.m.take_refusal().expect("a refusal").detail
    }

    fn esdhc(&self) -> &Esdhc {
        self.m.device::<Esdhc>(self.sd)
    }

    fn idle(&mut self, tick: u64) {
        assert_eq!(self.m.idle_until(tick), None);
    }

    /// Issue a command; with `settle`, also idle to its data phase. Returns IRQSTAT and CMDRSP0.
    fn cmd(&mut self, xfertyp: u32, arg: u32, settle: bool) -> (u32, u32) {
        self.w(CMDARG, arg);
        self.w(XFERTYP, xfertyp);
        if settle && let Some(t) = self.esdhc().next_event() {
            self.idle(t);
        }
        (self.r(IRQSTAT), self.r(CMDRSP0))
    }

    /// Point channel 59 at `dst` for `nbytes`: 32-bit words from DATPORT, 16 bytes a request; then SERQ.
    fn arm(&mut self, dst: u32, nbytes: u32) {
        let t = EDMA + 0x1000 + 32 * CH;
        for (off, size, v) in [
            (0x00, Size::Long, SD + DATPORT),
            (0x04, Size::Word, 0x0202),
            (0x06, Size::Word, 0),
            (0x08, Size::Long, 16),
            (0x0C, Size::Long, 0),
            (0x10, Size::Long, dst),
            (0x16, Size::Word, 4),
            (0x14, Size::Word, nbytes / 16),
            (0x1C, Size::Word, nbytes / 16),
            (0x18, Size::Long, 0),
            (0x1E, Size::Word, 0),
        ] {
            self.m.access(t + off, size, Some(v)).unwrap();
        }
        self.m.access(EDMA + 0x18, Size::Byte, Some(CH)).unwrap();
    }

    /// Point channel 59 at DATPORT from `src` for `nbytes`: 32-bit words, 16 bytes a request; then SERQ.
    fn arm_write(&mut self, src: u32, nbytes: u32) {
        let t = EDMA + 0x1000 + 32 * CH;
        for (off, size, v) in [
            (0x00, Size::Long, src),
            (0x04, Size::Word, 0x0202),
            (0x06, Size::Word, 4),
            (0x08, Size::Long, 16),
            (0x0C, Size::Long, 0),
            (0x10, Size::Long, SD + DATPORT),
            (0x16, Size::Word, 0),
            (0x14, Size::Word, nbytes / 16),
            (0x1C, Size::Word, nbytes / 16),
            (0x18, Size::Long, 0),
            (0x1E, Size::Word, 0),
        ] {
            self.m.access(t + off, size, Some(v)).unwrap();
        }
        self.m.access(EDMA + 0x18, Size::Byte, Some(CH)).unwrap();
    }

    /// Idle through every event the eSDHC has pending.
    fn settle(&mut self) {
        while let Some(t) = self.esdhc().next_event() {
            self.idle(t);
        }
    }

    fn card(&self) -> &Emmc {
        self.esdhc().card().unwrap()
    }

    fn tcd_csr(&mut self) -> u32 {
        let a = EDMA + 0x1000 + 32 * CH + 0x1E;
        self.m.access(a, Size::Word, None).unwrap()
    }

    fn ram(&self, addr: u32, len: u32) -> Vec<u8> {
        self.m.mem.peek(addr, len).unwrap()
    }

    fn fill(&mut self, addr: u32, byte: u8, len: usize) {
        self.m.mem.load(addr, &vec![byte; len]).unwrap();
    }

    fn source(&self) -> bool {
        self.m.mem.irq_lines().asserted(2) & 1 << SOURCE != 0
    }

    fn dma_line(&self) -> bool {
        self.m.dma_lines().asserted() & 1 << CH != 0
    }
}

#[test]
fn identification_responses() {
    let mut b = board(&[]);
    let (st, rsp) = b.cmd(CMD1, 0x40FF_8000, true);
    assert!(st & CC != 0);
    assert_eq!(rsp, OCR);
    b.w(IRQSTAT, 0xFFFF_FFFF);
    assert_eq!(b.r(IRQSTAT), 0, "IRQSTAT is write-1-to-clear");
    let (_, rsp) = b.cmd(CMD3, 0x0002_0000, true);
    assert_eq!(rsp, 0x900);
    let (_, rsp) = b.cmd(CMD9, 0x0002_0000, true);
    assert_eq!(rsp, CSD[0]);
    assert_eq!(
        [
            b.r(CMDRSP0),
            b.r(CMDRSP0 + 4),
            b.r(CMDRSP0 + 8),
            b.r(CMDRSP3)
        ],
        CSD
    );
    let (_, rsp) = b.cmd(0x0209_0000, 0, true); // CMD2
    assert_eq!(rsp, CID[0]);
    assert_eq!(b.r(CMDRSP3) >> 16 & 0xFF, 0x15, "MID in CMDRSP3 bits 23:16");
    let cmds: Vec<u8> = b
        .esdhc()
        .card()
        .unwrap()
        .commands()
        .iter()
        .map(|&(c, _)| c)
        .collect();
    assert_eq!(cmds, [1, 3, 9, 2]);
}

#[test]
fn cmd18_moves_sectors_through_channel_59() {
    let mut image = vec![0u8; 16 * 512];
    for i in 0..1024 {
        image[5 * 512 + i] = (i * 3) as u8;
    }
    let mut b = board(&image);
    b.w(IRQSIGEN, TC);
    b.arm(0x4010_2000, 1024);
    b.w(BLKATTR, 2 << 16 | 512);
    assert_eq!(b.r(BLKATTR), 1 << 16 | 512, "BLKCNT reads 1");
    // guess: the written count is the one the transfer uses
    let (st, _) = b.cmd(CMD18, 5, true);
    assert_eq!(b.ram(0x4010_2000, 1024), &image[5 * 512..7 * 512]);
    assert_eq!(st & (CC | TC), CC | TC);
    assert!(b.source());
    assert_ne!(b.tcd_csr() & 0x80, 0, "TCD DONE");
    assert!(!b.dma_line(), "the FIFO is drained");
    b.w(IRQSTAT, CC | TC);
    assert!(!b.source());
}

#[test]
fn reads_past_the_image_are_zero() {
    let mut b = board(&[0xAA; 512]);
    b.fill(0x4010_3000, 0x55, 1024);
    b.arm(0x4010_3000, 1024);
    b.w(BLKATTR, 2 << 16 | 512);
    b.cmd(CMD18, 0, true);
    let mut want = vec![0xAA; 512];
    want.extend([0; 512]);
    assert_eq!(b.ram(0x4010_3000, 1024), want);
}

#[test]
fn ext_csd_comes_word_reversed() {
    // measured on an MCF54415: EXT_CSD read through the 32-bit DATPORT lands at index ^ 3
    let mut b = board(&[]);
    b.arm(0x4010_4000, 512);
    b.w(BLKATTR, 1 << 16 | 512);
    let (st, _) = b.cmd(CMD8, 0, true);
    assert_eq!(st & (CC | TC), CC | TC);
    let ext = b.ram(0x4010_4000, 512);
    assert_eq!(ext[155 ^ 3], 1);
    assert_eq!(
        u32::from_be_bytes(ext[0xD4..0xD8].try_into().unwrap()),
        SECTORS,
        "SEC_COUNT (bytes 212-215, little-endian) read big-endian at +0xD4"
    );
    let others = ext
        .iter()
        .enumerate()
        .filter(|&(i, &v)| v != 0 && i != 155 ^ 3 && !(0xD4..0xD8).contains(&i))
        .count();
    assert_eq!(others, 0, "no other EXT_CSD defaults");
}

#[test]
fn bus_test_inverts_the_pattern_and_cmd19_reports_dce() {
    let mut b = board(&[]);
    b.w(BLKATTR, 1 << 16 | 4);
    b.cmd(CMD19, 0, true);
    assert_ne!(b.r(PRSSTAT) & BWEN, 0);
    b.w(DATPORT, 0x5A);
    // measured on an MCF54415: CMD19 ends with DCE even when the bus test passes.
    // guess: TC comes too and the data line is freed (the bring-up sequence waits for TC after the write and then
    // issues CMD14, which needs CDIHB clear)
    let st = b.r(IRQSTAT);
    assert_eq!(st & (TC | DCE), TC | DCE);
    assert_eq!(b.r(PRSSTAT) & (CDIHB | BWEN), 0);
    b.w(IRQSTAT, 0xFFFF_FFFF);
    b.cmd(CMD14, 0, true);
    assert_ne!(b.r(PRSSTAT) & BREN, 0);
    // guess: a status is latched before the word is read (the driver waits for one), DCE as for CMD19
    assert_eq!(b.r(IRQSTAT) & (TC | DCE), DCE);
    assert_eq!((b.r(DATPORT) ^ 0xA5) & 0xFF, 0);
    assert_eq!(b.r(IRQSTAT) & (TC | DCE), TC | DCE);
    assert_eq!(b.r(PRSSTAT) & CDIHB, 0, "the data line is free again");
}

#[test]
fn bus_test_word_written_before_bwen_is_lost() {
    let mut b = board(&[]);
    b.cmd(CMD19, 0, false);
    b.w(DATPORT, 0x5A); // too early
    b.idle(LATENCY);
    assert_eq!(b.r(IRQSTAT) & TC, 0, "no TC: the driver must wait for BWEN");
    assert_ne!(b.r(PRSSTAT) & CDIHB, 0);
}

#[test]
fn data_phase_follows_the_command_after_latency() {
    let mut b = board(&[0; 1024]);
    b.w(IRQSIGEN, TC);
    b.arm(0x4010_2000, 512);
    b.w(BLKATTR, 1 << 16 | 512);
    let (st, _) = b.cmd(CMD18, 0, false);
    assert_eq!(st & (CC | TC), CC, "CC now, TC later");
    assert_ne!(b.r(PRSSTAT) & (CDIHB | DLA), 0);
    assert!(!b.source());
    assert_eq!(b.esdhc().next_event(), Some(LATENCY));
    let mut wake = Vec::new();
    assert!(b.esdhc().wake_sources(&mut wake));
    assert_eq!(wake, [(2, SOURCE)]);
    b.idle(LATENCY);
    assert_ne!(b.r(IRQSTAT) & TC, 0);
    assert!(b.source());
    assert_eq!(b.r(PRSSTAT) & (CDIHB | DLA), 0);
    assert_eq!(b.esdhc().next_event(), None);
    let mut wake = Vec::new();
    assert!(b.esdhc().wake_sources(&mut wake));
    assert!(wake.is_empty(), "nothing pending");
}

#[test]
fn data_command_while_busy_halts() {
    let mut b = board(&[0; 1024]);
    b.w(BLKATTR, 1 << 16 | 512);
    b.cmd(CMD18, 0, false);
    b.w(CMDARG, 0);
    let d = b.refused(XFERTYP, CMD18);
    assert!(d.contains("CDIHB"), "{d}");
    assert_eq!(b.esdhc().card().unwrap().commands().len(), 1);
}

#[test]
fn rstd_cancels_a_pending_transfer() {
    let mut b = board(&[0xAA; 512]);
    b.fill(0x4010_6000, 0x55, 512);
    b.arm(0x4010_6000, 512);
    b.w(BLKATTR, 1 << 16 | 512);
    b.cmd(CMD18, 0, false);
    b.w(SYSCTL, RSTD);
    assert_eq!(b.esdhc().next_event(), None);
    b.idle(10 * LATENCY);
    assert_eq!(b.ram(0x4010_6000, 512), [0x55; 512]);
    assert_eq!(b.r(PRSSTAT) & (CDIHB | DLA), 0);
    assert_eq!(b.r(IRQSTAT) & TC, 0);
}

#[test]
fn a_failed_read_needs_rstd_and_cmd12() {
    let mut b = board(&[0xAA; 512]);
    *b.m.device_mut::<Esdhc>(b.sd).card_mut().unwrap().faults() = 1;
    b.arm(0x4010_7000, 512);
    b.w(BLKATTR, 1 << 16 | 512);
    let (st, _) = b.cmd(CMD18, 0, true);
    assert_eq!(st & (DCE | TC), DCE, "DCE, no TC");
    assert_ne!(b.r(PRSSTAT) & CDIHB, 0, "the data line stays busy");
    b.w(IRQSTAT, 0xFFFF_FFFF);
    b.w(SYSCTL, RSTC | RSTD);
    let (st, _) = b.cmd(CMD16, 512, true);
    assert_ne!(st & CTOE, 0, "the card is still sending: no response");
    assert_eq!(st & CC, 0);
    b.w(IRQSTAT, 0xFFFF_FFFF);
    let (st, _) = b.cmd(CMD12, 0, true);
    assert_eq!(st & (CC | CTOE), CC);
    b.w(IRQSTAT, 0xFFFF_FFFF);
    b.arm(0x4010_7000, 512);
    let (st, _) = b.cmd(CMD18, 0, true);
    assert_eq!(st & (CC | TC), CC | TC);
    assert_eq!(b.ram(0x4010_7000, 512), [0xAA; 512]);
}

#[test]
fn unmodelled_write_commands_halt() {
    // CMD24 (WRITE_BLOCK); CMD38 other than TRIM has its own test
    for cmd in [24] {
        let mut b = board(&[]);
        b.w(CMDARG, 0);
        let d = b.refused(XFERTYP, cmd << 24);
        assert!(d.contains(&format!("CMD{cmd}")), "{d}");
        assert!(b.esdhc().card().unwrap().commands().is_empty());
        assert!(b.esdhc().writes().iter().all(|&(o, _)| o != XFERTYP));
    }
}

#[test]
fn emode_other_than_address_invariant_halts() {
    let mut b = board(&[]);
    b.w(PROCTL, 0);
    let d = b.refused(XFERTYP, CMD0);
    assert!(d.contains("EMODE"), "{d}");
}

#[test]
fn rsta_leaves_sysctl_at_its_reset_value() {
    // measured on an MCF54415: an RSTA write leaves SYSCTL at its reset value 0x8008 and drops the rest of the
    // written value; the next plain write is stored
    let mut b = board(&[]);
    b.w(SYSCTL, RSTA);
    assert_eq!(b.r(SYSCTL), 0x8008);
    assert_eq!(b.r(PROCTL), 0x20);
    assert_eq!(b.r(WML), 0x0810_0810);
    b.w(SYSCTL, RSTA | INITA | 0x1058);
    assert_eq!(b.r(SYSCTL), 0x8008);
    b.w(SYSCTL, 0x1058);
    assert_eq!(b.r(SYSCTL), 0x1058);
}

#[test]
fn no_card_reads_zero() {
    // a card with an empty image reads zeros
    let mut b = board(&[]);
    b.fill(0x4010_5000, 0x55, 512);
    b.arm(0x4010_5000, 512);
    b.w(BLKATTR, 1 << 16 | 512);
    let (st, _) = b.cmd(CMD18, 0, true);
    assert_eq!(st & (CC | TC), CC | TC);
    assert_eq!(b.ram(0x4010_5000, 512), [0; 512]);
    // with no card at all nothing answers: CTOE, no data phase
    let mut b = board_with(None);
    b.fill(0x4010_5000, 0x55, 512);
    b.arm(0x4010_5000, 512);
    b.w(BLKATTR, 1 << 16 | 512);
    let (st, _) = b.cmd(CMD18, 0, true);
    assert_eq!(st & (CC | CTOE | TC), CTOE);
    assert_eq!(b.r(PRSSTAT) & CDIHB, 0);
    assert_eq!(b.esdhc().next_event(), None);
    assert_eq!(b.ram(0x4010_5000, 512), [0x55; 512]);
    assert!(b.esdhc().card().is_none());
}

#[test]
fn reserved_irqsigen_bits_halt() {
    let mut b = board(&[]);
    b.w(IRQSIGEN, 0x117F_01FF); // every defined status bit: accepted
    assert_eq!(b.r(IRQSIGEN), 0x117F_01FF);
    let d = b.refused(IRQSIGEN, 0xFFFF_0002); // TC plus reserved bits
    assert!(d.contains("IRQSIGEN"), "{d}");
    assert_eq!(
        b.r(IRQSIGEN),
        0x117F_01FF,
        "a refused write changes nothing"
    );
}

#[test]
fn register_writes_are_recorded_in_order() {
    let mut b = board(&[]);
    b.w(SYSCTL, 0x1058);
    b.w(CMDARG, 0x40FF_8000);
    b.w(XFERTYP, CMD1);
    assert_eq!(
        b.esdhc().writes(),
        [(SYSCTL, 0x1058), (CMDARG, 0x40FF_8000), (XFERTYP, CMD1)]
    );
}

#[test]
fn reset_values_follow_the_rm() {
    // RM Table 25-2, measured on an MCF54415 (SYSCTL 0x8008: SDCLKEN set at reset)
    let reset = [
        (DSADDR, 0),
        (BLKATTR, 0x0001_0000),
        (CMDARG, 0),
        (XFERTYP, 0),
        (CMDRSP0, 0),
        (CMDRSP3, 0),
        (DATPORT, 0),
        (PRSSTAT, 0xFF88_00F8),
        (PROCTL, 0x20),
        (SYSCTL, 0x8008),
        (IRQSTAT, 0),
        (IRQSTATEN, 0x117F_013F),
        (IRQSIGEN, 0),
        (AUTOC12ERR, 0),
        (HOSTCAPBLT, 0x07F3_0000),
        (WML, 0x0810_0810),
        (ADMAESR, 0),
        (ADMASAR, 0),
        (VENDOR, 1),
        (HOSTVER, 0x1201),
    ];
    let mut b = board(&[]);
    for (off, v) in reset {
        assert_eq!(b.r(off), v, "+{off:#x}");
    }
    // measured on an MCF54415: an empty DATPORT reads 0; RSTA puts DSADDR, BLKATTR, CMDARG, PROCTL, IRQSTATEN,
    // IRQSIGEN, WML, ADMASAR, VENDOR and SYSCTL (the rest of the written value is dropped) back to their reset
    // values. guess: the rest too (RM Table 25-13: "affects the entire host controller")
    for (off, v) in [
        (DSADDR, 0x4010_0000),
        (BLKATTR, 3 << 16 | 512),
        (CMDARG, 7),
        (PROCTL, PROCTL_4BIT),
        (IRQSTATEN, 0x3),
        (IRQSIGEN, 0x3),
        (WML, 0x0004_0004),
        (ADMASAR, 0x4010_0000),
        (VENDOR, 0),
    ] {
        b.w(off, v);
    }
    b.w(SYSCTL, 0x8000);
    b.w(SYSCTL, 0x1050);
    b.w(SYSCTL, RSTA | 0x1050);
    for (off, v) in reset {
        assert_eq!(b.r(off), v, "+{off:#x} after RSTA");
    }
}

#[test]
fn irqstaten_gates_status_bits() {
    // RM 25.3.11: a status bit whose IRQSTATEN bit is clear is cleared and never set
    let mut b = board(&[]);
    b.w(IRQSIGEN, CC);
    b.w(IRQSTATEN, 0x117F_013F & !CC);
    let (st, rsp) = b.cmd(CMD1, 0, true);
    assert_eq!(st & CC, 0, "CC masked");
    assert_eq!(rsp, OCR, "the command still completed");
    assert!(!b.source());
    b.w(IRQSTATEN, 0xFFFF_FFFF);
    assert_eq!(b.r(IRQSTATEN), 0x117F_01FF, "reserved bits read 0");
    let (st, _) = b.cmd(CMD1, 0, true);
    assert_ne!(st & CC, 0);
    assert!(b.source());
    b.w(IRQSTATEN, 0x117F_01FF & !CC);
    assert_eq!(
        b.r(IRQSTAT) & CC,
        0,
        "clearing the enable clears the status bit"
    );
    assert!(!b.source());
}

#[test]
fn blkattr_reads_blkcnt_1_and_wml_its_levels_measured() {
    // measured on an MCF54415, no transfer running: BLKATTR writes of BLKCNT 2, 3, 0x20 and 0x40 read back BLKCNT
    // 1 while BLKSIZE takes (0x0020_0200 read 0x0001_0200); WML reads 0x0800_0800 with the written levels
    let mut b = board(&[]);
    for count in [2, 3, 0x20, 0x40] {
        b.w(BLKATTR, count << 16 | 0x200);
        assert_eq!(b.r(BLKATTR), 0x0001_0200, "BLKCNT {count:#x}");
    }
    b.w(BLKATTR, 3 << 16 | 4);
    assert_eq!(b.r(BLKATTR), 0x0001_0004);
    b.w(WML, 0x0010_0010);
    assert_eq!(b.r(WML), 0x0810_0810);
    b.w(WML, 0x0004_0020);
    assert_eq!(b.r(WML), 0x0804_0820);
}

#[test]
fn rstd_restores_proctl_and_wml() {
    // measured on an MCF54415: RSTD (and RSTA) put PROCTL back to 0x20 and WML to 0x0810_0810, though RM Table
    // 25-13's RSTD list leaves them out
    let mut b = board(&[]);
    b.w(PROCTL, PROCTL_4BIT);
    b.w(WML, 0x0004_0004);
    assert_eq!(
        b.r(WML),
        0x0804_0804,
        "bits 27 and 11 read 1 (RM Figure 25-17)"
    );
    b.w(SYSCTL, RSTD | 0x8008);
    assert_eq!(b.r(PROCTL), 0x20);
    assert_eq!(b.r(WML), 0x0810_0810);
    // RM Table 25-13: RSTD clears TC (with BRR, BWR, DINT, BGE) and RSTC clears CC; neither clears the errors
    b.w(IRQSTATEN, 0x117F_01FF);
    b.cmd(CMD1, 0, true);
    b.w(PROCTL, PROCTL_4BIT);
    b.w(SYSCTL, RSTC | 0x8008);
    assert_eq!(b.r(IRQSTAT) & CC, 0);
    assert_eq!(b.r(PROCTL), PROCTL_4BIT, "RSTC leaves PROCTL");
    b.w(PROCTL, 0x20);
    b.arm(0x4010_2000, 512);
    b.w(BLKATTR, 1 << 16 | 512);
    b.cmd(CMD18, 0, true);
    b.cmd(CMD1, 0x0002_0000, true);
    let st = b.r(IRQSTAT);
    assert_eq!(st & (CC | TC), CC | TC);
    b.w(SYSCTL, RSTD | 0x8008);
    assert_eq!(b.r(IRQSTAT) & (CC | TC), CC);
    b.w(SYSCTL, RSTA | 0x8008);
    assert_eq!(b.r(IRQSTAT), 0, "RSTA clears every status bit");
}

#[test]
fn cmd6_busy_holds_dat0_and_refuses_commands() {
    let mut b = board(&[]);
    assert_ne!(b.r(PRSSTAT) & DAT0, 0, "DAT0 idles high");
    let (st, _) = b.cmd(CMD6, BUS_WIDTH_4, true);
    assert_ne!(st & CC, 0);
    assert_eq!(
        b.r(PRSSTAT) & DAT0,
        0,
        "the card holds DAT0 low while it switches"
    );
    b.w(CMDARG, 0);
    let d = b.refused(XFERTYP, CMD16);
    assert!(d.contains("busy"), "{d}");
    b.idle(BUSY - 1);
    assert_eq!(b.r(PRSSTAT) & DAT0, 0);
    b.idle(BUSY);
    assert_ne!(b.r(PRSSTAT) & DAT0, 0);
    let (st, _) = b.cmd(CMD16, 512, true);
    assert_ne!(st & CC, 0);
}

#[test]
fn divider_change_with_sdclken_set_refuses() {
    // RM 25.3.9 SDCLKEN: change the SD clock frequency only with SDCLKEN clear. SDCLKEN set by a software write
    // guards the divider; the reset value's SDCLKEN does not
    let mut b = board(&[]);
    b.w(SYSCTL, 0x1058);
    let d = b.refused(SYSCTL, 0x1068);
    assert!(d.contains("divider"), "{d}");
    assert_eq!(b.r(SYSCTL), 0x1058);
    b.w(SYSCTL, 0x1050);
    b.w(SYSCTL, 0x8000);
    b.w(SYSCTL, 0x1050);
    b.w(SYSCTL, 0x1058);
    b.w(SYSCTL, 0x0E_1058); // DTOCV is not the divider
    assert_eq!(b.r(SYSCTL), 0x0E_1058);
}

#[test]
fn first_sysctl_write_after_reset_may_change_the_divider() {
    // SYSCTL resets to 0x8008 with SDCLKEN set (measured on an MCF54415), and a driver's first write after reset
    // sets its own divider with SDCLKEN: only SDCLKEN set by a write since the last reset (power-on or RSTA) guards
    // the divider
    let mut b = board(&[]);
    b.w(SYSCTL, 0x1058);
    let d = b.refused(SYSCTL, 0x8008);
    assert!(d.contains("divider"), "{d}");
    b.w(SYSCTL, RSTA);
    assert_eq!(b.r(SYSCTL), 0x8008);
    b.w(SYSCTL, 0x8008); // SDCLKEN now set by a write
    let d = b.refused(SYSCTL, 0x1058);
    assert!(d.contains("divider"), "{d}");
    // RSTA forgets the write that set SDCLKEN, and the value written with it is dropped (measured on an MCF54415):
    // the reset SDCLKEN does not guard the divider
    let mut b = board(&[]);
    b.w(SYSCTL, 0x1058);
    b.w(SYSCTL, RSTA | 0x1058);
    assert_eq!(b.r(SYSCTL), 0x8008);
    b.w(SYSCTL, 0x2058);
    let d = b.refused(SYSCTL, 0x3058);
    assert!(d.contains("divider"), "{d}");
}

#[test]
fn bus_width_mismatch_reads_fail_with_dce() {
    // measured on an MCF54415: after RSTD put the host back to a 1-bit bus, with the card still at 4 bits, every
    // read failed its CRC
    let mut b = board(&[0xAA; 512]);
    b.cmd(CMD6, BUS_WIDTH_4, true);
    b.idle(BUSY);
    b.w(PROCTL, PROCTL_4BIT);
    b.arm(0x4010_2000, 512);
    b.w(BLKATTR, 1 << 16 | 512);
    let (st, _) = b.cmd(CMD18, 0, true);
    assert_eq!(st & (TC | DCE), TC, "matched widths read");
    b.w(IRQSTAT, 0xFFFF_FFFF);
    b.w(SYSCTL, RSTD | 0x8008);
    b.fill(0x4010_2000, 0x55, 512);
    b.arm(0x4010_2000, 512);
    let (st, _) = b.cmd(CMD18, 0, true);
    assert_eq!(st & (TC | DCE), DCE);
    assert_eq!(b.ram(0x4010_2000, 512), [0x55; 512], "nothing moved");
    // the other way round: CMD0 puts the card back to 1 bit, the host stays at 4
    b.w(IRQSTAT, 0xFFFF_FFFF);
    b.w(SYSCTL, RSTD | 0x8008);
    b.w(PROCTL, PROCTL_4BIT);
    b.cmd(CMD0, 0, true);
    let (st, _) = b.cmd(CMD18, 0, true);
    assert_eq!(st & (TC | DCE), DCE);
}

#[test]
fn undrained_data_phase_times_out() {
    // guess: a data phase whose FIFO is not drained one tick after it began ends with DTOE and the FIFO cleared
    // (the RM's data timeout counter, SYSCTL DTOCV, is not modelled)
    let mut b = board(&[0xAA; 1024]);
    b.w(IRQSIGEN, DTOE);
    b.w(BLKATTR, 1 << 16 | 512);
    b.cmd(CMD18, 0, false);
    b.idle(LATENCY);
    assert!(
        b.dma_line(),
        "channel 59 is not enabled: the request stays up"
    );
    assert_eq!(b.r(IRQSTAT) & (TC | DTOE), 0);
    let mut wake = Vec::new();
    assert!(b.esdhc().wake_sources(&mut wake));
    assert_eq!(wake, [(2, SOURCE)]);
    b.idle(LATENCY + 1);
    assert!(!b.dma_line());
    assert_eq!(b.r(IRQSTAT) & (TC | DTOE), DTOE);
    assert!(b.source());
    assert_ne!(
        b.r(PRSSTAT) & CDIHB,
        0,
        "the data line stays busy until RSTD"
    );
    assert_eq!(b.r(DATPORT), 0, "the FIFO is cleared");
    // a channel that takes only half the data
    b.w(IRQSTAT, 0xFFFF_FFFF);
    b.w(SYSCTL, RSTD | 0x8008);
    b.arm(0x4010_2000, 512);
    let csr = EDMA + 0x1000 + 32 * CH + 0x1E;
    b.m.access(csr, Size::Word, Some(0x08)).unwrap(); // D_REQ: the request enable drops after 512 bytes
    b.w(BLKATTR, 2 << 16 | 512);
    let t = b.m.now();
    b.cmd(CMD18, 0, false);
    b.idle(t + LATENCY);
    assert_eq!(b.ram(0x4010_2000, 512), [0xAA; 512]);
    assert!(b.dma_line());
    b.idle(t + LATENCY + 1);
    assert_eq!(b.r(IRQSTAT) & (TC | DTOE), DTOE);
}

#[test]
fn take_writes_and_take_commands_drain_and_record_off_logs_nothing() {
    let mut b = board(&[0; 1024]);
    b.cmd(0x0209_0000, 0, true); // CMD2
    assert!(!b.esdhc().writes().is_empty());
    let writes = b.m.device_mut::<Esdhc>(b.sd).take_writes();
    assert!(writes.contains(&(CMDARG, 0)));
    assert!(b.esdhc().writes().is_empty());
    let cmds =
        b.m.device_mut::<Esdhc>(b.sd)
            .card_mut()
            .unwrap()
            .take_commands();
    assert_eq!(cmds, [(2, 0)]);
    assert!(b.esdhc().card().unwrap().commands().is_empty());
    b.m.device_mut::<Esdhc>(b.sd).record(false);
    b.m.device_mut::<Esdhc>(b.sd)
        .card_mut()
        .unwrap()
        .record(false);
    let (_, rsp) = b.cmd(0x0209_0000, 0, true);
    assert_eq!(rsp, CID[0], "the card still answers");
    assert!(b.esdhc().writes().is_empty());
    assert!(b.esdhc().card().unwrap().commands().is_empty());
}

#[test]
fn cmd25_writes_sectors_from_channel_59_and_busies_the_card() {
    let mut b = board(&[]);
    let data: Vec<u8> = (0..1024u32).map(|i| (i * 7) as u8).collect();
    b.m.mem.load(0x4010_2000, &data).unwrap();
    b.arm_write(0x4010_2000, 1024);
    b.w(BLKATTR, 2 << 16 | 512);
    let (st, _) = b.cmd(CMD25, 9, false);
    assert_eq!(st & CC, CC);
    let now = b.m.now();
    b.idle(now + LATENCY);
    // the eDMA filled the controller in the data phase's tick; the card is busy, then TC
    assert_eq!(b.card().written(), [(9, 2)]);
    assert_eq!(&b.card().image()[9 * 512..11 * 512], data.as_slice());
    assert_eq!(b.r(PRSSTAT) & DAT0, 0, "busy after the last block");
    assert_eq!(b.r(IRQSTAT) & TC, 0);
    b.idle(now + LATENCY + WRITE_BUSY);
    assert_eq!(b.r(IRQSTAT) & TC, TC);
    assert_eq!(
        b.r(PRSSTAT) & (DAT0 | CDIHB | DLA),
        DAT0,
        "the data line is free"
    );
    assert!(!b.dma_line());
    let (_, r1) = b.cmd(CMD13, 0x0002_0000, true);
    assert_eq!(r1, 0x900, "no error");
}

#[test]
fn a_write_past_the_end_is_reported_by_cmd13_once() {
    let mut b = board(&[]);
    b.arm_write(0x4010_2000, 1024);
    b.w(BLKATTR, 2 << 16 | 512);
    b.cmd(CMD25, SECTORS - 1, false);
    b.settle();
    assert_eq!(b.r(IRQSTAT) & TC, TC);
    assert!(b.card().written().is_empty());
    let (_, r1) = b.cmd(CMD13, 0x0002_0000, true);
    assert_eq!(r1, 0x900 | OUT_OF_RANGE);
    let (_, r1) = b.cmd(CMD13, 0x0002_0000, true);
    assert_eq!(r1, 0x900, "cleared by the read");
}

#[test]
fn an_injected_fault_fails_a_write_with_dce_and_writes_nothing() {
    let mut b = board(&[]);
    *b.m.device_mut::<Esdhc>(b.sd).card_mut().unwrap().faults() = 1;
    b.arm_write(0x4010_2000, 512);
    b.w(BLKATTR, 1 << 16 | 512);
    b.cmd(CMD25, 3, false);
    b.settle();
    assert_eq!(b.r(IRQSTAT) & (TC | DCE), DCE);
    assert!(b.card().written().is_empty());
}

#[test]
fn a_write_the_dma_never_feeds_times_out() {
    let mut b = board(&[]);
    b.w(BLKATTR, 1 << 16 | 512);
    b.cmd(CMD25, 3, false);
    b.settle();
    assert_eq!(b.r(IRQSTAT) & (TC | DTOE), DTOE);
    assert!(b.card().written().is_empty());
}

fn trim(b: &mut Board, start: u32, end: u32) {
    b.cmd(CMD35, start, true);
    b.cmd(CMD36, end, true);
    let now = b.m.now();
    let (st, _) = b.cmd(CMD38, TRIM, false);
    assert_eq!(st & (CC | TC), CC);
    assert_eq!(b.r(PRSSTAT) & DAT0, 0, "busy");
    b.idle(now + ERASE_BUSY);
    assert_eq!(b.r(IRQSTAT) & TC, TC, "TC when the busy ends");
}

#[test]
fn trim_fills_with_the_erased_value() {
    let image = vec![0x5A; 8 * 512];
    for (erased, byte) in [(1, 0xFF), (0, 0x00)] {
        let card = Emmc::new(
            image.clone(),
            EmmcConfig {
                sec_count: SECTORS,
                cid: CID,
                csd: CSD,
                ext_csd: vec![(181, erased)],
                ocr: OCR,
            },
        );
        let mut b = board_with(Some(card));
        trim(&mut b, 2, 4);
        let img = b.card().image();
        assert!(img[2 * 512..5 * 512].iter().all(|&x| x == byte));
        assert!(
            img[..2 * 512]
                .iter()
                .chain(&img[5 * 512..])
                .all(|&x| x == 0x5A)
        );
        assert_eq!(b.card().trimmed(), [(2, 4)]);
    }
}

#[test]
fn a_trim_ending_before_its_start_is_an_erase_param_error() {
    let mut b = board(&[]);
    trim(&mut b, 5, 4);
    assert!(b.card().trimmed().is_empty());
    let (_, r1) = b.cmd(CMD13, 0x0002_0000, true);
    assert_eq!(r1, 0x900 | ERASE_PARAM);
}

#[test]
fn only_trim_is_modelled_of_cmd38() {
    let mut b = board(&[]);
    b.w(CMDARG, 0);
    assert!(b.refused(XFERTYP, CMD38).contains("only TRIM"));
}

#[test]
fn erase_group_def_and_set_block_count_are_taken() {
    let mut b = board(&[]);
    // CMD6 write-byte ERASE_GROUP_DEF (EXT_CSD 175) = 1
    let (st, _) = b.cmd(CMD6, 0x03AF_0100, true);
    assert_ne!(st & CC, 0);
    b.idle(BUSY);
    b.w(IRQSTAT, 0xFFFF_FFFF);
    // CMD23 SET_BLOCK_COUNT, R1
    let (st, _) = b.cmd(0x171A_0000, 8, true);
    assert_eq!(st & (CC | CTOE), CC);
    let cmds = b.esdhc().card().unwrap().commands().to_vec();
    assert!(cmds.contains(&(6, 0x03AF_0100)) && cmds.contains(&(23, 8)), "{cmds:x?}");
    // other EXT_CSD bytes stay refused
    b.w(CMDARG, 0x03AE_0100);
    let d = b.refused(XFERTYP, CMD6);
    assert!(d.contains("CMD6"), "{d}");
}
