//! The chip map: every module of the MCF5441x at its RM base address, wired to its interrupt sources and DMA
//! request lines. The expected tables below are read from the RM: the module bases from RM Table 1-3 and Table
//! 1-4 (and RM Table 40-2 for the DSPIs, RM Table 1-2 and RM ch. 16 for RGPIO), the interrupt sources from RM
//! Table 17-15, Table 17-16 and Table 17-17, the DMA request lines from RM Table 19-6.

mod common;

use mcf5441x_emu::parts::{Emmc, EmmcConfig, SpiNor};
use mcf5441x_emu::soc::{
    Chip, ChipOptions, Ddrmc, Dspi, Edma, Esdhc, Gates, Intc, Mcf5441x, NandArray, Nfc, Pit, Pll,
    Ssi, StubBlock, Uart, UsbOtg,
};
use mcf5441x_emu::{Device, Fault, Limit, Machine, Size, Stop};

/// The DDR window the tests map RAM in (RM Table 1-2): 16 MiB of it.
const DDR: u32 = 0x4000_0000;

/// INTC0, INTC1, INTC2 (RM Table 1-3, slots 18-20).
const INTC: [u32; 3] = [0xFC04_8000, 0xFC04_C000, 0xFC05_0000];
/// The interrupt controller IACK slot (RM Table 1-3, slot 21).
const GIACK: u32 = 0xFC05_4000;
/// eDMA controller (RM Table 1-3, slot 17).
const EDMA: u32 = 0xFC04_4000;
/// PIT0-PIT3 (RM Table 1-3, slots 32-35) and their INTC2 sources (RM Table 17-17).
const PIT: [(u32, u8); 4] = [
    (0xFC08_0000, 13),
    (0xFC08_4000, 14),
    (0xFC08_8000, 15),
    (0xFC08_C000, 16),
];
/// UART0-UART9: base (RM Table 1-3 slots 24-27, Table 1-4 slots 24-29), controller and source (RM Table 17-15,
/// Table 17-16) and the receive DMA request (RM Table 19-6).
const UART: [(u32, usize, u8, u8); 10] = [
    (0xFC06_0000, 0, 26, 2),
    (0xFC06_4000, 0, 27, 4),
    (0xFC06_8000, 0, 28, 6),
    (0xFC06_C000, 0, 29, 16),
    (0xEC06_0000, 1, 48, 18),
    (0xEC06_4000, 1, 49, 20),
    (0xEC06_8000, 1, 50, 22),
    (0xEC06_C000, 1, 51, 32),
    (0xEC07_0000, 1, 52, 34),
    (0xEC07_4000, 1, 53, 36),
];
/// SSI0 and SSI1 (RM Table 1-3, slots 47 and 50) with their transmit 0 and receive 0 requests (RM Table 19-6).
const SSI: [(u32, u8, u8); 2] = [(0xFC0B_C000, 50, 48), (0xFC0C_8000, 54, 52)];
/// DSPI0-DSPI3 (RM Table 40-2).
const DSPI: [u32; 4] = [0xFC05_C000, 0xFC03_C000, 0xEC03_8000, 0xEC03_C000];
/// eSDHC (RM Table 1-3, slot 51), INTC2 source 31 (RM Table 17-17), DMA request 59 (RM Table 19-6).
const ESDHC: u32 = 0xFC0C_C000;
const ESDHC_SOURCE: u8 = 31;
const ESDHC_CH: u8 = 59;
/// USB On-the-Go (RM Table 1-3, slot 44), INTC2 source 17 (RM Table 17-17).
const USB: u32 = 0xFC0B_0000;
const USB_SOURCE: u8 = 17;
/// DDR controller, PLL, NAND flash controller (RM Table 1-3, slots 46, 48, 63).
const DDRMC: u32 = 0xFC0B_8000;
const PLL: u32 = 0xFC0C_0000;
const NFC: u32 = 0xFC0F_C000;
/// The blocks that read back what is written (RM Table 1-3 and Table 1-4).
const STUBS: [(&str, u32); 5] = [
    ("XBS", 0xFC00_4000),
    ("FlexBus", 0xFC00_8000),
    ("SCM", 0xFC04_0000),
    ("CCM", 0xEC09_0000),
    ("GPIO", 0xEC09_4000),
];
/// Every other module of RM Table 1-3 and Table 1-4, and RGPIO (RM Table 1-2, RM ch. 16).
const UNMODELLED: [(&str, u32); 25] = [
    ("FlexCAN 0", 0xFC02_0000),
    ("FlexCAN 1", 0xFC02_4000),
    ("I2C 1", 0xFC03_8000),
    ("I2C 0", 0xFC05_8000),
    ("DMA timer 0", 0xFC07_0000),
    ("DMA timer 1", 0xFC07_4000),
    ("DMA timer 2", 0xFC07_8000),
    ("DMA timer 3", 0xFC07_C000),
    ("Edge port 0", 0xFC09_0000),
    ("ADC", 0xFC09_4000),
    ("DAC 0", 0xFC09_8000),
    ("DAC 1", 0xFC09_C000),
    ("Robust real-time clock", 0xFC0A_8000),
    ("SIM", 0xFC0A_C000),
    ("USB host", 0xFC0B_4000),
    ("RNG", 0xFC0C_4000),
    ("MAC-NET0", 0xFC0D_4000),
    ("MAC-NET1", 0xFC0D_8000),
    ("L2 Ethernet switch 0", 0xFC0D_C000),
    ("L2 Ethernet switch 1", 0xFC0E_0000),
    ("1-Wire", 0xEC00_8000),
    ("I2C 2", 0xEC01_0000),
    ("I2C 3", 0xEC01_4000),
    ("I2C 4", 0xEC01_8000),
    ("I2C 5", 0xEC01_C000),
];
/// The rest of RM Table 1-4, and RGPIO.
const UNMODELLED_MORE: [(&str, u32); 2] = [("mcPWM", 0xEC08_8000), ("RGPIO", 0x8C00_0000)];

/// Test values for the board's side.
const UART_TICKS: [u64; 10] = [100, 101, 102, 103, 104, 105, 106, 107, 108, 109];
const SSI_TICKS: [u64; 2] = [500, 600];
const US: u64 = 100;
const SD_LATENCY: u64 = 1000;
const SD_BUSY: u64 = 2000;
const PLL_CR: u32 = 0x0000_0019;
const PLL_DR: u32 = 0x0043_0C41;
const NOR_DSPI: u8 = 1;
const NOR_PCS: u8 = 2;
const JEDEC: [u8; 6] = [0x01, 0x20, 0x18, 0x4D, 0x01, 0x80];

fn card(image: &[u8]) -> Emmc {
    Emmc::new(
        image.to_vec(),
        EmmcConfig {
            sec_count: 0x8000,
            cid: [1, 0, 0, 0x0015_0000],
            csd: [0x1111_1111, 0x2222_2222, 0x3333_3333, 0x0044_4444],
            ext_csd: vec![(155, 1)],
            ocr: 0xC0FF_8080,
        },
    )
}

fn options() -> ChipOptions {
    ChipOptions {
        nor: Some((NOR_DSPI, NOR_PCS, SpiNor::new(1 << 20, JEDEC, &[]))),
        card: Some(card(&[])),
        nand: Some(NandArray {
            pages: vec![vec![0xA5; 16]],
            bad: Default::default(),
            payload: 16,
        }),
        pll: (PLL_CR, PLL_DR),
        uart_ticks_per_byte: UART_TICKS,
        ssi_ticks_per_frame: SSI_TICKS,
        usb_ticks_per_us: US,
        sd_data_latency: SD_LATENCY,
        sd_cmd6_busy: SD_BUSY,
        gates: Gates::Open,
    }
}

/// A machine with DDR RAM and the chip built with `options`.
fn chip_with(options: ChipOptions) -> (Machine, Chip) {
    let mut m = Machine::new();
    m.mem.map_ram(DDR, 0x0100_0000);
    let chip = Mcf5441x::build(&mut m, options);
    (m, chip)
}

fn built() -> (Machine, Chip) {
    chip_with(options())
}

fn r(m: &mut Machine, addr: u32, size: Size) -> u32 {
    m.access(addr, size, None)
        .unwrap_or_else(|f| panic!("read {addr:#x}: {f:?} {:?}", m.take_refusal()))
}

fn w(m: &mut Machine, addr: u32, size: Size, value: u32) {
    m.access(addr, size, Some(value))
        .unwrap_or_else(|f| panic!("write {addr:#x}: {f:?} {:?}", m.take_refusal()));
}

fn idle(m: &mut Machine, tick: u64) {
    assert_eq!(m.idle_until(tick), None);
}

/// IPRH:IPRL of controller `c`, read at its RM base.
fn ipr(m: &mut Machine, c: usize) -> u64 {
    let hi = r(m, INTC[c], Size::Long);
    let lo = r(m, INTC[c] + 4, Size::Long);
    u64::from(hi) << 32 | u64::from(lo)
}

/// Give source `s` of controller `c` `level` and unmask it (ICRn, CIMR: RM 17.2.9, RM 17.2.6).
fn enable(m: &mut Machine, c: usize, s: u8, level: u32) {
    w(m, INTC[c] + 0x40 + u32::from(s), Size::Byte, level);
    w(m, INTC[c] + 0x1D, Size::Byte, u32::from(s));
}

/// The global level-`level` IACK (RM 17.2.10).
fn gliack(m: &mut Machine, level: u32) -> u32 {
    r(m, GIACK + 0xE0 + 4 * level, Size::Byte)
}

fn refusal(m: &mut Machine, addr: u32) -> String {
    assert_eq!(
        m.access(addr, Size::Long, None),
        Err(Fault::Device(Stop::Unmodelled)),
        "{addr:#x}"
    );
    m.take_refusal().expect("a refusal").detail
}

fn check_intc_and_pits() {
    let (mut m, chip) = built();
    for (i, &id) in chip.intc.iter().enumerate() {
        assert_eq!(m.device::<Intc>(id).index(), i, "INTC{i} added in order");
        assert_eq!(ipr(&mut m, i), 0);
    }
    // CLMASK reads 0x0F in INTC0 (RM 17.2.7); the IACK slot answers with the spurious vector at reset.
    assert_eq!(r(&mut m, INTC[0] + 0x1E, Size::Byte), 0x0F);
    assert_eq!(gliack(&mut m, 6), 0x18);
    for (n, &(base, source)) in PIT.iter().enumerate() {
        m.device::<Pit>(chip.pit[n]);
        w(&mut m, base + 2, Size::Word, 9); // PMR
        w(&mut m, base, Size::Word, 0x000B); // PCSR: PIE RLD EN, PRE 0: a period of 10 ticks
        let t = m.now() + 20;
        idle(&mut m, t);
        assert_ne!(
            ipr(&mut m, 2) & 1 << source,
            0,
            "PIT{n} -> INTC2 source {source}"
        );
        w(&mut m, base, Size::Word, 0x0004); // stop it, clear PIF
        assert_eq!(ipr(&mut m, 2) & 1 << source, 0, "PIT{n}");
    }
    // PIT0 through the global IACK: vector 64 + 2 * 64 + 13 (RM 17.3.1.3)
    enable(&mut m, 2, PIT[0].1, 6);
    w(&mut m, PIT[0].0 + 2, Size::Word, 9);
    w(&mut m, PIT[0].0, Size::Word, 0x000B);
    let t = m.now() + 20;
    idle(&mut m, t);
    assert_eq!(gliack(&mut m, 6), 205);
}

fn check_uarts() {
    for (n, &(base, c, source, ch)) in UART.iter().enumerate() {
        let (mut m, chip) = built();
        m.device::<Uart>(chip.uart[n]);
        w(&mut m, base + 0x08, Size::Byte, 0x01); // UCR: receiver enable
        w(&mut m, base + 0x14, Size::Byte, 0x02); // UIMR: RXRDY
        m.device_mut::<Uart>(chip.uart[n])
            .receive(&[(10, Some(0x50 + n as u8))]);
        idle(&mut m, 10 + UART_TICKS[n] - 1);
        assert_eq!(
            r(&mut m, base + 0x04, Size::Byte) & 0x01,
            0,
            "UART{n} RXRDY early"
        );
        idle(&mut m, 10 + UART_TICKS[n]);
        assert_eq!(
            r(&mut m, base + 0x04, Size::Byte) & 0x01,
            0x01,
            "UART{n} RXRDY"
        );
        assert_eq!(
            ipr(&mut m, c),
            1 << source,
            "UART{n} -> INTC{c} source {source}"
        );
        assert_eq!(
            m.dma_lines().asserted(),
            1 << ch,
            "UART{n} receive -> DMA request {ch}"
        );
        enable(&mut m, c, source, 3);
        assert_eq!(gliack(&mut m, 3), 64 + 64 * c as u32 + u32::from(source));
        assert_eq!(r(&mut m, base + 0x0C, Size::Byte), 0x50 + n as u32);
        assert_eq!(m.dma_lines().asserted(), 0);
        assert_eq!(ipr(&mut m, c), 0);
    }
}

fn check_edma() {
    let (mut m, chip) = built();
    m.device::<Edma>(chip.edma);
    m.mem.load(0x4030_0000, &[0x11, 0x22, 0x33, 0x44]).unwrap();
    let t = EDMA + 0x1000 + 32; // channel 1's TCD
    for (off, size, v) in [
        (0x00, Size::Long, 0x4030_0000), // SADDR
        (0x04, Size::Word, 0x0202),      // ATTR: 32-bit
        (0x06, Size::Word, 4),           // SOFF
        (0x08, Size::Long, 4),           // NBYTES
        (0x10, Size::Long, 0x4030_0100), // DADDR
        (0x14, Size::Word, 1),           // CITER
        (0x16, Size::Word, 4),           // DOFF
        (0x1C, Size::Word, 1),           // BITER
        (0x1E, Size::Word, 0x0003),      // CSR: INTMAJOR START
    ] {
        w(&mut m, t + off, size, v);
    }
    assert_eq!(
        m.mem.peek(0x4030_0100, 4).unwrap(),
        [0x11, 0x22, 0x33, 0x44]
    );
    assert_eq!(ipr(&mut m, 0), 1 << 9, "channel 1 -> INTC0 source 9");
}

fn check_ssis() {
    for (n, &(base, tx, rx)) in SSI.iter().enumerate() {
        let (mut m, chip) = built();
        assert_eq!(
            r(&mut m, base + 0x14, Size::Long),
            0x3003,
            "SSI{n} ISR at reset"
        );
        for (off, v) in [
            (0x10, 0x30),                      // CR: SYN, I2S master
            (0x1C, 0xE8),                      // TCR
            (0x20, 0x480),                     // RCR
            (0x24, 0x16101),                   // CCR
            (0x2C, 0x22),                      // FCSR: watermarks 2
            (0x18, 0x0010_0000 | 0x0040_0000), // IER: TDMAE RDMAE
            (0x10, 0x37),                      // CR: RE TE SSI_EN
        ] {
            w(&mut m, base + off, Size::Long, v);
        }
        assert_eq!(
            m.device::<Ssi>(chip.ssi[n]).next_event(),
            Some(SSI_TICKS[n])
        );
        assert_eq!(
            m.dma_lines().asserted(),
            1 << tx,
            "SSI{n} TX0 -> DMA request {tx}"
        );
        idle(&mut m, SSI_TICKS[n]);
        assert_eq!(
            m.dma_lines().asserted(),
            1 << tx | 1 << rx,
            "SSI{n} RX0 -> DMA request {rx}"
        );
    }
}

fn check_dspis() {
    let (mut m, chip) = built();
    for (n, &base) in DSPI.iter().enumerate() {
        m.device::<Dspi>(chip.dspi[n]);
        assert_eq!(
            r(&mut m, base, Size::Long),
            0x0000_4001,
            "DSPI{n} MCR at reset"
        );
    }
    // The NOR on DSPI1 PCS2: RDID through the registers.
    let base = DSPI[usize::from(NOR_DSPI)];
    w(&mut m, base + 0x0C, Size::Long, 0x3801_0000); // CTAR0: 8-bit frames
    w(&mut m, base, Size::Long, 0x8007_0C00); // MCR: master, PCSIS0-2, clear FIFOs, running
    let pcs = 1 << (16 + u32::from(NOR_PCS));
    for (i, b) in [0x9F, 0, 0, 0].into_iter().enumerate() {
        let tail = if i == 3 { 1 << 27 } else { 1 << 31 }; // EOQ on the last, CONT on the others
        w(&mut m, base + 0x34, Size::Long, pcs | tail | b);
    }
    let got: Vec<u32> = (0..4)
        .map(|_| r(&mut m, base + 0x38, Size::Long) & 0xFF)
        .collect();
    assert_eq!(
        got[1..],
        JEDEC[..3].iter().map(|&b| u32::from(b)).collect::<Vec<_>>()
    );
    let dspi = m.device::<Dspi>(chip.dspi[usize::from(NOR_DSPI)]);
    assert_eq!(dspi.device::<SpiNor>(NOR_PCS).log().len(), 1);
}

fn check_esdhc() {
    let mut image = vec![0u8; 4 * 512];
    for (i, b) in image.iter_mut().enumerate() {
        *b = (i * 7) as u8;
    }
    let mut o = options();
    o.card = Some(card(&image));
    let (mut m, chip) = chip_with(o);
    assert!(m.device::<Esdhc>(chip.esdhc).card().is_some());
    assert_eq!(r(&mut m, ESDHC + 0xFC, Size::Long), 0x1201, "HOSTVER");
    // Channel 59: 32-bit words from DATPORT, 16 bytes a request, into DDR.
    let t = EDMA + 0x1000 + 32 * u32::from(ESDHC_CH);
    for (off, size, v) in [
        (0x00, Size::Long, ESDHC + 0x20),
        (0x04, Size::Word, 0x0202),
        (0x06, Size::Word, 0),
        (0x08, Size::Long, 16),
        (0x0C, Size::Long, 0),
        (0x10, Size::Long, 0x4010_2000),
        (0x16, Size::Word, 4),
        (0x14, Size::Word, 1024 / 16),
        (0x1C, Size::Word, 1024 / 16),
        (0x18, Size::Long, 0),
        (0x1E, Size::Word, 0),
    ] {
        w(&mut m, t + off, size, v);
    }
    w(&mut m, EDMA + 0x18, Size::Byte, u32::from(ESDHC_CH)); // SERQ
    w(&mut m, ESDHC + 0x38, Size::Long, 0x2); // IRQSIGEN: TC
    w(&mut m, ESDHC + 0x04, Size::Long, 2 << 16 | 512); // BLKATTR
    w(&mut m, ESDHC + 0x08, Size::Long, 1); // CMDARG
    w(&mut m, ESDHC + 0x0C, Size::Long, 0x123A_0036); // XFERTYP: CMD18
    let now = m.now();
    assert_eq!(
        m.device::<Esdhc>(chip.esdhc).next_event(),
        Some(now + SD_LATENCY)
    );
    assert_eq!(ipr(&mut m, 2) & 1 << ESDHC_SOURCE, 0);
    idle(&mut m, now + SD_LATENCY);
    assert_eq!(m.mem.peek(0x4010_2000, 1024).unwrap(), &image[512..1536]);
    assert_eq!(
        ipr(&mut m, 2),
        1 << ESDHC_SOURCE,
        "eSDHC -> INTC2 source 31"
    );
}

fn check_usb() {
    let (mut m, chip) = built();
    assert_eq!(r(&mut m, USB, Size::Long), 0xE241_FA05, "ID");
    // What a driver does before RS, then a bus reset: USBSTS[URI] with USBINTR set.
    const QH: u32 = 0x4010_0800;
    const TD: u32 = 0x4010_1000;
    const BUF: u32 = 0x4010_2000;
    w(&mut m, USB + 0x1A8, Size::Long, 0x1E); // USBMODE
    w(&mut m, USB + 0x158, Size::Long, QH); // EPLISTADDR
    m.mem
        .write(QH, Size::Long, 64 << 16 | 1 << 29 | 1 << 15)
        .unwrap();
    m.mem
        .write(QH + 0x40, Size::Long, 64 << 16 | 1 << 29)
        .unwrap();
    w(&mut m, USB + 0x148, Size::Long, 0x157); // USBINTR
    w(&mut m, USB + 0x140, Size::Long, 1); // USBCMD: RS
    m.with_device::<UsbOtg, _>(chip.usb, |u, cx| {
        u.bus_reset_start(cx);
        u.bus_reset_end(cx);
    });
    assert_ne!(
        ipr(&mut m, 2) & 1 << USB_SOURCE,
        0,
        "USB OTG -> INTC2 source 17"
    );
    // A high-speed IN of 512 bytes from DDR never completes (measured on an MCF54415): DDR is the slow memory.
    w(&mut m, USB + 0x1C4, Size::Long, 0x00C8_00C8); // EPCR1: bulk, enabled
    m.mem
        .write(QH + 64 * 3, Size::Long, 512 << 16 | 1 << 29)
        .unwrap();
    m.mem.load(BUF, &[0xA5; 512]).unwrap();
    m.mem.write(TD, Size::Long, 1).unwrap();
    m.mem
        .write(TD + 4, Size::Long, 512 << 16 | 1 << 15 | 0x80)
        .unwrap();
    for k in 0..5 {
        m.mem
            .write(TD + 8 + 4 * k, Size::Long, BUF + 0x1000 * k)
            .unwrap();
    }
    m.mem.write(QH + 64 * 3 + 8, Size::Long, TD).unwrap();
    w(&mut m, USB + 0x1B0, Size::Long, 1 << 17); // EPPRIME: EP1 IN
    let token = m.with_device::<UsbOtg, _>(chip.usb, |u, cx| u.in_token(1, 0, cx));
    assert_eq!(token, mcf5441x_emu::soc::Token::Nak);
    assert_eq!(m.device::<UsbOtg>(chip.usb).stats().underflows, 1);
}

fn check_ddrmc_pll_nfc() {
    let (mut m, chip) = built();
    m.device::<Ddrmc>(chip.ddrmc);
    assert_eq!(r(&mut m, DDRMC + 0x0C, Size::Long), 0x100, "DDR_CR03");
    m.device::<Pll>(chip.pll);
    assert_eq!(r(&mut m, PLL, Size::Long), PLL_CR);
    assert_eq!(r(&mut m, PLL + 4, Size::Long), PLL_DR);
    assert_eq!(r(&mut m, PLL + 8, Size::Long), 0x30, "PLL_SR LOCK LOCKS");
    let nfc = chip.nfc.expect("an NFC with a NAND array");
    assert_eq!(m.device::<Nfc>(nfc).array().payload, 16);
    assert_eq!(
        r(&mut m, NFC + 0x3F00 + 0x30, Size::Long),
        0x000E_A631,
        "NFC_CFG"
    );
}

#[test]
fn chip_map_matches_the_rm() {
    check_intc_and_pits();
    check_uarts();
    check_edma();
    check_ssis();
    check_dspis();
    check_esdhc();
    check_usb();
    check_ddrmc_pll_nfc();
}

#[test]
fn unmodelled_module_access_names_the_module() {
    let (mut m, _) = built();
    for (name, base) in UNMODELLED.into_iter().chain(UNMODELLED_MORE) {
        assert_eq!(refusal(&mut m, base), format!("{name} +0x0"));
        assert_eq!(refusal(&mut m, base + 0xC), format!("{name} +0xc"));
    }
    // the end of each 16 KB slot (RM 1.8.1)
    for (name, base) in UNMODELLED {
        assert_eq!(refusal(&mut m, base + 0x3FFC), format!("{name} +0x3ffc"));
    }
    // without a NAND array the NFC is not modelled either
    let mut o = options();
    o.nand = None;
    let (mut m, chip) = chip_with(o);
    assert_eq!(chip.nfc, None);
    assert!(refusal(&mut m, NFC + 0x3F30).starts_with("NAND flash controller"));
    // and a program that reaches one stops on that instruction
    let (mut m, _) = built();
    // move.l 0xFC020000,%d0
    m.mem
        .load(0x4010_0000, &[0x20, 0x39, 0xFC, 0x02, 0x00, 0x00])
        .unwrap();
    m.cpu.pc = 0x4010_0000;
    let stop = m.run(Limit {
        max_instructions: Some(10),
        ..Limit::default()
    });
    assert_eq!(stop.kind, Stop::Unmodelled);
    assert_eq!(stop.pc, 0x4010_0000);
    assert_eq!(stop.detail, "FlexCAN 0 +0x0");
}

#[test]
fn stub_blocks_read_back() {
    let (mut m, chip) = built();
    let names: Vec<&str> = chip.stubs.iter().map(|&(n, _)| n).collect();
    assert_eq!(names, STUBS.map(|(n, _)| n));
    for ((name, base), &(_, id)) in STUBS.into_iter().zip(&chip.stubs) {
        assert_eq!(m.device::<StubBlock>(id).name(), name);
        assert_eq!(r(&mut m, base + 0x10, Size::Long), 0, "{name}");
        w(&mut m, base + 0x10, Size::Long, 0x1234_5678);
        assert_eq!(r(&mut m, base + 0x10, Size::Long), 0x1234_5678, "{name}");
        w(&mut m, base + 0x3FFE, Size::Word, 0xBEEF);
        assert_eq!(r(&mut m, base + 0x3FFE, Size::Word), 0xBEEF, "{name}");
        assert_eq!(m.device::<StubBlock>(id).writes().len(), 2, "{name}");
    }
}

/// Load `src` at 0x4010_0000 into the built chip with VBR at 0x4020_0000 and A7 at the top of DDR; returns the
/// machine, the chip and the addresses of `labels`.
fn program(src: &str, labels: &[&str]) -> (Machine, Chip, Vec<u32>) {
    let tail: String = labels.iter().map(|l| format!(".long {l}\n")).collect();
    let code = common::assemble(&format!("{src}\n.balign 2\n{tail}"), common::BASE);
    let (code, tail) = code.split_at(code.len() - 4 * labels.len());
    let addrs = tail
        .chunks(4)
        .map(|c| u32::from_be_bytes(c.try_into().unwrap()))
        .collect();
    let (mut m, chip) = built();
    m.mem.load(common::BASE, code).unwrap();
    m.cpu.pc = common::BASE;
    m.cpu.a[7] = common::STACK;
    m.cpu.vbr = common::VBR;
    (m, chip, addrs)
}

#[test]
fn pitstop_runs_on_the_built_chip() {
    let (mut m, chip, l) = program(include_str!("fixtures/pitstop.s"), &["done", "pit_isr"]);
    w(&mut m, UART[9].0 + 0x08, Size::Byte, 0x04); // UART9 UCR: transmitter enable
    common::set_vector(&mut m, 205, l[1]);
    let r = m.run(common::until(l[0]));
    assert_eq!(r.kind, Stop::UntilPc, "{r:?}");
    assert_eq!(m.mem.read(0x4010_2000, Size::Long), Ok(5));
    // Five 132000-tick periods, passed in STOP rather than in executed code.
    assert!((5 * 132_000..6 * 132_000).contains(&m.now()), "{}", m.now());
    assert!(m.instructions() < m.now() / 100, "{}", m.instructions());
    assert_eq!(m.cpu.a[7], common::STACK);
    let t = m.now() + 4 * UART_TICKS[9];
    idle(&mut m, t);
    let uart = m.device::<Uart>(chip.uart[9]);
    assert_eq!(uart.text(), "ok");
    let stamps: Vec<u64> = uart.sent().iter().map(|&(t, _)| t).collect();
    assert!(stamps[1] - stamps[0] >= UART_TICKS[9], "{stamps:?}");
}

#[test]
fn uart_rx_through_edma_on_the_built_chip() {
    // UART8's receive request (DMA request 34) has the eDMA move each character into a ring in DDR; channel 34
    // interrupts on INTC1 source 26 (RM Table 17-16).
    const RING: u32 = 0x4010_0400;
    let (base, _, _, ch) = UART[8];
    let (mut m, chip) = built();
    w(&mut m, base + 0x08, Size::Byte, 0x01); // UCR: receiver enable
    let t = EDMA + 0x1000 + 32 * u32::from(ch);
    for (off, size, v) in [
        (0x00, Size::Long, base + 0x0C), // SADDR: URB
        (0x04, Size::Word, 10 << 3),     // ATTR: DMOD 10, 8-bit
        (0x08, Size::Long, 1),           // NBYTES
        (0x10, Size::Long, RING),        // DADDR
        (0x16, Size::Word, 1),           // DOFF
        (0x14, Size::Word, 4),           // CITER
        (0x1C, Size::Word, 4),           // BITER
        (0x1E, Size::Word, 0x2),         // CSR: INTMAJOR
    ] {
        w(&mut m, t + off, size, v);
    }
    w(&mut m, EDMA + 0x18, Size::Byte, u32::from(ch)); // SERQ
    let tick = UART_TICKS[8];
    m.device_mut::<Uart>(chip.uart[8]).receive(&[
        (0, Some(b'a')),
        (tick, Some(b'b')),
        (2 * tick, Some(b'c')),
        (3 * tick, Some(b'd')),
    ]);
    idle(&mut m, 3 * tick);
    assert_eq!(m.mem.peek(RING, 3).unwrap(), b"abc");
    assert_eq!(ipr(&mut m, 1), 0, "major loop not done");
    idle(&mut m, 4 * tick);
    assert_eq!(m.mem.peek(RING, 4).unwrap(), b"abcd");
    assert_eq!(ipr(&mut m, 1), 1 << 26, "channel 34 -> INTC1 source 26");
    assert_eq!(
        r(&mut m, base + 0x04, Size::Byte) & 0x11,
        0,
        "no RXRDY, no OE"
    );
    assert_eq!(m.device::<Uart>(chip.uart[8]).lost(), 0);
}

#[test]
fn serq_on_a_channel_nothing_drives_is_refused() {
    // UART transmit requests (RM Table 19-6: channel 3 is UART0 transmit) are never driven by the UART model:
    // enabling one is refused, the channel named. A line a model drives, and a channel RM Table 19-6 lists as
    // "available for software" (30, 31, 42, 43, 46, 47), take SERQ.
    let (mut m, _) = built();
    assert_eq!(
        m.access(EDMA + 0x18, Size::Byte, Some(3)),
        Err(Fault::Device(Stop::Unmodelled))
    );
    let detail = m.take_refusal().expect("a refusal").detail;
    assert_eq!(
        detail,
        "eDMA: channel 3: its peripheral request is not modelled"
    );
    let (_, _, _, rx) = UART[0];
    let driven = [rx, SSI[0].1, SSI[0].2, SSI[1].1, SSI[1].2, ESDHC_CH];
    for ch in driven.into_iter().chain([30, 31, 42, 43, 46, 47]) {
        w(&mut m, EDMA + 0x18, Size::Byte, u32::from(ch));
    }
    for ch in UART.map(|u| u.3) {
        w(&mut m, EDMA + 0x18, Size::Byte, u32::from(ch));
    }
    for ch in [0, 12, 13, 39, 56, 58, 63] {
        assert_eq!(
            m.access(EDMA + 0x18, Size::Byte, Some(ch)),
            Err(Fault::Device(Stop::Unmodelled)),
            "channel {ch}"
        );
        let d = m.take_refusal().expect("a refusal").detail;
        assert!(d.contains(&format!("channel {ch}:")), "{d}");
    }
}

#[test]
fn chip_options_default_builds_a_chip() {
    let o = ChipOptions::default();
    assert!(o.nor.is_none() && o.card.is_none() && o.nand.is_none());
    // RM Figures 8-3 and 8-4: LOLEN, FBKDIV 9; OUTDIV5 0x1F, OUTDIV4 7, OUTDIV3 9, OUTDIV2 3, OUTDIV1 1
    assert_eq!(o.pll, (0x1009, 0x03E7_2461));
    assert_eq!(o.uart_ticks_per_byte, [0; 10]);
    assert_eq!(o.ssi_ticks_per_frame, [1, 1]);
    assert_eq!(o.usb_ticks_per_us, 1);
    assert_eq!((o.sd_data_latency, o.sd_cmd6_busy), (1, 1));
    let (mut m, chip) = chip_with(o);
    assert_eq!(r(&mut m, PLL, Size::Long), 0x1009);
    assert_eq!(r(&mut m, PLL + 4, Size::Long), 0x03E7_2461);
    assert!(chip.nor(&m).is_none() && chip.card(&m).is_none() && chip.nfc.is_none());
}

#[test]
fn chip_options_builders_set_their_fields() {
    let o = ChipOptions::default()
        .with_nor(2, 3, SpiNor::new(1 << 20, JEDEC, &[]))
        .with_card(card(&[]))
        .with_nand(NandArray {
            pages: vec![vec![0; 16]],
            bad: Default::default(),
            payload: 16,
        })
        .with_pll(5, 6)
        .with_uart_ticks(4, 77)
        .with_ssi_ticks(1, 9)
        .with_usb_ticks_per_us(33)
        .with_sd_timing(11, 22);
    assert_eq!(o.nor.as_ref().map(|n| (n.0, n.1)), Some((2, 3)));
    assert!(o.card.is_some() && o.nand.is_some());
    assert_eq!(o.pll, (5, 6));
    assert_eq!(o.uart_ticks_per_byte[4], 77);
    assert_eq!(o.uart_ticks_per_byte[3], 0);
    assert_eq!(o.ssi_ticks_per_frame, [1, 9]);
    assert_eq!(o.usb_ticks_per_us, 33);
    assert_eq!((o.sd_data_latency, o.sd_cmd6_busy), (11, 22));
}

#[test]
fn chip_nor_and_card_accessors() {
    let (mut m, chip) = built();
    assert_eq!(chip.nor(&m).unwrap().sector_size(), 0x10000);
    assert!(chip.card(&m).is_some());
    chip.nor_mut(&mut m).unwrap().faults();
    assert_eq!(*chip.card_mut(&mut m).unwrap().faults(), 0);
    let nor = m
        .device::<Dspi>(chip.dspi[usize::from(NOR_DSPI)])
        .device::<SpiNor>(NOR_PCS);
    assert!(std::ptr::eq(nor, chip.nor(&m).unwrap()));
}
