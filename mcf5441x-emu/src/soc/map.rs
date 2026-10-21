//! The chip map: every module of the MCF5441x at its RM base address (RM Table 1-3, Table 1-4), wired to its
//! interrupt sources (RM Table 17-15, Table 17-16, Table 17-17) and DMA request lines (RM Table 19-6).

use crate::device::DeviceId;
use crate::machine::Machine;
use crate::parts::{Emmc, SpiNor};

use super::{
    Ddrmc, Dspi, Edma, Esdhc, EsdhcConfig, Intc, NandArray, Nfc, Pit, Pll, Ssi, SsiConfig,
    StubBlock, StubMode, Uart, UartConfig, Unmodelled, UsbConfig, UsbOtg,
};

/// Each peripheral slot is 16 KB (RM 1.8.1).
const SLOT: u32 = 0x4000;

/// INTC0, INTC1, INTC2 and the global IACK registers (RM Table 1-3).
const INTC: [u32; 3] = [0xFC04_8000, 0xFC04_C000, 0xFC05_0000];
const IACK: u32 = 0xFC05_4000;
/// The eDMA's registers and TCDs (RM ch. 19) take the first 8 KB of its slot.
const EDMA: (u32, u32) = (0xFC04_4000, 0x2000);
/// PIT0-PIT3 (RM Table 1-3): INTC2 sources 13-16 (RM Table 17-17).
const PIT: u32 = 0xFC08_0000;
const PIT_SOURCE: u8 = 13;
/// UART0-UART9 (RM Table 1-3, Table 1-4): (base, controller, source, receive DMA request), RM Table 17-15,
/// Table 17-16 and Table 19-6.
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
/// SSI0 and SSI1 (RM Table 1-3): (base, transmit 0 request, receive 0 request), RM Table 19-6.
const SSI: [(u32, u8, u8); 2] = [(0xFC0B_C000, 50, 48), (0xFC0C_8000, 54, 52)];
/// DSPI0-DSPI3 (RM Table 40-2).
const DSPI: [u32; 4] = [0xFC05_C000, 0xFC03_C000, 0xEC03_8000, 0xEC03_C000];
/// The eSDHC (RM Table 1-3): INTC2 source 31 (RM Table 17-17), DMA request 59 (RM Table 19-6).
const ESDHC: u32 = 0xFC0C_C000;
const ESDHC_SOURCE: u8 = 31;
const ESDHC_DMA: u8 = 59;
/// The USB OTG controller (RM Table 1-3; its registers end below +0x1000, RM ch. 24): INTC2 source 17 (RM
/// Table 17-17).
const USB: (u32, u32) = (0xFC0B_0000, 0x1000);
const USB_SOURCE: u8 = 17;
/// The DDR window (RM Table 1-2): a high-speed IN of more than 64 bytes from it never completes, measured on an
/// MCF54415.
const DDR_WINDOW: std::ops::Range<u32> = 0x4000_0000..0x8000_0000;
/// The DDR controller, the PLL and the NFC (RM Table 1-3).
const DDRMC: u32 = 0xFC0B_8000;
const PLL: u32 = 0xFC0C_0000;
const NFC: u32 = 0xFC0F_C000;

/// The blocks mapped as [`StubBlock`]s in [`StubMode::ReadBack`] (RM Table 1-3, Table 1-4).
const STUBS: [(&str, u32); 5] = [
    ("XBS", 0xFC00_4000),
    ("FlexBus", 0xFC00_8000),
    ("SCM", 0xFC04_0000),
    ("CCM", 0xEC09_0000),
    ("GPIO", 0xEC09_4000),
];

/// The modules of RM Table 1-3 and Table 1-4 nobody models, each over its 16 KB slot.
const UNMODELLED: [(&str, u32); 26] = [
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
    ("mcPWM", 0xEC08_8000),
];
/// Rapid GPIO: 0x8C00_0000-0x8FFF_FFFF (RM Table 1-2, RM ch. 16).
const RGPIO: (u32, u32) = (0x8C00_0000, 0x0400_0000);

/// What the board decides about the chip: the parts outside it and the time scale of its clocks, in ticks of the
/// machine clock (one instruction is one tick).
pub struct ChipOptions {
    /// The SPI NOR flash and where it sits: (DSPI 0-3, PCS 0-7, the part). `None`: nothing on any DSPI.
    pub nor: Option<(u8, u8, SpiNor)>,
    /// The eMMC behind the eSDHC. `None`: no card (commands time out).
    pub card: Option<Emmc>,
    /// The NAND flash behind the NFC. `None`: the NFC is not mapped as a model, and an access to it stops the run
    /// with [`Stop::Unmodelled`](crate::Stop::Unmodelled).
    pub nand: Option<NandArray>,
    /// PLL_CR and PLL_DR as the reset configuration (or the boot stage before) left them (see [`Pll::new`]).
    pub pll: (u32, u32),
    /// Each UART's character time ([`UartConfig::ticks_per_byte`]), UART0 to UART9.
    pub uart_ticks_per_byte: [u64; 10],
    /// Each SSI's frame period ([`SsiConfig::ticks_per_frame`]), SSI0 and SSI1. At least 1.
    pub ssi_ticks_per_frame: [u64; 2],
    /// The USB controller's time scale ([`UsbConfig::ticks_per_us`]).
    pub usb_ticks_per_us: u64,
    /// The eSDHC's command-to-data delay ([`EsdhcConfig::data_latency`]).
    pub sd_data_latency: u64,
    /// How long the card holds DAT0 busy after a CMD6 ([`EsdhcConfig::cmd6_busy`]).
    pub sd_cmd6_busy: u64,
}

/// The devices [`Mcf5441x::build`] added, for [`Machine::device`] and [`Machine::with_device`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Chip {
    /// INTC0, INTC1, INTC2 ([`Intc`]).
    pub intc: [DeviceId; 3],
    /// The global IACK registers ([`GlobalIack`](super::GlobalIack)).
    pub iack: DeviceId,
    /// PIT0-PIT3 ([`Pit`]).
    pub pit: [DeviceId; 4],
    /// The eDMA ([`Edma`]).
    pub edma: DeviceId,
    /// UART0-UART9 ([`Uart`]).
    pub uart: [DeviceId; 10],
    /// SSI0 and SSI1 ([`Ssi`]).
    pub ssi: [DeviceId; 2],
    /// DSPI0-DSPI3 ([`Dspi`]).
    pub dspi: [DeviceId; 4],
    /// The eSDHC ([`Esdhc`]).
    pub esdhc: DeviceId,
    /// The USB OTG controller ([`UsbOtg`]).
    pub usb: DeviceId,
    /// The DDR controller ([`Ddrmc`]).
    pub ddrmc: DeviceId,
    /// The PLL ([`Pll`]).
    pub pll: DeviceId,
    /// The NFC ([`Nfc`]), when [`ChipOptions::nand`] gave it an array.
    pub nfc: Option<DeviceId>,
    /// The [`StubBlock`]s, by name: XBS, FlexBus, SCM, CCM, GPIO.
    pub stubs: Vec<(&'static str, DeviceId)>,
}

/// The MCF5441x's on-chip modules, mapped into a [`Machine`].
///
/// | Base | Size | Block | Interrupt (controller, source) | DMA requests |
/// |---|---|---|---|---|
/// | 0xFC04_8000, 0xFC04_C000, 0xFC05_0000 | 0x4000 | INTC0-2 ([`Intc::chip`], added in that order) | | |
/// | 0xFC05_4000 | 0x4000 | global IACK | | |
/// | 0xFC04_4000 | 0x2000 | eDMA (bus master) | its own routes, see [`Edma`] | |
/// | 0xFC08_0000 + 0x4000 n | 0x4000 | PIT0-3 | INTC2 13-16 | |
/// | 0xFC06_0000 + 0x4000 n | 0x4000 | UART0-3 | INTC0 26-29 | receive 2, 4, 6, 16 |
/// | 0xEC06_0000 + 0x4000 n | 0x4000 | UART4-7 | INTC1 48-51 | receive 18, 20, 22, 32 |
/// | 0xEC07_0000, 0xEC07_4000 | 0x4000 | UART8, UART9 | INTC1 52, 53 | receive 34, 36 |
/// | 0xFC0B_C000, 0xFC0C_8000 | 0x4000 | SSI0, SSI1 | | TX0 50, 54; RX0 48, 52 |
/// | 0xFC05_C000, 0xFC03_C000, 0xEC03_8000, 0xEC03_C000 | 0x4000 | DSPI0-3 | | |
/// | 0xFC0C_C000 | 0x4000 | eSDHC | INTC2 31 | 59 |
/// | 0xFC0B_0000 | 0x1000 | USB OTG (bus master) | INTC2 17 | |
/// | 0xFC0B_8000 | 0x4000 | DDR controller | | |
/// | 0xFC0C_0000 | 0x4000 | PLL | | |
/// | 0xFC0F_C000 | 0x4000 | NFC (with a NAND array) | | |
///
/// The stub blocks (read back what is written): XBS 0xFC00_4000, FlexBus 0xFC00_8000, SCM 0xFC04_0000, CCM
/// 0xEC09_0000 (with the reset controller and power management), GPIO 0xEC09_4000 (pin multiplexing and
/// control). Every other module of RM Table 1-3 and Table 1-4 is an [`Unmodelled`] block over its 16 KB slot,
/// named as the table names it, and so is RGPIO at 0x8C00_0000-0x8FFF_FFFF: an access stops the run with
/// [`Stop::Unmodelled`](crate::Stop::Unmodelled) and the detail `"<module> +<offset>"`.
///
/// The interrupt sources the models leave unwired (SSI, DSPI, NFC, the DDR controller and the PLL raise no
/// interrupts; their enables are refused, see each model) are not driven.
///
/// Not mapped: the SRAM (the board maps it where its RAMBAR puts it, RM ch. 7), the DDR (its size is the
/// board's), the FlexBus chip-select windows, and the reserved slots, which fault as unmapped.
pub struct Mcf5441x;

impl Mcf5441x {
    /// Map every module into `m` with the board's `options`; returns their ids.
    ///
    /// # Panics
    /// When a window overlaps RAM or a device `m` already has, when [`ChipOptions::nor`] names a DSPI above 3 or
    /// a PCS above 7, or when a model's constructor rejects an option (an SSI frame period of 0, a NAND payload
    /// larger than an NFC buffer).
    pub fn build(m: &mut Machine, options: ChipOptions) -> Chip {
        let ([intc0, intc1, intc2], iack) = Intc::chip();
        // In this order: the controllers' indices are the order they are added (RM 17.3.2).
        let intc = [
            m.add_interrupt_controller(INTC[0], SLOT, Box::new(intc0)),
            m.add_interrupt_controller(INTC[1], SLOT, Box::new(intc1)),
            m.add_interrupt_controller(INTC[2], SLOT, Box::new(intc2)),
        ];
        let iack = m.add_device(IACK, SLOT, Box::new(iack));

        let edma = m.add_bus_master(EDMA.0, EDMA.1, Box::new(Edma::new()));

        let pit = [0u8, 1, 2, 3].map(|n| {
            let base = PIT + SLOT * u32::from(n);
            m.add_device(base, SLOT, Box::new(Pit::new(2, PIT_SOURCE + n)))
        });

        let uart = std::array::from_fn(|n| {
            let (base, controller, source, rx) = UART[n];
            let config = UartConfig {
                controller,
                source,
                rx_dma: Some(rx),
                ticks_per_byte: options.uart_ticks_per_byte[n],
            };
            m.add_device(base, SLOT, Box::new(Uart::new(config)))
        });

        let ssi = std::array::from_fn(|n| {
            let (base, tx_dma, rx_dma) = SSI[n];
            let config = SsiConfig {
                tx_dma,
                rx_dma,
                ticks_per_frame: options.ssi_ticks_per_frame[n],
            };
            m.add_device(base, SLOT, Box::new(Ssi::new(config)))
        });

        let mut dspis = [(); 4].map(|_| Dspi::new());
        if let Some((d, pcs, nor)) = options.nor {
            assert!(d < 4, "no DSPI{d}: DSPI0-DSPI3 only");
            dspis[usize::from(d)].attach(pcs, Box::new(nor));
        }
        let mut dspis = dspis.into_iter();
        let dspi = DSPI.map(|base| m.add_device(base, SLOT, Box::new(dspis.next().unwrap())));

        let esdhc = m.add_device(
            ESDHC,
            SLOT,
            Box::new(Esdhc::new(
                EsdhcConfig {
                    controller: 2,
                    source: ESDHC_SOURCE,
                    dma: ESDHC_DMA,
                    data_latency: options.sd_data_latency,
                    cmd6_busy: options.sd_cmd6_busy,
                },
                options.card,
            )),
        );

        let usb = m.add_bus_master(
            USB.0,
            USB.1,
            Box::new(UsbOtg::new(UsbConfig {
                controller: 2,
                source: USB_SOURCE,
                ticks_per_us: options.usb_ticks_per_us,
                slow_memory: Some(DDR_WINDOW),
            })),
        );

        let ddrmc = m.add_device(DDRMC, SLOT, Box::new(Ddrmc::new()));
        let (cr, dr) = options.pll;
        let pll = m.add_device(PLL, SLOT, Box::new(Pll::new(cr, dr)));
        let nfc = match options.nand {
            Some(array) => Some(m.add_device(NFC, SLOT, Box::new(Nfc::new(array)))),
            None => {
                let block = Unmodelled::new("NAND flash controller (no NAND array)");
                m.add_device(NFC, SLOT, Box::new(block));
                None
            }
        };

        let stubs = STUBS
            .iter()
            .map(|&(name, base)| {
                let block = StubBlock::new(name, StubMode::ReadBack);
                (name, m.add_device(base, SLOT, Box::new(block)))
            })
            .collect();
        for (name, base) in UNMODELLED {
            m.add_device(base, SLOT, Box::new(Unmodelled::new(name)));
        }
        m.add_device(RGPIO.0, RGPIO.1, Box::new(Unmodelled::new("RGPIO")));

        Chip {
            intc,
            iack,
            pit,
            edma,
            uart,
            ssi,
            dspi,
            esdhc,
            usb,
            ddrmc,
            pll,
            nfc,
            stubs,
        }
    }
}
