//! The chip map: every module of the MCF5441x at its RM base address (RM Table 1-3, Table 1-4), wired to its
//! interrupt sources (RM Table 17-15, Table 17-16, Table 17-17) and DMA request lines (RM Table 19-6).

use crate::device::DeviceId;
use crate::machine::Machine;
use crate::parts::{Emmc, SpiNor};

use super::scm::{self, POWER_MANAGEMENT};
use super::{
    Ddrmc, Dspi, Dtim, Edma, Eport, Esdhc, EsdhcConfig, GpioPorts, Intc, NandArray, Nfc, Pit, Pll,
    PowerManagement, Ssi, SsiConfig, StubBlock, StubMode, Uart, UartConfig, Unmodelled, UsbConfig,
    UsbOtg,
};

/// Each peripheral slot is 16 KB (RM 1.8.1).
const SLOT: u32 = 0x4000;

/// INTC0, INTC1, INTC2 and the global IACK registers (RM Table 1-3).
const INTC: [u32; 3] = [0xFC04_8000, 0xFC04_C000, 0xFC05_0000];
const IACK: u32 = 0xFC05_4000;
/// The eDMA's registers and TCDs (RM ch. 19) take the first 8 KB of its slot.
const EDMA: (u32, u32) = (0xFC04_4000, 0x2000);
/// The eDMA channels RM Table 19-6 lists as "available for software": no peripheral drives them.
const SOFTWARE_DMA: [u8; 6] = [30, 31, 42, 43, 46, 47];
/// PIT0-PIT3 (RM Table 1-3): INTC2 sources 13-16 (RM Table 17-17).
const PIT: u32 = 0xFC08_0000;
const PIT_SOURCE: u8 = 13;
/// DMA timers 0-3 (RM Table 1-3): INTC0 sources 32-35 (RM Table 17-15).
const DTIM: u32 = 0xFC07_0000;
const DTIM_SOURCE: u8 = 32;
/// Edge port 0 (RM Table 1-3): pin IRQn drives INTC0 source n, 1-7 (RM Table 17-15).
const EPORT: u32 = 0xFC09_0000;
/// UART0-UART9 (RM Table 1-3, Table 1-4): (base, controller, source, receive DMA request, transmit DMA request),
/// RM Table 17-15, Table 17-16 and Table 19-6.
const UART: [(u32, usize, u8, u8, u8); 10] = [
    (0xFC06_0000, 0, 26, 2, 3),
    (0xFC06_4000, 0, 27, 4, 5),
    (0xFC06_8000, 0, 28, 6, 7),
    (0xFC06_C000, 0, 29, 16, 17),
    (0xEC06_0000, 1, 48, 18, 19),
    (0xEC06_4000, 1, 49, 20, 21),
    (0xEC06_8000, 1, 50, 22, 23),
    (0xEC06_C000, 1, 51, 32, 33),
    (0xEC07_0000, 1, 52, 34, 35),
    (0xEC07_4000, 1, 53, 36, 37),
];
/// SSI0 and SSI1 (RM Table 1-3): (base, transmit 0 request, receive 0 request), RM Table 19-6.
const SSI: [(u32, u8, u8); 2] = [(0xFC0B_C000, 50, 48), (0xFC0C_8000, 54, 52)];
/// DSPI0-DSPI3 (RM Table 40-2): (base, receive request RFDF, transmit request TFFF), RM Table 19-6.
const DSPI: [(u32, u8, u8); 4] = [
    (0xFC05_C000, 12, 13),
    (0xFC03_C000, 14, 15),
    (0xEC03_8000, 28, 29),
    (0xEC03_C000, 44, 45),
];
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
const UNMODELLED: [(&str, u32); 21] = [
    ("FlexCAN 0", 0xFC02_0000),
    ("FlexCAN 1", 0xFC02_4000),
    ("I2C 1", 0xFC03_8000),
    ("I2C 0", 0xFC05_8000),
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

/// The module of each clock gate: (module, base, gate), gate `64 s + n` for CDn of PPM{H,L}R`s`. CDn is the
/// module's slot number n (RM 9.2.4): slot n of 0xFC00_0000 for PPM{H,L}R0, of 0xEC00_0000 for PPM{H,L}R1. The
/// SCM, the crossbar switch and the global IACK slot have no gate; neither has RGPIO, outside the slots.
#[rustfmt::skip]
const CLOCK_GATES: [(&str, u32, u8); 58] = [
    ("FlexBus", 0xFC00_8000, 2),               // RM Table 9-6, CD2
    ("FlexCAN 0", 0xFC02_0000, 8),             // RM Table 9-6, CD8
    ("FlexCAN 1", 0xFC02_4000, 9),             // RM Table 9-6, CD9
    ("I2C 1", 0xFC03_8000, 14),                // RM Table 9-6, CD14
    ("DSPI1", 0xFC03_C000, 15),                // RM Table 9-6, CD15
    ("eDMA", 0xFC04_4000, 17),                 // RM Table 9-6, CD17
    ("INTC0", 0xFC04_8000, 18),                // RM Table 9-6, CD18
    ("INTC1", 0xFC04_C000, 19),                // RM Table 9-6, CD19
    ("INTC2", 0xFC05_0000, 20),                // RM Table 9-6, CD20
    ("I2C 0", 0xFC05_8000, 22),                // RM Table 9-6, CD22
    ("DSPI0", 0xFC05_C000, 23),                // RM Table 9-6, CD23
    ("UART0", 0xFC06_0000, 24),                // RM Table 9-6, CD24
    ("UART1", 0xFC06_4000, 25),                // RM Table 9-6, CD25
    ("UART2", 0xFC06_8000, 26),                // RM Table 9-6, CD26
    ("UART3", 0xFC06_C000, 27),                // RM Table 9-6, CD27
    ("DMA timer 0", 0xFC07_0000, 28),          // RM Table 9-6, CD28
    ("DMA timer 1", 0xFC07_4000, 29),          // RM Table 9-6, CD29
    ("DMA timer 2", 0xFC07_8000, 30),          // RM Table 9-6, CD30
    ("DMA timer 3", 0xFC07_C000, 31),          // RM Table 9-6, CD31
    ("PIT0", 0xFC08_0000, 32),                 // RM Table 9-5, CD32
    ("PIT1", 0xFC08_4000, 33),                 // RM Table 9-5, CD33
    ("PIT2", 0xFC08_8000, 34),                 // RM Table 9-5, CD34
    ("PIT3", 0xFC08_C000, 35),                 // RM Table 9-5, CD35
    ("Edge port 0", 0xFC09_0000, 36),          // RM Table 9-5, CD36
    ("ADC", 0xFC09_4000, 37),                  // RM Table 9-5, CD37
    ("DAC 0", 0xFC09_8000, 38),                // RM Table 9-5, CD38
    ("DAC 1", 0xFC09_C000, 39),                // RM Table 9-5, CD39
    ("Robust real-time clock", 0xFC0A_8000, 42), // RM Table 9-5, CD42
    ("SIM", 0xFC0A_C000, 43),                  // RM Table 9-5, CD43
    ("USB OTG", 0xFC0B_0000, 44),              // RM Table 9-5, CD44
    ("USB host", 0xFC0B_4000, 45),             // RM Table 9-5, CD45
    ("DDR controller", 0xFC0B_8000, 46),       // RM Table 9-5, CD46
    ("SSI0", 0xFC0B_C000, 47),                 // RM Table 9-5, CD47
    ("PLL", 0xFC0C_0000, 48),                  // RM Table 9-5, CD48
    ("RNG", 0xFC0C_4000, 49),                  // RM Table 9-5, CD49
    ("SSI1", 0xFC0C_8000, 50),                 // RM Table 9-5, CD50
    ("eSDHC", 0xFC0C_C000, 51),                // RM Table 9-5, CD51
    ("MAC-NET0", 0xFC0D_4000, 53),             // RM Table 9-5, CD53
    ("MAC-NET1", 0xFC0D_8000, 54),             // RM Table 9-5, CD54
    ("L2 Ethernet switch 0", 0xFC0D_C000, 55), // RM Table 9-5, CD55
    ("L2 Ethernet switch 1", 0xFC0E_0000, 56), // RM Table 9-5, CD56
    ("NFC", 0xFC0F_C000, 63),                  // RM Table 9-5, CD63
    ("1-Wire", 0xEC00_8000, 64 + 2),           // RM Table 9-8, CD2
    ("I2C 2", 0xEC01_0000, 64 + 4),            // RM Table 9-8, CD4
    ("I2C 3", 0xEC01_4000, 64 + 5),            // RM Table 9-8, CD5
    ("I2C 4", 0xEC01_8000, 64 + 6),            // RM Table 9-8, CD6
    ("I2C 5", 0xEC01_C000, 64 + 7),            // RM Table 9-8, CD7
    ("DSPI2", 0xEC03_8000, 64 + 14),           // RM Table 9-8, CD14
    ("DSPI3", 0xEC03_C000, 64 + 15),           // RM Table 9-8, CD15
    ("UART4", 0xEC06_0000, 64 + 24),           // RM Table 9-8, CD24
    ("UART5", 0xEC06_4000, 64 + 25),           // RM Table 9-8, CD25
    ("UART6", 0xEC06_8000, 64 + 26),           // RM Table 9-8, CD26
    ("UART7", 0xEC06_C000, 64 + 27),           // RM Table 9-8, CD27
    ("UART8", 0xEC07_0000, 64 + 28),           // RM Table 9-8, CD28
    ("UART9", 0xEC07_4000, 64 + 29),           // RM Table 9-8, CD29
    ("mcPWM", 0xEC08_8000, 64 + 34),           // RM Table 9-7, CD34
    ("CCM", 0xEC09_0000, 64 + 36),             // RM Table 9-7, CD36
    ("GPIO", 0xEC09_4000, 64 + 37),            // RM Table 9-7, CD37
];

/// The clock gates after a soft reset, measured on an MCF54415: PPMHR0, PPMLR0, PPMHR1, PPMLR1. RM Table 9-1
/// gives PPMHR0 0x7E1E_FBFF: the chip also has CD56 and CD55 set. A power-on reset was not measured.
pub const RESET_GATES: [u32; 4] = [0x7F9E_FBFF, 0xFFDE_F300, 0xFFFF_FFCF, 0x3F00_C0F4];

/// The clock gates [`Mcf5441x::build`] starts with (RM 9.2.4): a gated module reads all ones at every size and a
/// write to it is dropped (both measured on an MCF54415); it neither counts nor raises anything (guess: not
/// measured; see [`Device::gate`](crate::Device::gate)).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Gates {
    /// Every module clocked (PPM{H,L}R all 0), as the models were before clock gates were modelled: the default.
    #[default]
    Open,
    /// The gates after a soft reset ([`RESET_GATES`], measured on an MCF54415).
    Reset,
    /// These register values.
    Set {
        hr0: u32,
        lr0: u32,
        hr1: u32,
        lr1: u32,
    },
}

impl Gates {
    /// PPMHR0, PPMLR0, PPMHR1, PPMLR1.
    pub fn registers(self) -> [u32; 4] {
        match self {
            Gates::Open => [0; 4],
            Gates::Reset => RESET_GATES,
            Gates::Set { hr0, lr0, hr1, lr1 } => [hr0, lr0, hr1, lr1],
        }
    }
}

/// PLL_CR as the default reset configuration leaves it (RM Figure 8-3): LOLEN set, REFDIV 0, FBKDIV 9.
const PLL_CR_RESET: u32 = 1 << 12 | 9;
/// PLL_DR as the default and parallel reset configurations leave it (RM Figure 8-4): OUTDIV5 0x1F, OUTDIV4 7,
/// OUTDIV3 9, OUTDIV2 3, OUTDIV1 1 (the reset row gives OUTDIV4 and OUTDIV3; the note gives the others).
const PLL_DR_RESET: u32 = 0x1F << 21 | 7 << 16 | 9 << 10 | 3 << 5 | 1;

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
    /// How long the card is busy after a write's last block ([`EsdhcConfig::write_busy`]).
    pub sd_write_busy: u64,
    /// How long the card is busy after a CMD38 ([`EsdhcConfig::erase_busy`]).
    pub sd_erase_busy: u64,
    /// The clock gates at the start ([`Gates`]).
    pub gates: Gates,
    /// Each DMA timer's DT*n*IN pin: ticks between its falling edges, or `None` when it never toggles (see
    /// [`Dtim::new`]), DTIN0 to DTIN3.
    pub dtin: [Option<u64>; 4],
}

impl Default for ChipOptions {
    /// The chip with nothing on it: no NOR, card or NAND; PLL_CR 0x1009 and PLL_DR 0x03E7_2461, the values the
    /// default reset configuration (RCON) leaves (RM Figures 8-3 and 8-4); UARTs with no character time (0 ticks
    /// per byte); SSIs with a frame every tick; USB with 1 tick per microsecond; the eSDHC with a data latency of
    /// 1 tick and a CMD6 busy of 1 tick; every clock gate open ([`Gates::Open`], not the chip's reset state, which
    /// [`Gates::Reset`] gives); no DT*n*IN pin ever toggles.
    fn default() -> ChipOptions {
        ChipOptions {
            nor: None,
            card: None,
            nand: None,
            pll: (PLL_CR_RESET, PLL_DR_RESET),
            uart_ticks_per_byte: [0; 10],
            ssi_ticks_per_frame: [1; 2],
            usb_ticks_per_us: 1,
            sd_data_latency: 1,
            sd_cmd6_busy: 1,
            sd_write_busy: 1,
            sd_erase_busy: 1,
            gates: Gates::Open,
            dtin: [None; 4],
        }
    }
}

impl ChipOptions {
    /// Put `nor` on DSPI`dspi`'s chip select PCS`pcs`.
    pub fn with_nor(mut self, dspi: u8, pcs: u8, nor: SpiNor) -> ChipOptions {
        self.nor = Some((dspi, pcs, nor));
        self
    }

    /// Put `card` behind the eSDHC.
    pub fn with_card(mut self, card: Emmc) -> ChipOptions {
        self.card = Some(card);
        self
    }

    /// Put `nand` behind the NFC.
    pub fn with_nand(mut self, nand: NandArray) -> ChipOptions {
        self.nand = Some(nand);
        self
    }

    /// Set PLL_CR and PLL_DR as the reset configuration left them.
    pub fn with_pll(mut self, cr: u32, dr: u32) -> ChipOptions {
        self.pll = (cr, dr);
        self
    }

    /// Set UART`n`'s character time (0-9). Panics when `n` is above 9.
    pub fn with_uart_ticks(mut self, n: usize, ticks_per_byte: u64) -> ChipOptions {
        self.uart_ticks_per_byte[n] = ticks_per_byte;
        self
    }

    /// Set SSI`n`'s frame period (0 or 1). Panics when `n` is above 1; [`Mcf5441x::build`] rejects a period of 0.
    pub fn with_ssi_ticks(mut self, n: usize, ticks_per_frame: u64) -> ChipOptions {
        self.ssi_ticks_per_frame[n] = ticks_per_frame;
        self
    }

    /// Set the USB controller's time scale.
    pub fn with_usb_ticks_per_us(mut self, ticks_per_us: u64) -> ChipOptions {
        self.usb_ticks_per_us = ticks_per_us;
        self
    }

    /// Set the card's busy times after a write's last block and after a CMD38, in ticks.
    pub fn with_sd_busy(mut self, write: u64, erase: u64) -> ChipOptions {
        self.sd_write_busy = write;
        self.sd_erase_busy = erase;
        self
    }

    /// Set the eSDHC's command-to-data delay and how long the card holds DAT0 busy after a CMD6.
    pub fn with_sd_timing(mut self, data_latency: u64, cmd6_busy: u64) -> ChipOptions {
        self.sd_data_latency = data_latency;
        self.sd_cmd6_busy = cmd6_busy;
        self
    }

    /// Start with the clock gates `gates`.
    pub fn with_gates(mut self, gates: Gates) -> ChipOptions {
        self.gates = gates;
        self
    }

    /// Give DMA timer `n`'s DT*n*IN pin a falling edge every `ticks_per_edge` ticks, or none (`None`, the default).
    /// Panics when `n` is above 3; [`Mcf5441x::build`] rejects a period of 0.
    pub fn with_dtin(mut self, n: usize, ticks_per_edge: Option<u64>) -> ChipOptions {
        self.dtin[n] = ticks_per_edge;
        self
    }
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
    /// DMA timers 0-3 ([`Dtim`]).
    pub dtim: [DeviceId; 4],
    /// Edge port 0 ([`Eport`]).
    pub eport: DeviceId,
    /// The eDMA ([`Edma`]).
    pub edma: DeviceId,
    /// UART0-UART9 ([`Uart`]).
    pub uart: [DeviceId; 10],
    /// SSI0 and SSI1 ([`Ssi`]).
    pub ssi: [DeviceId; 2],
    /// DSPI0-DSPI3 ([`Dspi`]).
    pub dspi: [DeviceId; 4],
    /// Where the NOR sits, (DSPI, PCS), when [`ChipOptions::nor`] gave one: see [`Chip::nor`].
    pub nor: Option<(u8, u8)>,
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

impl Chip {
    /// The SPI NOR flash, if [`ChipOptions::nor`] gave one.
    pub fn nor<'m>(&self, m: &'m Machine) -> Option<&'m SpiNor> {
        let (d, pcs) = self.nor?;
        Some(
            m.device::<Dspi>(self.dspi[usize::from(d)])
                .device::<SpiNor>(pcs),
        )
    }

    /// The SPI NOR flash, mutably.
    pub fn nor_mut<'m>(&self, m: &'m mut Machine) -> Option<&'m mut SpiNor> {
        let (d, pcs) = self.nor?;
        Some(
            m.device_mut::<Dspi>(self.dspi[usize::from(d)])
                .device_mut::<SpiNor>(pcs),
        )
    }

    /// The clock gates as PPMHR0, PPMLR0, PPMHR1 and PPMLR1 read (bit set: the module's clock is off).
    pub fn gates(&self, m: &Machine) -> [u32; 4] {
        scm::registers(m.clock_gates())
    }

    /// Drive the pad of edge port pin IRQ`n` at `level`, for the edge port ([`Eport::set_pin`]) and as its GPIO
    /// port C pin ([`GpioPorts::drive`]), as the one pad it is: IRQ1 PC1, IRQ2 PC2, IRQ3 PC3, IRQ4 PC4, IRQ6 PC5 and
    /// IRQ7 PC6 (RM Table 15-2's primary functions). guess: the GPIO data register reads the pad while it is muxed
    /// to IRQ`n`, as the RM's port data registers read the pins whatever their function (not measured). Panics for
    /// IRQ0 and IRQ5, which have no pad here.
    pub fn drive_irq(&self, m: &mut Machine, n: u8, level: bool) {
        let bit = match n {
            1..=4 => n,
            6 | 7 => n - 1,
            _ => panic!("no IRQ{n} pad: IRQ1-IRQ4, IRQ6 and IRQ7"),
        };
        m.with_device(self.eport, |e: &mut Eport, cx| e.set_pin(n, level, cx));
        self.gpio_mut(m).drive(2, bit, Some(level));
    }

    /// The GPIO ports ([`GpioPorts`]) inside the GPIO stub block.
    pub fn gpio<'m>(&self, m: &'m Machine) -> &'m GpioPorts {
        m.device::<StubBlock>(self.stub("GPIO"))
            .registers()
            .expect("the GPIO stub holds the ports")
    }

    /// The GPIO ports, mutably: drive inputs, take the change log.
    pub fn gpio_mut<'m>(&self, m: &'m mut Machine) -> &'m mut GpioPorts {
        m.device_mut::<StubBlock>(self.stub("GPIO"))
            .registers_mut()
            .expect("the GPIO stub holds the ports")
    }

    fn stub(&self, name: &str) -> DeviceId {
        self.stubs
            .iter()
            .find(|(n, _)| *n == name)
            .expect("the chip has the stub")
            .1
    }

    /// The eSDHC's card, if [`ChipOptions::card`] gave one.
    pub fn card<'m>(&self, m: &'m Machine) -> Option<&'m Emmc> {
        m.device::<Esdhc>(self.esdhc).card()
    }

    /// The eSDHC's card, mutably.
    pub fn card_mut<'m>(&self, m: &'m mut Machine) -> Option<&'m mut Emmc> {
        m.device_mut::<Esdhc>(self.esdhc).card_mut()
    }
}

/// The MCF5441x's on-chip modules, mapped into a [`Machine`].
///
/// | Base | Size | Block | Interrupt (controller, source) | DMA requests |
/// |---|---|---|---|---|
/// | 0xFC04_8000, 0xFC04_C000, 0xFC05_0000 | 0x4000 | INTC0-2 ([`Intc::chip`], added in that order) | | |
/// | 0xFC05_4000 | 0x4000 | global IACK | | |
/// | 0xFC04_4000 | 0x2000 | eDMA (bus master) | its own routes, see [`Edma`] | software: 30, 31, 42, 43, 46, 47 |
/// | 0xFC08_0000 + 0x4000 n | 0x4000 | PIT0-3 | INTC2 13-16 | |
/// | 0xFC07_0000 + 0x4000 n | 0x4000 | DMA timers 0-3 | INTC0 32-35 | |
/// | 0xFC09_0000 | 0x4000 | Edge port 0 | INTC0 1-7 (pins IRQ1-IRQ7) | |
/// | 0xFC06_0000 + 0x4000 n | 0x4000 | UART0-3 | INTC0 26-29 | receive 2, 4, 6, 16; transmit 3, 5, 7, 17 |
/// | 0xEC06_0000 + 0x4000 n | 0x4000 | UART4-7 | INTC1 48-51 | receive 18, 20, 22, 32; transmit 19, 21, 23, 33 |
/// | 0xEC07_0000, 0xEC07_4000 | 0x4000 | UART8, UART9 | INTC1 52, 53 | receive 34, 36; transmit 35, 37 |
/// | 0xFC0B_C000, 0xFC0C_8000 | 0x4000 | SSI0, SSI1 | | TX0 50, 54; RX0 48, 52 |
/// | 0xFC05_C000, 0xFC03_C000, 0xEC03_8000, 0xEC03_C000 | 0x4000 | DSPI0-3 | | RFDF 12, 14, 28, 44; TFFF 13, 15, 29, 45 |
/// | 0xFC0C_C000 | 0x4000 | eSDHC | INTC2 31 | 59 |
/// | 0xFC0B_0000 | 0x1000 | USB OTG (bus master) | INTC2 17 | |
/// | 0xFC0B_8000 | 0x4000 | DDR controller | | |
/// | 0xFC0C_0000 | 0x4000 | PLL | | |
/// | 0xFC0F_C000 | 0x4000 | NFC (with a NAND array) | | |
///
/// The stub blocks (read back what is written): XBS 0xFC00_4000, FlexBus 0xFC00_8000, SCM 0xFC04_0000, CCM
/// 0xEC09_0000 (with the reset controller and power management), GPIO 0xEC09_4000 (pin multiplexing and
/// control). The SCM's peripheral power management registers (+0x2C-+0x3F) are a [`PowerManagement`] model
/// inside its stub ([`StubBlock::with_registers`]); writes to them are still in the stub's log.
///
/// Clock gates (RM 9.2.4): every module in a peripheral slot with a CDn bit (RM Tables 9-5 to 9-8) is tied to its
/// gate, `64 s + n` for CDn of PPM{H,L}R`s` ([`Machine::set_clock_gate`]); the SCM, the crossbar switch, the
/// global IACK slot and RGPIO have none. While a module's gate is closed, it reads all ones at every size and a
/// write to it is dropped (both measured on an MCF54415); its events are held and its lines kept as they were
/// (guess: not measured). [`ChipOptions::gates`] sets the gates at the start: by default all open, not the reset
/// state.
///
/// Every other module of RM Table 1-3 and Table 1-4 is an [`Unmodelled`] block over its 16 KB slot, named as the
/// table names it, and so is RGPIO at 0x8C00_0000-0x8FFF_FFFF: an access stops the run with
/// [`Stop::Unmodelled`](crate::Stop::Unmodelled) and the detail `"<module> +<offset>"`.
///
/// The interrupt sources the models leave unwired are not driven: SSI, DSPI and NFC raise no interrupts (their
/// enables are refused, see each model); the DDR controller and the PLL store their interrupt enables (DDR INTMASK,
/// PLL_CR LOCIRQ and LOLIRQ) but never raise an interrupt, and neither is wired to an interrupt controller; a PLL
/// loss of lock that would request its interrupt (or a reset) stops the run ([`Pll::set_locked`]).
///
/// The eDMA ([`Edma::with_driven`]) takes SERQ only on the request lines in the table's last column (the
/// software channels of RM Table 19-6 among them): on any other channel (the timers', the I2C modules', the
/// external DREQ pins, ...) nothing would ever request, and SERQ is refused. A software start works on every channel.
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
    /// larger than an NFC buffer, a DT*n*IN period of 0).
    pub fn build(m: &mut Machine, options: ChipOptions) -> Chip {
        let ([intc0, intc1, intc2], iack) = Intc::chip();
        // In this order: the controllers' indices are the order they are added (RM 17.3.2).
        let intc = [
            m.add_interrupt_controller(INTC[0], SLOT, Box::new(intc0)),
            m.add_interrupt_controller(INTC[1], SLOT, Box::new(intc1)),
            m.add_interrupt_controller(INTC[2], SLOT, Box::new(intc2)),
        ];
        let iack = m.add_device(IACK, SLOT, Box::new(iack));

        // SERQ is taken on the request lines the models below drive and on the software channels only.
        let driven = UART
            .iter()
            .flat_map(|u| [u.3, u.4])
            .chain(DSPI.iter().flat_map(|d| [d.1, d.2]))
            .chain(SSI.iter().flat_map(|s| [s.1, s.2]))
            .chain([ESDHC_DMA])
            .chain(SOFTWARE_DMA)
            .fold(0u64, |mask, ch| mask | 1 << ch);
        let edma = m.add_bus_master(EDMA.0, EDMA.1, Box::new(Edma::with_driven(driven)));

        let pit = [0u8, 1, 2, 3].map(|n| {
            let base = PIT + SLOT * u32::from(n);
            m.add_device(base, SLOT, Box::new(Pit::new(2, PIT_SOURCE + n)))
        });

        let dtim = [0u8, 1, 2, 3].map(|n| {
            let base = DTIM + SLOT * u32::from(n);
            let dtin = options.dtin[usize::from(n)];
            m.add_device(base, SLOT, Box::new(Dtim::new(n, 0, DTIM_SOURCE + n, dtin)))
        });
        let eport = m.add_device(EPORT, SLOT, Box::new(Eport::new(0)));

        let uart = std::array::from_fn(|n| {
            let (base, controller, source, rx, tx) = UART[n];
            let config = UartConfig {
                controller,
                source,
                rx_dma: Some(rx),
                tx_dma: Some(tx),
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

        let nor_at = options.nor.as_ref().map(|&(d, pcs, _)| (d, pcs));
        let mut dspis = DSPI.map(|(_, rx, tx)| Dspi::with_dma(rx, tx));
        if let Some((d, pcs, nor)) = options.nor {
            assert!(d < 4, "no DSPI{d}: DSPI0-DSPI3 only");
            dspis[usize::from(d)].attach(pcs, Box::new(nor));
        }
        let mut dspis = dspis.into_iter();
        let dspi = DSPI.map(|(base, ..)| m.add_device(base, SLOT, Box::new(dspis.next().unwrap())));

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
                    write_busy: options.sd_write_busy,
                    erase_busy: options.sd_erase_busy,
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
                let mut block = StubBlock::new(name, StubMode::ReadBack);
                if name == "SCM" {
                    let pm = Box::new(PowerManagement::new());
                    block = block.with_registers(POWER_MANAGEMENT, pm);
                }
                if name == "GPIO" {
                    block = block.with_registers(GpioPorts::RANGE, Box::new(GpioPorts::new()));
                }
                (name, m.add_device(base, SLOT, Box::new(block)))
            })
            .collect();
        for (name, base) in UNMODELLED {
            m.add_device(base, SLOT, Box::new(Unmodelled::new(name)));
        }
        m.add_device(RGPIO.0, RGPIO.1, Box::new(Unmodelled::new("RGPIO")));

        for (name, base, gate) in CLOCK_GATES {
            // CDn is the slot number.
            debug_assert_eq!(base >> 14 & 0x3F, u32::from(gate % 64), "{name}");
            debug_assert_eq!(base < 0xFC00_0000, gate >= 64, "{name}");
            let id = m.mem.device_at(base).expect("every gated module is mapped");
            m.set_clock_gate(id, Some(gate));
        }
        m.set_clock_gates(scm::gates(options.gates.registers()));

        Chip {
            intc,
            iack,
            pit,
            dtim,
            eport,
            edma,
            uart,
            ssi,
            dspi,
            nor: nor_at,
            esdhc,
            usb,
            ddrmc,
            pll,
            nfc,
            stubs,
        }
    }
}
