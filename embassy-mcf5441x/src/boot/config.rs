//! The board's description of its clocks, DDR and boot medium, checked at compile time against the reference
//! manual's limits (RM 8.1.2, 8.2.2, 8.3.2). A board above a frequency limit says so with [`OutOfSpec`].
use crate::clocks::Clocks;

/// The PLL fields (RM 8.2): fVCO = fREF × (FBKDIV + 1) / 2^REFDIV, core = fVCO / (OUTDIV1 + 1), bus =
/// fVCO / (OUTDIV2 + 1), NFC = fVCO / (OUTDIV5 + 1) (0: the NFC clock is off).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct PllSettings {
    pub refdiv: u8,
    pub fbkdiv: u8,
    pub outdiv1: u8,
    pub outdiv2: u8,
    pub outdiv5: u8,
}

/// `Keep`: the boot hardware set the PLL (e.g. a serial-boot header's SBF_RCON), and the settings say what it set;
/// the boot part checks them against PLL_CR/PLL_DR and halts on a mismatch. `Set`: the boot part programs them.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Pll {
    Keep(PllSettings),
    Set(PllSettings),
}

/// The PLL reference, in Hz: a crystal on EXTAL/XTAL, or an oscillator driving EXTAL.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum RefClock {
    Crystal(u32),
    Oscillator(u32),
}

/// The source of the DDR controller's 2× clock (`MISCCR2[DDR2CLK]`, RM 10.3.10): the core clock or the VCO.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Ddr2Clk {
    Core,
    Vco,
}

/// The DDR controller as the board's DRAM needs it: CR words as (index 0..=63, value) in write order with START
/// (CR09 bit 24) clear, since the sequence starts the controller itself; DDR_PADCR; and the SDRAM pad drive
/// (MSCR_SDRAMC: full or half strength).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Ddr {
    pub cr: &'static [(u8, u32)],
    pub padcr: u32,
    pub full_strength: bool,
}

/// Where in the sequence an [`ExtraWrite`] runs.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Phase {
    /// Right after the PLL is set (or checked) and locked.
    AfterPll,
    /// Right before the DDR controller's clock is ungated.
    BeforeDdrController,
}

/// The width of a register access.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Width {
    W8,
    W16,
    W32,
}

/// A board register write the config has no field for, such as crossbar priorities.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct ExtraWrite {
    pub phase: Phase,
    pub width: Width,
    pub addr: u32,
    pub value: u32,
}

/// The NOR flash on FlexBus CS0 (`boot-nor`): its chip-select control word (wait states, port size, ...) and its
/// size in bytes. It stays at address 0 (CSAR0 = 0).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct FlexBusCs0 {
    pub cscr: u32,
    pub size: u32,
}

/// Where the image's bytes are once the boot part runs. `Mapped(base)` and `SpiFlash { offset }` name where image
/// offset 0 (the initial SP) sits; the app part is found from there by its offset in the image. `Nand`: the pages
/// after the four boot pages, in the boot-page format.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum AppSource {
    Mapped(u32),
    SpiFlash { offset: u32, ctar: u32 },
    Nand,
}

/// `IKnowWhatImDoing` lifts the frequency limits (reference, VCO, core, NFC, FBKDIV range); the encodings and the
/// controller's mandatory fields stay checked.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum OutOfSpec {
    No,
    IKnowWhatImDoing,
}

/// A board's cold-boot description. Make one with [`BootConfig::builder`] in a `const` or `static` initializer: a
/// check that fails is then a compile error with the check's message.
#[derive(Copy, Clone, Debug)]
pub struct BootConfig {
    pub(crate) ref_clock: RefClock,
    pub(crate) pll: Pll,
    pub(crate) ddr: Ddr,
    pub(crate) ddr2clk: Option<Ddr2Clk>,
    pub(crate) outdiv3: Option<u8>,
    pub(crate) outdiv4: Option<u8>,
    pub(crate) extra: &'static [ExtraWrite],
    pub(crate) cs0: Option<FlexBusCs0>,
    #[allow(dead_code)] // read by the fetch
    pub(crate) app: AppSource,
}

/// Collects a [`BootConfig`]'s optional parts; [`build`](Self::build) checks everything.
#[derive(Copy, Clone, Debug)]
pub struct BootConfigBuilder {
    ref_clock: RefClock,
    pll: Pll,
    ddr: Ddr,
    ddr2clk: Option<Ddr2Clk>,
    outdiv3: Option<u8>,
    outdiv4: Option<u8>,
    extra: &'static [ExtraWrite],
    cs0: Option<FlexBusCs0>,
    app: Option<AppSource>,
    out_of_spec: OutOfSpec,
}

const MHZ: u64 = 1_000_000;
/// CR09 START (RM 21.4): the sequence sets it after every other CR write.
pub(crate) const START: u32 = 1 << 24;

impl BootConfig {
    pub const fn builder(ref_clock: RefClock, pll: Pll, ddr: Ddr) -> BootConfigBuilder {
        BootConfigBuilder {
            ref_clock,
            pll,
            ddr,
            ddr2clk: None,
            outdiv3: None,
            outdiv4: None,
            extra: &[],
            cs0: None,
            app: None,
            out_of_spec: OutOfSpec::No,
        }
    }

    /// The clocks these settings give.
    pub const fn clocks(&self) -> Clocks {
        clocks_of(ref_hz(self.ref_clock), settings(self.pll))
    }
}

const fn ref_hz(r: RefClock) -> u32 {
    match r {
        RefClock::Crystal(hz) | RefClock::Oscillator(hz) => hz,
    }
}

pub(crate) const fn settings(p: Pll) -> PllSettings {
    match p {
        Pll::Keep(s) | Pll::Set(s) => s,
    }
}

const fn vco_hz(fref: u32, s: PllSettings) -> u64 {
    fref as u64 * (s.fbkdiv as u64 + 1) >> s.refdiv
}

const fn clocks_of(fref: u32, s: PllSettings) -> Clocks {
    let vco = vco_hz(fref, s);
    Clocks {
        vco_hz: vco as u32,
        core_hz: (vco / (s.outdiv1 as u64 + 1)) as u32,
        bus_hz: (vco / (s.outdiv2 as u64 + 1)) as u32,
    }
}

impl BootConfigBuilder {
    /// Switch the DDR 2× clock source (inside a limp-mode window).
    pub const fn ddr2clk(mut self, c: Ddr2Clk) -> Self {
        self.ddr2clk = Some(c);
        self
    }
    /// Set `PLL_DR[OUTDIV3]` (the eSDHC clock), which a serial-boot header can't.
    pub const fn outdiv3(mut self, v: u8) -> Self {
        self.outdiv3 = Some(v);
        self
    }
    /// Set `PLL_DR[OUTDIV4]` (the USB clock), which a serial-boot header can't.
    pub const fn outdiv4(mut self, v: u8) -> Self {
        self.outdiv4 = Some(v);
        self
    }
    pub const fn extra(mut self, w: &'static [ExtraWrite]) -> Self {
        self.extra = w;
        self
    }
    pub const fn cs0(mut self, c: FlexBusCs0) -> Self {
        self.cs0 = Some(c);
        self
    }
    pub const fn app(mut self, a: AppSource) -> Self {
        self.app = Some(a);
        self
    }
    pub const fn out_of_spec(mut self, o: OutOfSpec) -> Self {
        self.out_of_spec = o;
        self
    }

    /// Check everything and make the config. Panics (a compile error in a `const`/`static`) with the failed check.
    ///
    /// # Checks
    ///
    /// Each of these is a compile error. The frequency limits lift with `.out_of_spec(OutOfSpec::IKnowWhatImDoing)`;
    /// the rest don't.
    ///
    /// The reference below 14 MHz:
    ///
    /// ```compile_fail
    /// # use embassy_mcf5441x::boot::*;
    /// # const DDR: Ddr = Ddr { cr: &[(0, 0x0000_0101)], padcr: 1 << 9 | 1, full_strength: false };
    /// # const S: PllSettings = PllSettings { refdiv: 0, fbkdiv: 19, outdiv1: 1, outdiv2: 3, outdiv5: 7 };
    /// # const A: AppSource = AppSource::Mapped(0x8000_0000);
    /// const C: BootConfig = BootConfig::builder(RefClock::Crystal(13_000_000), Pll::Set(S), DDR).cs0(FlexBusCs0 { cscr: 0, size: 0x100_0000 }).app(A).build();
    /// ```
    ///
    /// The same, out of spec on purpose:
    ///
    /// ```
    /// # use embassy_mcf5441x::boot::*;
    /// # const DDR: Ddr = Ddr { cr: &[(0, 0x0000_0101)], padcr: 1 << 9 | 1, full_strength: false };
    /// # const S: PllSettings = PllSettings { refdiv: 0, fbkdiv: 19, outdiv1: 1, outdiv2: 3, outdiv5: 7 };
    /// # const A: AppSource = AppSource::Mapped(0x8000_0000);
    /// const C: BootConfig = BootConfig::builder(RefClock::Crystal(13_000_000), Pll::Set(S), DDR).cs0(FlexBusCs0 { cscr: 0, size: 0x100_0000 }).app(A).out_of_spec(OutOfSpec::IKnowWhatImDoing).build();
    /// ```
    ///
    /// A VCO above 500 MHz:
    ///
    /// ```compile_fail
    /// # use embassy_mcf5441x::boot::*;
    /// # const DDR: Ddr = Ddr { cr: &[(0, 0x0000_0101)], padcr: 1 << 9 | 1, full_strength: false };
    /// # const S: PllSettings = PllSettings { refdiv: 0, fbkdiv: 19, outdiv1: 1, outdiv2: 3, outdiv5: 7 };
    /// # const A: AppSource = AppSource::Mapped(0x8000_0000);
    /// const C: BootConfig = BootConfig::builder(RefClock::Crystal(25_000_000), Pll::Set(PllSettings { fbkdiv: 20, outdiv1: 2, outdiv2: 5, outdiv5: 0, ..S }), DDR).cs0(FlexBusCs0 { cscr: 0, size: 0x100_0000 }).app(A).build();
    /// ```
    ///
    /// The same, out of spec on purpose:
    ///
    /// ```
    /// # use embassy_mcf5441x::boot::*;
    /// # const DDR: Ddr = Ddr { cr: &[(0, 0x0000_0101)], padcr: 1 << 9 | 1, full_strength: false };
    /// # const S: PllSettings = PllSettings { refdiv: 0, fbkdiv: 19, outdiv1: 1, outdiv2: 3, outdiv5: 7 };
    /// # const A: AppSource = AppSource::Mapped(0x8000_0000);
    /// const C: BootConfig = BootConfig::builder(RefClock::Crystal(25_000_000), Pll::Set(PllSettings { fbkdiv: 20, outdiv1: 2, outdiv2: 5, outdiv5: 0, ..S }), DDR).cs0(FlexBusCs0 { cscr: 0, size: 0x100_0000 }).app(A).out_of_spec(OutOfSpec::IKnowWhatImDoing).build();
    /// ```
    ///
    /// A core clock above 250 MHz:
    ///
    /// ```compile_fail
    /// # use embassy_mcf5441x::boot::*;
    /// # const DDR: Ddr = Ddr { cr: &[(0, 0x0000_0101)], padcr: 1 << 9 | 1, full_strength: false };
    /// # const S: PllSettings = PllSettings { refdiv: 0, fbkdiv: 19, outdiv1: 1, outdiv2: 3, outdiv5: 7 };
    /// # const A: AppSource = AppSource::Mapped(0x8000_0000);
    /// const C: BootConfig = BootConfig::builder(RefClock::Crystal(25_000_000), Pll::Set(PllSettings { outdiv1: 0, outdiv2: 1, ..S }), DDR).cs0(FlexBusCs0 { cscr: 0, size: 0x100_0000 }).app(A).build();
    /// ```
    ///
    /// The same, out of spec on purpose:
    ///
    /// ```
    /// # use embassy_mcf5441x::boot::*;
    /// # const DDR: Ddr = Ddr { cr: &[(0, 0x0000_0101)], padcr: 1 << 9 | 1, full_strength: false };
    /// # const S: PllSettings = PllSettings { refdiv: 0, fbkdiv: 19, outdiv1: 1, outdiv2: 3, outdiv5: 7 };
    /// # const A: AppSource = AppSource::Mapped(0x8000_0000);
    /// const C: BootConfig = BootConfig::builder(RefClock::Crystal(25_000_000), Pll::Set(PllSettings { outdiv1: 0, outdiv2: 1, ..S }), DDR).cs0(FlexBusCs0 { cscr: 0, size: 0x100_0000 }).app(A).out_of_spec(OutOfSpec::IKnowWhatImDoing).build();
    /// ```
    ///
    /// FBKDIV below 8:
    ///
    /// ```compile_fail
    /// # use embassy_mcf5441x::boot::*;
    /// # const DDR: Ddr = Ddr { cr: &[(0, 0x0000_0101)], padcr: 1 << 9 | 1, full_strength: false };
    /// # const S: PllSettings = PllSettings { refdiv: 0, fbkdiv: 19, outdiv1: 1, outdiv2: 3, outdiv5: 7 };
    /// # const A: AppSource = AppSource::Mapped(0x8000_0000);
    /// const C: BootConfig = BootConfig::builder(RefClock::Crystal(50_000_000), Pll::Set(PllSettings { fbkdiv: 7, outdiv5: 0, ..S }), DDR).cs0(FlexBusCs0 { cscr: 0, size: 0x100_0000 }).app(A).build();
    /// ```
    ///
    /// The same, out of spec on purpose:
    ///
    /// ```
    /// # use embassy_mcf5441x::boot::*;
    /// # const DDR: Ddr = Ddr { cr: &[(0, 0x0000_0101)], padcr: 1 << 9 | 1, full_strength: false };
    /// # const S: PllSettings = PllSettings { refdiv: 0, fbkdiv: 19, outdiv1: 1, outdiv2: 3, outdiv5: 7 };
    /// # const A: AppSource = AppSource::Mapped(0x8000_0000);
    /// const C: BootConfig = BootConfig::builder(RefClock::Crystal(50_000_000), Pll::Set(PllSettings { fbkdiv: 7, outdiv5: 0, ..S }), DDR).cs0(FlexBusCs0 { cscr: 0, size: 0x100_0000 }).app(A).out_of_spec(OutOfSpec::IKnowWhatImDoing).build();
    /// ```
    ///
    /// An NFC clock above 80 MHz:
    ///
    /// ```compile_fail
    /// # use embassy_mcf5441x::boot::*;
    /// # const DDR: Ddr = Ddr { cr: &[(0, 0x0000_0101)], padcr: 1 << 9 | 1, full_strength: false };
    /// # const S: PllSettings = PllSettings { refdiv: 0, fbkdiv: 19, outdiv1: 1, outdiv2: 3, outdiv5: 7 };
    /// # const A: AppSource = AppSource::Mapped(0x8000_0000);
    /// const C: BootConfig = BootConfig::builder(RefClock::Crystal(25_000_000), Pll::Set(PllSettings { outdiv5: 3, ..S }), DDR).cs0(FlexBusCs0 { cscr: 0, size: 0x100_0000 }).app(A).build();
    /// ```
    ///
    /// The same, out of spec on purpose:
    ///
    /// ```
    /// # use embassy_mcf5441x::boot::*;
    /// # const DDR: Ddr = Ddr { cr: &[(0, 0x0000_0101)], padcr: 1 << 9 | 1, full_strength: false };
    /// # const S: PllSettings = PllSettings { refdiv: 0, fbkdiv: 19, outdiv1: 1, outdiv2: 3, outdiv5: 7 };
    /// # const A: AppSource = AppSource::Mapped(0x8000_0000);
    /// const C: BootConfig = BootConfig::builder(RefClock::Crystal(25_000_000), Pll::Set(PllSettings { outdiv5: 3, ..S }), DDR).cs0(FlexBusCs0 { cscr: 0, size: 0x100_0000 }).app(A).out_of_spec(OutOfSpec::IKnowWhatImDoing).build();
    /// ```
    ///
    /// A bus clock that isn't half the core:
    ///
    /// ```compile_fail
    /// # use embassy_mcf5441x::boot::*;
    /// # const DDR: Ddr = Ddr { cr: &[(0, 0x0000_0101)], padcr: 1 << 9 | 1, full_strength: false };
    /// # const S: PllSettings = PllSettings { refdiv: 0, fbkdiv: 19, outdiv1: 1, outdiv2: 3, outdiv5: 7 };
    /// # const A: AppSource = AppSource::Mapped(0x8000_0000);
    /// const C: BootConfig = BootConfig::builder(RefClock::Crystal(25_000_000), Pll::Set(PllSettings { outdiv2: 1, ..S }), DDR).cs0(FlexBusCs0 { cscr: 0, size: 0x100_0000 }).app(A).build();
    /// ```
    ///
    /// CR index 64:
    ///
    /// ```compile_fail
    /// # use embassy_mcf5441x::boot::*;
    /// # const DDR: Ddr = Ddr { cr: &[(0, 0x0000_0101)], padcr: 1 << 9 | 1, full_strength: false };
    /// # const S: PllSettings = PllSettings { refdiv: 0, fbkdiv: 19, outdiv1: 1, outdiv2: 3, outdiv5: 7 };
    /// # const A: AppSource = AppSource::Mapped(0x8000_0000);
    /// const C: BootConfig = BootConfig::builder(RefClock::Crystal(25_000_000), Pll::Set(S), Ddr { cr: &[(64, 0)], ..DDR }).cs0(FlexBusCs0 { cscr: 0, size: 0x100_0000 }).app(A).build();
    /// ```
    ///
    /// START set in CR09:
    ///
    /// ```compile_fail
    /// # use embassy_mcf5441x::boot::*;
    /// # const DDR: Ddr = Ddr { cr: &[(0, 0x0000_0101)], padcr: 1 << 9 | 1, full_strength: false };
    /// # const S: PllSettings = PllSettings { refdiv: 0, fbkdiv: 19, outdiv1: 1, outdiv2: 3, outdiv5: 7 };
    /// # const A: AppSource = AppSource::Mapped(0x8000_0000);
    /// const C: BootConfig = BootConfig::builder(RefClock::Crystal(25_000_000), Pll::Set(S), Ddr { cr: &[(9, 1 << 24)], ..DDR }).cs0(FlexBusCs0 { cscr: 0, size: 0x100_0000 }).app(A).build();
    /// ```
    ///
    /// PADCR bit 9 clear:
    ///
    /// ```compile_fail
    /// # use embassy_mcf5441x::boot::*;
    /// # const DDR: Ddr = Ddr { cr: &[(0, 0x0000_0101)], padcr: 1 << 9 | 1, full_strength: false };
    /// # const S: PllSettings = PllSettings { refdiv: 0, fbkdiv: 19, outdiv1: 1, outdiv2: 3, outdiv5: 7 };
    /// # const A: AppSource = AppSource::Mapped(0x8000_0000);
    /// const C: BootConfig = BootConfig::builder(RefClock::Crystal(25_000_000), Pll::Set(S), Ddr { padcr: 1, ..DDR }).cs0(FlexBusCs0 { cscr: 0, size: 0x100_0000 }).app(A).build();
    /// ```
    ///
    /// No app source:
    ///
    /// ```compile_fail
    /// # use embassy_mcf5441x::boot::*;
    /// # const DDR: Ddr = Ddr { cr: &[(0, 0x0000_0101)], padcr: 1 << 9 | 1, full_strength: false };
    /// # const S: PllSettings = PllSettings { refdiv: 0, fbkdiv: 19, outdiv1: 1, outdiv2: 3, outdiv5: 7 };
    /// # const A: AppSource = AppSource::Mapped(0x8000_0000);
    /// const C: BootConfig = BootConfig::builder(RefClock::Crystal(25_000_000), Pll::Set(S), DDR).cs0(FlexBusCs0 { cscr: 0, size: 0x100_0000 }).build();
    /// ```
    ///
    pub const fn build(self) -> BootConfig {
        let strict = matches!(self.out_of_spec, OutOfSpec::No);
        let fref = ref_hz(self.ref_clock);
        let s = settings(self.pll);
        if strict && (fref < 14_000_000 || fref > 50_000_000) {
            panic!("BootConfig: the PLL reference must be 14 to 50 MHz (RM 8.1.2)");
        }
        if s.refdiv > 1 {
            panic!("BootConfig: REFDIV must be 0 (÷1) or 1 (÷2) (RM 8.2.1)");
        }
        if s.fbkdiv > 63 || s.outdiv1 > 31 || s.outdiv2 > 31 || s.outdiv5 > 31 {
            panic!("BootConfig: FBKDIV is 6 bits and each OUTDIV 5 bits wide (RM 8.2)");
        }
        if strict && s.fbkdiv < 8 {
            panic!("BootConfig: FBKDIV must be 8 to 63 (RM 8.2.1)");
        }
        let vco = vco_hz(fref, s);
        if strict && (vco < 240 * MHZ || vco > 500 * MHZ) {
            panic!("BootConfig: the VCO must run at 240 to 500 MHz (RM 8.1.2)");
        }
        if strict && s.outdiv1 > 7 {
            panic!("BootConfig: OUTDIV1 must be 0 to 7 (RM 8.2.2)");
        }
        if strict && vco / (s.outdiv1 as u64 + 1) > 250 * MHZ {
            panic!("BootConfig: the core clock must not exceed 250 MHz (RM 8.1.2)");
        }
        if s.outdiv2 as u32 + 1 != 2 * (s.outdiv1 as u32 + 1) {
            panic!("BootConfig: the bus clock must be exactly half the core clock (OUTDIV2 + 1 = 2 × (OUTDIV1 + 1), RM 8.3.2)");
        }
        if strict && s.outdiv5 != 0 && vco / (s.outdiv5 as u64 + 1) > 80 * MHZ {
            panic!("BootConfig: the NFC clock must not exceed 80 MHz (RM 8.1.2)");
        }
        if let Some(v) = self.outdiv3 {
            if v > 31 {
                panic!("BootConfig: OUTDIV3 is 5 bits wide");
            }
        }
        if let Some(v) = self.outdiv4 {
            if v > 31 {
                panic!("BootConfig: OUTDIV4 is 5 bits wide");
            }
        }
        let mut i = 0;
        while i < self.ddr.cr.len() {
            let (n, v) = self.ddr.cr[i];
            if n > 63 {
                panic!("BootConfig: DDR CR indices are 0 to 63");
            }
            if n == 9 && v & START != 0 {
                panic!("BootConfig: leave CR09 START clear; the boot sequence starts the controller last (RM 21.4)");
            }
            i += 1;
        }
        if self.ddr.padcr & 1 << 9 == 0 {
            panic!("BootConfig: DDR_PADCR bit 9 must be set (RM 21.4)");
        }
        if self.ddr.padcr & 0b11 == 0 {
            panic!("BootConfig: DDR_PADCR SPARE_DLY_CTRL must not be 00 (RM 21.4)");
        }
        let app = match self.app {
            Some(a) => a,
            None => panic!("BootConfig: name where the app part is with .app(...)"),
        };
        match app {
            AppSource::Mapped(_) => {}
            AppSource::SpiFlash { .. } => {
                if !cfg!(feature = "boot-serial") {
                    panic!("BootConfig: AppSource::SpiFlash needs boot-serial");
                }
            }
            AppSource::Nand => {
                if !cfg!(feature = "boot-nand") {
                    panic!("BootConfig: AppSource::Nand needs boot-nand");
                }
            }
        }
        if cfg!(feature = "boot-nor") && self.cs0.is_none() {
            panic!("BootConfig: boot-nor needs the NOR flash's chip select with .cs0(...)");
        }
        BootConfig {
            ref_clock: self.ref_clock,
            pll: self.pll,
            ddr: self.ddr,
            ddr2clk: self.ddr2clk,
            outdiv3: self.outdiv3,
            outdiv4: self.outdiv4,
            extra: self.extra,
            cs0: self.cs0,
            app,
        }
    }
}

/// Sources the enabled boot feature can't use are compile errors too.
#[cfg_attr(feature = "boot-serial", doc = "```compile_fail\nuse embassy_mcf5441x::boot::*;\nconst DDR: Ddr = Ddr { cr: &[], padcr: 1 << 9 | 1, full_strength: false };\nconst S: PllSettings = PllSettings { refdiv: 0, fbkdiv: 19, outdiv1: 1, outdiv2: 3, outdiv5: 7 };\nconst C: BootConfig = BootConfig::builder(RefClock::Crystal(25_000_000), Pll::Set(S), DDR).app(AppSource::Nand).build();\n```")]
#[cfg_attr(feature = "boot-nor", doc = "```compile_fail\nuse embassy_mcf5441x::boot::*;\nconst DDR: Ddr = Ddr { cr: &[], padcr: 1 << 9 | 1, full_strength: false };\nconst S: PllSettings = PllSettings { refdiv: 0, fbkdiv: 19, outdiv1: 1, outdiv2: 3, outdiv5: 7 };\nconst C: BootConfig = BootConfig::builder(RefClock::Crystal(25_000_000), Pll::Set(S), DDR).cs0(FlexBusCs0 { cscr: 0, size: 0x100_0000 }).app(AppSource::SpiFlash { offset: 0, ctar: 0 }).build();\n```")]
#[cfg_attr(feature = "boot-nor", doc = "```compile_fail\nuse embassy_mcf5441x::boot::*;\nconst DDR: Ddr = Ddr { cr: &[], padcr: 1 << 9 | 1, full_strength: false };\nconst S: PllSettings = PllSettings { refdiv: 0, fbkdiv: 19, outdiv1: 1, outdiv2: 3, outdiv5: 7 };\nconst C: BootConfig = BootConfig::builder(RefClock::Crystal(25_000_000), Pll::Set(S), DDR).app(AppSource::Mapped(0)).build();\n```")]
pub mod checks {}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;

    const DDR: Ddr = Ddr { cr: &[(0, 0x0000_0101)], padcr: 1 << 9 | 1, full_strength: false };
    const MANUAL: PllSettings = PllSettings { refdiv: 0, fbkdiv: 19, outdiv1: 1, outdiv2: 3, outdiv5: 7 };

    /// A builder every boot feature accepts: a mapped app part, and the NOR chip select `boot-nor` needs.
    fn builder(r: RefClock, p: Pll) -> BootConfigBuilder {
        BootConfig::builder(r, p, DDR).app(AppSource::Mapped(0x8000_0000)).cs0(FlexBusCs0 { cscr: 0, size: 0x100_0000 })
    }

    #[test]
    fn manual_example_builds() {
        let c = builder(RefClock::Crystal(25_000_000), Pll::Set(MANUAL)).build();
        let k = c.clocks();
        assert_eq!((k.vco_hz, k.core_hz, k.bus_hz), (500_000_000, 250_000_000, 125_000_000));
    }

    #[test]
    fn keep_is_checked_like_set() {
        let c = builder(RefClock::Crystal(25_000_000), Pll::Keep(MANUAL)).build();
        assert_eq!(c.clocks().core_hz, 250_000_000);
    }

    #[test]
    fn out_of_spec_opt_out_allows_a_525_mhz_vco() {
        // 25 MHz × 21: VCO 525 MHz and core 262.5 MHz, both over the limits
        let s = PllSettings { refdiv: 0, fbkdiv: 20, outdiv1: 1, outdiv2: 3, outdiv5: 0 };
        let c = builder(RefClock::Crystal(25_000_000), Pll::Keep(s))
            .out_of_spec(OutOfSpec::IKnowWhatImDoing)
            .build();
        assert_eq!(c.clocks().vco_hz, 525_000_000);
    }

    #[test]
    fn outdiv5_zero_means_nfc_off() {
        let s = PllSettings { outdiv5: 0, ..MANUAL };
        builder(RefClock::Crystal(25_000_000), Pll::Set(s)).build();
    }

    #[test]
    fn a_failed_check_names_the_limit() {
        let s = PllSettings { fbkdiv: 20, ..MANUAL };
        let r = std::panic::catch_unwind(|| builder(RefClock::Crystal(25_000_000), Pll::Set(s)).build());
        let msg = *r.unwrap_err().downcast::<&str>().unwrap();
        assert!(msg.contains("VCO"), "{msg}");
    }
}
