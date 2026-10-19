//! CTAR presets: SPI mode, baud rate and the three delays as one CTAR value (field layout RM §40.3.3). The frame
//! size is the driver's: 8-bit frames through CTAR0, 16-bit read frames through CTAR1 (`frames`).

/// SCK polarity and phase: mode = CPOL × 2 + CPHA.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Mode {
    /// SCK idles low; data is captured on the leading edge.
    Mode0 = 0,
    /// SCK idles low; data changes on the leading edge and is captured on the trailing edge.
    Mode1 = 1,
    /// SCK idles high; data is captured on the leading edge.
    Mode2 = 2,
    /// SCK idles high; data changes on the leading edge and is captured on the trailing edge.
    Mode3 = 3,
}

/// Baud-rate prescaler (PBR).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum BaudPrescaler {
    Div2 = 0,
    Div3 = 1,
    Div5 = 2,
    Div7 = 3,
}

/// Delay prescaler (PCSSCK, PASC, PDT).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum DelayPrescaler {
    Div1 = 0,
    Div3 = 1,
    Div5 = 2,
    Div7 = 3,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ConfigError {
    /// Even the slowest setting (bus / 7 / 32768) is faster than asked, or 0 Hz was asked for.
    FrequencyTooLow,
}

const PRESCALERS: [BaudPrescaler; 4] = [BaudPrescaler::Div2, BaudPrescaler::Div3, BaudPrescaler::Div5, BaudPrescaler::Div7];
const PBR_DIV: [u32; 4] = [2, 3, 5, 7];
/// BR scaler by field value: 2, 4, 6, 8, then powers of two up to 32768.
const BR_DIV: [u32; 16] = [2, 4, 6, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768];

const DBR: u32 = 1 << 31;
const FMSZ: u32 = 0xF << 27;
const LSBFE: u32 = 1 << 24;
const PBR: u32 = 0b11 << 16;
const BR: u32 = 0xF;

/// One CTAR's mode, baud rate and delays.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Ctar(u32);

impl Ctar {
    /// `mode`, MSB first, the fastest clock (bus / 2 / 2) and the shortest delays (2 bus clocks each).
    pub const fn new(mode: Mode) -> Self {
        Ctar((mode as u32) << 25)
    }

    /// SCK = bus / `pbr` × (1 + `double`) / BR scaler(`br`), where the scaler is 2, 4, 6, 8, then powers of
    /// two up to 32768. Doubling (DBR) with a prescaler other than 2 gives an uneven duty cycle.
    pub const fn baud(self, pbr: BaudPrescaler, br: u8, double: bool) -> Self {
        assert!(br < 16, "BR is 4 bits");
        Ctar((self.0 & !(DBR | PBR | BR)) | (double as u32) << 31 | (pbr as u32) << 16 | br as u32)
    }

    /// Chip select asserted to the first SCK edge: `pre` × 2^(`scaler` + 1) bus clocks (PCSSCK, CSSCK).
    pub const fn cs_to_sck(self, pre: DelayPrescaler, scaler: u8) -> Self {
        self.delay(22, 12, pre, scaler)
    }

    /// Last SCK edge to the chip select negated, in the same units (PASC, ASC).
    pub const fn after_sck(self, pre: DelayPrescaler, scaler: u8) -> Self {
        self.delay(20, 8, pre, scaler)
    }

    /// Chip select negated to the next assertion, in the same units (PDT, DT).
    pub const fn after_transfer(self, pre: DelayPrescaler, scaler: u8) -> Self {
        self.delay(18, 4, pre, scaler)
    }

    const fn delay(self, pre_at: u32, scaler_at: u32, pre: DelayPrescaler, scaler: u8) -> Self {
        assert!(scaler < 16, "delay scalers are 4 bits");
        Ctar((self.0 & !(0b11 << pre_at | 0xF << scaler_at)) | (pre as u32) << pre_at | (scaler as u32) << scaler_at)
    }

    /// Least significant bit first (LSBFE).
    pub const fn lsb_first(self) -> Self {
        Ctar(self.0 | LSBFE)
    }

    /// The fastest SCK no faster than `hz` on a `bus_hz` bus clock. Doubling is only tried with prescaler 2, which
    /// keeps the duty cycle at 50/50.
    pub const fn with_frequency(self, bus_hz: u32, hz: u32) -> Result<Self, ConfigError> {
        if hz == 0 {
            return Err(ConfigError::FrequencyTooLow);
        }
        // divisor = num / den = pbr × br / (1 + dbr); best = (num, den, pbr index, br, dbr)
        let mut best: Option<(u32, u32, usize, u8, bool)> = None;
        let mut p = 0;
        while p < 4 {
            let mut b = 0;
            while b < 16 {
                let mut d = 0;
                while d < if p == 0 { 2 } else { 1 } {
                    let (num, den) = (PBR_DIV[p] * BR_DIV[b], 1 + d);
                    // bus × den / num <= hz, in u64 products (no division)
                    let fits = bus_hz as u64 * den as u64 <= hz as u64 * num as u64;
                    let faster = match best {
                        None => true,
                        Some((bn, bd, ..)) => (num as u64) * (bd as u64) < (bn as u64) * (den as u64),
                    };
                    if fits && faster {
                        best = Some((num, den, p, b as u8, d == 1));
                    }
                    d += 1;
                }
                b += 1;
            }
            p += 1;
        }
        match best {
            Some((_, _, p, b, d)) => Ok(self.baud(PRESCALERS[p], b, d)),
            None => Err(ConfigError::FrequencyTooLow),
        }
    }

    /// SCK on a `bus_hz` bus clock (rounded down; u32 arithmetic).
    pub const fn sck_hz(self, bus_hz: u32) -> u32 {
        let num = PBR_DIV[((self.0 & PBR) >> 16) as usize] * BR_DIV[(self.0 & BR) as usize];
        if self.0 & DBR != 0 { bus_hz / num * 2 + bus_hz % num * 2 / num } else { bus_hz / num }
    }

    /// The CTAR word with `bits`-bit frames (4..=16).
    pub(crate) const fn frames(self, bits: u8) -> u32 {
        (self.0 & !FMSZ) | ((bits as u32 - 1) << 27)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const BUS: u32 = 125_000_000;

    #[test]
    fn a_mode3_ctar_packs_its_fields() {
        // DBR, FMSZ, CPOL|CPHA, PBR /5, BR 1 (scaler 4), PDT /3, DT 1 (RM 40.3.3's CTAR layout)
        let t = Ctar::new(Mode::Mode3).baud(BaudPrescaler::Div5, 1, true).after_transfer(DelayPrescaler::Div3, 1);
        let fields = 1 << 31 | 0b11 << 25 | 0b10 << 16 | 1 | 0b01 << 18 | 1 << 4;
        assert_eq!(t.frames(8), fields | 7 << 27);
        assert_eq!(t.frames(16), fields | 15 << 27);
        assert_eq!(t.sck_hz(BUS), 12_500_000); // 125 MHz / 5 × 2 / 4
    }

    #[test]
    fn modes_set_cpol_and_cpha() {
        assert_eq!(Ctar::new(Mode::Mode0).frames(8), 7 << 27);
        assert_eq!(Ctar::new(Mode::Mode1).frames(8), 7 << 27 | 1 << 25);
        assert_eq!(Ctar::new(Mode::Mode2).frames(8), 7 << 27 | 1 << 26);
        assert_eq!(Ctar::new(Mode::Mode3).frames(8), 7 << 27 | 0b11 << 25);
    }

    #[test]
    fn each_builder_writes_only_its_fields() {
        let base = Ctar::new(Mode::Mode0).frames(8);
        assert_eq!(Ctar::new(Mode::Mode0).cs_to_sck(DelayPrescaler::Div7, 15).frames(8) ^ base, 0b11 << 22 | 0xF << 12);
        assert_eq!(Ctar::new(Mode::Mode0).after_sck(DelayPrescaler::Div5, 9).frames(8) ^ base, 0b10 << 20 | 9 << 8);
        assert_eq!(Ctar::new(Mode::Mode0).after_transfer(DelayPrescaler::Div3, 1).frames(8) ^ base, 0b01 << 18 | 1 << 4);
        assert_eq!(Ctar::new(Mode::Mode0).lsb_first().frames(8) ^ base, 1 << 24);
        // a second baud() replaces the first entirely
        let t = Ctar::new(Mode::Mode0).baud(BaudPrescaler::Div7, 15, true).baud(BaudPrescaler::Div2, 1, false);
        assert_eq!(t.frames(8) ^ base, 1);
    }

    #[test]
    fn frequency_is_the_fastest_not_above_the_request() {
        // bus / 3 × 2 would be 41.7 MHz, but doubling with prescaler 3 has an uneven duty cycle: bus / 4
        assert_eq!(Ctar::new(Mode::Mode3).with_frequency(BUS, 41_666_667).unwrap().sck_hz(BUS), 31_250_000);
        assert_eq!(Ctar::new(Mode::Mode0).with_frequency(BUS, 100_000_000).unwrap().sck_hz(BUS), 62_500_000);
        assert_eq!(Ctar::new(Mode::Mode0).with_frequency(BUS, 1_000_000).unwrap().sck_hz(BUS), 976_562); // bus / 2 / 64
        // the mode and delays survive
        let t = Ctar::new(Mode::Mode3).after_transfer(DelayPrescaler::Div1, 2);
        let f = t.with_frequency(BUS, 1_000_000).unwrap();
        assert_eq!(f.frames(8) & !(DBR | PBR | BR), t.frames(8));
    }

    #[test]
    fn too_slow_or_zero_is_refused() {
        assert_eq!(Ctar::new(Mode::Mode0).with_frequency(BUS, 0), Err(ConfigError::FrequencyTooLow));
        // the slowest setting is bus / 7 / 32768 = 544 Hz
        assert_eq!(Ctar::new(Mode::Mode0).with_frequency(BUS, 500), Err(ConfigError::FrequencyTooLow));
        assert_eq!(Ctar::new(Mode::Mode0).with_frequency(BUS, 545).unwrap().sck_hz(BUS), 544);
    }
}
