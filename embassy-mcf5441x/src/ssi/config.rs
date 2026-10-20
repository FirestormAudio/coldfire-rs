//! The SSI's configuration and the register words it gives (RM 35.3), built with the PAC's field setters.
use crate::pac::ssi0::{regs, vals};

/// How frames are framed. I2S modes fix the frame to two words; network mode has `Config::slots` words per frame.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Mode {
    /// I2S master: the SSI drives the bit clock and frame sync.
    I2sMaster,
    /// I2S slave: the bit clock and frame sync come in.
    I2sSlave,
    /// Network mode (TDM): `slots` words per frame, the SSI driving the clocks.
    Network,
}

/// The SSI baud clock's source (MISCCR SSInSRC, RM 10.3.4): the SSI_CLKIN pin, or the PLL divided by `div` (CDRH
/// SSInDIV, 2..=255; RM 10.3.6).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ClockSource {
    Clkin,
    Pll { div: u8 },
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum WordLength {
    W8,
    W10,
    W12,
    W16,
    W18,
    W20,
    W22,
    W24,
}

impl WordLength {
    pub const fn bits(self) -> u32 {
        match self {
            WordLength::W8 => 8,
            WordLength::W10 => 10,
            WordLength::W12 => 12,
            WordLength::W16 => 16,
            WordLength::W18 => 18,
            WordLength::W20 => 20,
            WordLength::W22 => 22,
            WordLength::W24 => 24,
        }
    }
}

/// An SSI setup: transmit and receive share the frame (synchronous mode, CR SYN), from FIFO 0 each.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Config {
    pub mode: Mode,
    pub word: WordLength,
    /// Words per frame (CCR DC + 1): 2 for I2S, 1..=32 in network mode.
    pub slots: u8,
    /// The bit-clock dividers (CCR PM, PSR, DIV2): bit clock = baud clock / ((DIV2 + 1) × (7 × PSR + 1) × (PM + 1) × 2).
    pub pm: u8,
    pub psr: bool,
    pub div2: bool,
    pub clock: ClockSource,
    /// FIFO watermarks (FCSR TFWM0/RFWM0, 1..=15): DMA requests while the TX FIFO has this many free slots, or the RX
    /// FIFO this many words.
    pub tx_watermark: u8,
    pub rx_watermark: u8,
    /// Sign-extend received words to 32 bits (RCR RXEXT).
    pub rx_sign_extend: bool,
    /// Receive on the bit clock's falling edge (RCR RSCKP).
    pub rx_falling: bool,
}

impl Config {
    /// I2S master, `slots` words of `word` bits, PM 1, from SSI_CLKIN, watermarks 2, RX sign-extended.
    pub const fn i2s_master(word: WordLength, slots: u8) -> Self {
        Self {
            mode: Mode::I2sMaster,
            word,
            slots,
            pm: 1,
            psr: false,
            div2: false,
            clock: ClockSource::Clkin,
            tx_watermark: 2,
            rx_watermark: 2,
            rx_sign_extend: true,
            rx_falling: false,
        }
    }
}

/// The words `Ssi::new` writes (CR without SSI_EN, TE or RE).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) struct Words {
    pub cr: u32,
    pub tcr: u32,
    pub rcr: u32,
    pub ccr: u32,
    pub fcsr: u32,
    pub ier: u32,
}

pub(crate) const fn words(c: &Config) -> Words {
    if c.slots == 0 || c.slots > 32 {
        panic!("ssi: slots must be 1 to 32 (CCR DC)");
    }
    if c.tx_watermark == 0 || c.tx_watermark > 15 || c.rx_watermark == 0 || c.rx_watermark > 15 {
        panic!("ssi: FIFO watermarks are 1 to 15");
    }
    if let ClockSource::Pll { div } = c.clock {
        if div < 2 {
            panic!("ssi: ClockSource::Pll div must be 2 to 255 (0 and 1 mean divide by 254)");
        }
    }
    let master = !matches!(c.mode, Mode::I2sSlave);
    let mut cr = regs::Cr(0);
    cr.set_syn(vals::Syn::from_bits(1));
    match c.mode {
        Mode::I2sMaster => cr.set_i2s(vals::I2s::from_bits(0b01)),
        Mode::I2sSlave => cr.set_i2s(vals::I2s::from_bits(0b10)),
        Mode::Network => cr.set_net(vals::Net::from_bits(1)),
    }
    let mut tcr = regs::Tcr(0);
    tcr.set_tfen0(vals::Tfen0::from_bits(1));
    tcr.set_tsckp(true);
    if master {
        tcr.set_tfdir(vals::Tfdir::from_bits(1));
        tcr.set_txdir(vals::Txdir::from_bits(1));
    }
    let mut rcr = regs::Rcr(0);
    rcr.set_rfen0(vals::Rfen0::from_bits(1));
    rcr.set_rxext(vals::Rxext::from_bits(c.rx_sign_extend as u8));
    rcr.set_rsckp(c.rx_falling);
    let mut ccr = regs::Ccr(0);
    ccr.set_wl((c.word.bits() / 2 - 1) as u8);
    ccr.set_dc(c.slots - 1);
    ccr.set_pm(c.pm);
    ccr.set_psr(c.psr);
    ccr.set_div2(c.div2);
    let mut fcsr = regs::Fcsr(0);
    fcsr.set_tfwm0(c.tx_watermark);
    fcsr.set_rfwm0(c.rx_watermark);
    let mut ier = regs::Ier(0);
    ier.set_tdmae(true);
    ier.set_rdmae(true);
    Words { cr: cr.0, tcr: tcr.0, rcr: rcr.0, ccr: ccr.0, fcsr: fcsr.0, ier: ier.0 }
}

/// MISCCR's SSInSRC field for SSI `instance`: (mask, value).
pub(crate) const fn misccr_src(instance: u8, c: ClockSource) -> (u16, u16) {
    let bit = 1u16 << (4 + instance);
    (bit, if matches!(c, ClockSource::Pll { .. }) { bit } else { 0 })
}

/// CDRH's SSInDIV field for SSI `instance`: (mask, value).
pub(crate) const fn cdrh_div(instance: u8, div: u8) -> (u16, u16) {
    if instance == 0 { (0xFF00, (div as u16) << 8) } else { (0x00FF, div as u16) }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;

    #[test]
    fn i2s_master_24_bit_stereo_words() {
        let w = words(&Config::i2s_master(WordLength::W24, 2));
        assert_eq!(w.cr, 1 << 4 | 0b01 << 5);                                  // SYN, I2S master; SSI_EN off
        assert_eq!(w.tcr, 1 << 7 | 1 << 6 | 1 << 5 | 1 << 3);                  // TFEN0, TFDIR, TXDIR, TSCKP
        assert_eq!(w.rcr, 1 << 7 | 1 << 10);                                    // RFEN0, RXEXT
        assert_eq!(w.ccr, 11 << 13 | 1 << 8 | 1);                              // WL 24 bits (2 × (11 + 1)), DC 2 slots, PM 1
        assert_eq!(w.fcsr, 2 | 2 << 4);                                         // TFWM0 2, RFWM0 2
        assert_eq!(w.ier, 1 << 20 | 1 << 22);                                   // TDMAE, RDMAE
    }

    #[test]
    fn word_lengths_map_to_wl_field() {
        let bits = [(WordLength::W8, 8), (WordLength::W10, 10), (WordLength::W12, 12), (WordLength::W16, 16),
                    (WordLength::W18, 18), (WordLength::W20, 20), (WordLength::W22, 22), (WordLength::W24, 24)];
        for (w, n) in bits {
            assert_eq!(words(&Config::i2s_master(w, 2)).ccr >> 13 & 0xF, n / 2 - 1, "{n}");
        }
    }

    #[test]
    fn network_mode_sets_net_and_slot_count() {
        let c = Config { mode: Mode::Network, ..Config::i2s_master(WordLength::W16, 4) };
        let w = words(&c);
        assert_eq!(w.cr & (1 << 3 | 0b11 << 5), 1 << 3);                       // NET, I2S off
        assert_eq!(w.ccr >> 8 & 0x1F, 3);
    }

    #[test]
    fn slave_clears_the_direction_bits() {
        let c = Config { mode: Mode::I2sSlave, ..Config::i2s_master(WordLength::W24, 2) };
        let w = words(&c);
        assert_eq!(w.cr >> 5 & 0b11, 0b10);
        assert_eq!(w.tcr & (1 << 6 | 1 << 5), 0);                               // TFDIR, TXDIR: clocks come in
    }

    #[test]
    fn clock_source_field() {
        assert_eq!(misccr_src(1, ClockSource::Clkin), (1 << 5, 0));
        assert_eq!(misccr_src(0, ClockSource::Pll { div: 8 }), (1 << 4, 1 << 4));
        assert_eq!(cdrh_div(0, 8), (0xFF00, 8 << 8));
        assert_eq!(cdrh_div(1, 8), (0x00FF, 8));
    }

    #[test]
    fn slots_out_of_range_is_refused() {
        for n in [0u8, 33] {
            let r = std::panic::catch_unwind(|| words(&Config::i2s_master(WordLength::W16, n)));
            assert!(r.unwrap_err().downcast::<&str>().unwrap().contains("slots"), "{n}");
        }
        let r = std::panic::catch_unwind(|| words(&Config { clock: ClockSource::Pll { div: 1 }, ..Config::i2s_master(WordLength::W16, 2) }));
        assert!(r.unwrap_err().downcast::<&str>().unwrap().contains("div"));
    }
}
