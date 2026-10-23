//! GPIO: typed pins on ports A–K (PA0…PK7, per package).
//!
//! A pin becomes GPIO when its direction is set (`Flex::set_as_output`/`set_as_input`; `Output::new`,
//! `Input::new`), after its level, pull and direction, so a pad leaving a peripheral never floats or glitches.
//! Pads whose PAR layout is unverified (pinmux.json: PAR_CS, PAR_IRQ0H, PAR_DSPIOWL, part of PAR_FBCTL) and the
//! MII/RMII/ULPI pads under PAR_FEC's group mode keep their mux: they must already be GPIO.
use embassy_hal_internal::PeripheralType;

pub(crate) trait SealedPin {
    /// Port index: A = 0 … K = 10.
    fn port(&self) -> u8;
    /// Bit 0..=7 within the port.
    fn bit(&self) -> u8;
    /// The pad's PAR field, if its layout is settled (`pinmux.json`); `None` leaves the pad's mux alone.
    fn par(&self) -> Option<crate::pinmux::ParField>;
}

/// A GPIO pin token.
#[allow(private_bounds)]
pub trait Pin: PeripheralType + Into<AnyPin> + SealedPin + Sized + 'static {}

/// Any pin, its port, bit and PAR field carried at run time.
#[derive(Debug, PartialEq, Eq)]
pub struct AnyPin {
    pub(crate) port: u8,
    pub(crate) bit: u8,
    pub(crate) par: Option<crate::pinmux::ParField>,
}

embassy_hal_internal::impl_peripheral!(AnyPin); // Copy, Clone, PeripheralType

impl SealedPin for AnyPin {
    fn port(&self) -> u8 {
        self.port
    }
    fn bit(&self) -> u8 {
        self.bit
    }
    fn par(&self) -> Option<crate::pinmux::ParField> {
        self.par
    }
}

impl Pin for AnyPin {}

macro_rules! impl_pin {
    ($name:ident, $port:expr, $bit:expr, $par:expr) => {
        impl $crate::gpio::SealedPin for $crate::peripherals::$name {
            fn port(&self) -> u8 {
                $port
            }
            fn bit(&self) -> u8 {
                $bit
            }
            fn par(&self) -> Option<$crate::pinmux::ParField> {
                $par
            }
        }
        impl $crate::gpio::Pin for $crate::peripherals::$name {}
        impl From<$crate::peripherals::$name> for $crate::gpio::AnyPin {
            fn from(_: $crate::peripherals::$name) -> Self {
                $crate::gpio::AnyPin { port: $port, bit: $bit, par: $par }
            }
        }
    };
}
pub(crate) use impl_pin;

use embassy_hal_internal::Peri;

// Byte register blocks from the GPIO base, one byte per port A..K (PCR: one 16-bit word per port).
const PODR: usize = 0x00;
const PDDR: usize = 0x0C;
const PPDSDR: usize = 0x18;
const PCLRR: usize = 0x24;
const PCR: usize = 0x30;

fn base() -> usize {
    crate::pac::GPIO.as_ptr() as usize
}

/// A port's byte register in `block` (PODR, PDDR, PPDSDR, PCLRR).
const fn byte_reg(base: usize, block: usize, port: u8) -> usize {
    base + block + port as usize
}

/// A port's 16-bit PCR.
const fn pcr_reg(base: usize, port: u8) -> usize {
    base + PCR + 2 * port as usize
}

/// Set pin `bit` of `port`'s pull (PCR), for drivers that own a pad without a `Flex`.
pub(crate) fn set_pull(port: u8, bit: u8, pull: Pull) {
    let a = pcr_reg(base(), port);
    let (mask, v) = pcr_bits(bit, pull);
    critical_section::with(|_| unsafe { coldfire::mmio::w16(a, (coldfire::mmio::r16(a) & !mask) | v) });
}

/// (mask, value) of a pin's PUE/PUS pair in its port's PCR.
const fn pcr_bits(bit: u8, pull: Pull) -> (u16, u16) {
    let mask = 0b11 << (2 * bit);
    let v = match pull {
        Pull::None => 0,
        Pull::Down => 0b10,
        Pull::Up => 0b11,
    };
    (mask, v << (2 * bit))
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Level {
    Low,
    High,
}

impl From<bool> for Level {
    fn from(b: bool) -> Self {
        if b { Level::High } else { Level::Low }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Pull {
    None,
    Up,
    Down,
}

/// A pin that can be input or output.
pub struct Flex<'d> {
    pin: Peri<'d, AnyPin>,
}

impl<'d> Flex<'d> {
    /// Takes the pin as a GPIO.
    ///
    /// The pin becomes GPIO when its direction is set (see the module doc for the pads that keep their mux).
    pub fn new(pin: Peri<'d, impl Pin>) -> Self {
        Self { pin: pin.into() }
    }

    fn reg(&self, block: usize) -> usize {
        byte_reg(base(), block, self.pin.port())
    }

    fn mask(&self) -> u8 {
        1 << self.pin.bit()
    }

    /// Input; then the pad is switched to GPIO (if its PAR layout is settled). An input drives nothing, so the
    /// order only matters for the pull, which is set before.
    pub fn set_as_input(&mut self) {
        let a = self.reg(PDDR);
        critical_section::with(|_| unsafe { coldfire::mmio::w8(a, coldfire::mmio::r8(a) & !self.mask()) });
        self.take_pad();
    }

    /// Output; then the pad is switched to GPIO (if its PAR layout is settled), already driving the level set
    /// before.
    pub fn set_as_output(&mut self) {
        let a = self.reg(PDDR);
        critical_section::with(|_| unsafe { coldfire::mmio::w8(a, coldfire::mmio::r8(a) | self.mask()) });
        self.take_pad();
    }

    fn take_pad(&self) {
        if let Some(f) = self.pin.par() {
            crate::pinmux::set_function(f, crate::pinmux::Function::Gpio);
        }
    }

    pub fn set_pull(&mut self, pull: Pull) {
        let a = pcr_reg(base(), self.pin.port());
        let (mask, v) = pcr_bits(self.pin.bit(), pull);
        critical_section::with(|_| unsafe { coldfire::mmio::w16(a, (coldfire::mmio::r16(a) & !mask) | v) });
    }

    /// PPDSDR: writing 1 sets that output bit, 0 leaves the others alone.
    pub fn set_high(&mut self) {
        unsafe { coldfire::mmio::w8(self.reg(PPDSDR), self.mask()) }
    }

    /// PCLRR: writing 0 clears that output bit, 1 leaves the others alone.
    pub fn set_low(&mut self) {
        unsafe { coldfire::mmio::w8(self.reg(PCLRR), !self.mask()) }
    }

    pub fn set_level(&mut self, level: Level) {
        match level {
            Level::Low => self.set_low(),
            Level::High => self.set_high(),
        }
    }

    pub fn toggle(&mut self) {
        if self.is_set_high() { self.set_low() } else { self.set_high() }
    }

    /// The pin's input state (PPDSDR read).
    pub fn is_high(&self) -> bool {
        unsafe { coldfire::mmio::r8(self.reg(PPDSDR)) & self.mask() != 0 }
    }

    pub fn is_low(&self) -> bool {
        !self.is_high()
    }

    /// The output latch (PODR).
    pub fn is_set_high(&self) -> bool {
        unsafe { coldfire::mmio::r8(self.reg(PODR)) & self.mask() != 0 }
    }
}

/// An input pin.
pub struct Input<'d> {
    pin: Flex<'d>,
}

impl<'d> Input<'d> {
    /// An input with the given pull.
    ///
    /// The pin becomes GPIO when its direction is set (see the module doc for the pads that keep their mux).
    pub fn new(pin: Peri<'d, impl Pin>, pull: Pull) -> Self {
        let mut pin = Flex::new(pin);
        pin.set_pull(pull);
        pin.set_as_input();
        Self { pin }
    }
    pub fn is_high(&self) -> bool {
        self.pin.is_high()
    }
    pub fn is_low(&self) -> bool {
        self.pin.is_low()
    }
    pub fn level(&self) -> Level {
        self.pin.is_high().into()
    }
}

/// An output pin, driven to `initial` before its direction turns to output (no glitch).
pub struct Output<'d> {
    pin: Flex<'d>,
}

impl<'d> Output<'d> {
    /// An output driven to `initial`.
    ///
    /// The pin becomes GPIO when its direction is set (see the module doc for the pads that keep their mux).
    pub fn new(pin: Peri<'d, impl Pin>, initial: Level) -> Self {
        let mut pin = Flex::new(pin);
        pin.set_level(initial);
        pin.set_as_output();
        Self { pin }
    }
    pub fn set_high(&mut self) {
        self.pin.set_high()
    }
    pub fn set_low(&mut self) {
        self.pin.set_low()
    }
    pub fn set_level(&mut self, level: Level) {
        self.pin.set_level(level)
    }
    pub fn toggle(&mut self) {
        self.pin.toggle()
    }
    pub fn is_set_high(&self) -> bool {
        self.pin.is_set_high()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn register_addresses_per_port() {
        let g = 0xEC09_4000;
        assert_eq!(byte_reg(g, PODR, 0), g); // PODR_A
        assert_eq!(byte_reg(g, PDDR, 2), g + 0x0E); // PDDR_C
        assert_eq!(byte_reg(g, PPDSDR, 2), g + 0x1A); // PPDSDR_C
        assert_eq!(byte_reg(g, PCLRR, 10), g + 0x2E); // PCLRR_K
        assert_eq!(pcr_reg(g, 1), g + 0x32); // PCR_B, 16-bit
    }

    #[test]
    fn pull_bits() {
        // PUSn = bit 2n (1 = up), PUEn = bit 2n + 1 (1 = enabled)
        assert_eq!(pcr_bits(3, Pull::Up), (0b11 << 6, 0b11 << 6));
        assert_eq!(pcr_bits(3, Pull::Down), (0b11 << 6, 0b10 << 6));
        assert_eq!(pcr_bits(0, Pull::None), (0b11, 0));
    }
}
