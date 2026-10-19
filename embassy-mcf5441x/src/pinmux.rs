//! Pin multiplexing (PAR registers) and slew rate (SRCR registers), RM ch. 15.
//!
//! A pad's 2-bit PAR field selects one of the columns of Table 15-2: GPIO, the primary function (the signal
//! the pad is named after), alternate 1 or alternate 2. Drivers set the fields for the pins they're given.
//! The GPIO driver switches a pad to GPIO when its direction is set, for pads with a settled layout
//! (embassy-mcf5441x/pinmux.json); the rest keep their mux.

/// What a 2-bit PAR field selects (Table 15-10; note 01 is alternate 2 and 10 alternate 1).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Function {
    Gpio = 0b00,
    Alt2 = 0b01,
    Alt1 = 0b10,
    Primary = 0b11,
}

/// A PAR field: its register's offset from the GPIO base, its lowest bit and its width (1 or 2 bits).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct ParField {
    pub(crate) offset: u8,
    pub(crate) shift: u8,
    pub(crate) width: u8,
}

impl ParField {
    pub(crate) const fn new(offset: u8, shift: u8, width: u8) -> Self {
        Self { offset, shift, width }
    }
}

// PAR registers (offsets from the GPIO base 0xEC09_4000); embassy-mcf5441x/pinmux.json maps each pin to its field.
// Each is used only on the parts/packages that have its pads (PAR_FEC: only by `usb`, on the 256-pin parts), and
// some (the unverified layouts) not at all yet: hence the allows.
#[allow(dead_code)]
pub(crate) const PAR_FBCTL: u8 = 0x48;
#[allow(dead_code)]
pub(crate) const PAR_BE: u8 = 0x49;
#[allow(dead_code)]
pub(crate) const PAR_CS: u8 = 0x4A;
#[allow(dead_code)]
pub(crate) const PAR_CANI2C: u8 = 0x4B;
#[allow(dead_code)]
pub(crate) const PAR_IRQ0H: u8 = 0x4C;
#[allow(dead_code)]
pub(crate) const PAR_IRQ0L: u8 = 0x4D;
#[allow(dead_code)]
pub(crate) const PAR_DSPIOWH: u8 = 0x4E;
#[allow(dead_code)]
pub(crate) const PAR_DSPIOWL: u8 = 0x4F;
#[allow(dead_code)]
pub(crate) const PAR_TIMER: u8 = 0x50;
#[allow(dead_code)]
pub(crate) const PAR_UART2: u8 = 0x51;
#[allow(dead_code)]
pub(crate) const PAR_UART1: u8 = 0x52;
#[allow(dead_code)]
pub(crate) const PAR_UART0: u8 = 0x53;
#[allow(dead_code)]
pub(crate) const PAR_SDHCH: u8 = 0x54;
#[allow(dead_code)]
pub(crate) const PAR_SDHCL: u8 = 0x55;
#[allow(dead_code)]
pub(crate) const PAR_SIMP0H: u8 = 0x56;
#[allow(dead_code)]
pub(crate) const PAR_SIMP0L: u8 = 0x57;
#[allow(dead_code)]
pub(crate) const PAR_SSI0H: u8 = 0x58;
#[allow(dead_code)]
pub(crate) const PAR_SSI0L: u8 = 0x59;
#[allow(dead_code)]
pub(crate) const PAR_DEBUGH1: u8 = 0x5A;
#[allow(dead_code)]
pub(crate) const PAR_DEBUGH0: u8 = 0x5B;
#[allow(dead_code)]
pub(crate) const PAR_DEBUGL: u8 = 0x5C;
#[allow(dead_code)]
pub(crate) const PAR_FEC: u8 = 0x5E;

/// `old` with `field` set to `f`. A 1-bit field has only GPIO (0) and primary (1) (Table 15-11).
pub(crate) const fn par_update(old: u8, field: ParField, f: Function) -> u8 {
    // the generated tables never ask for it: an alternate on a 1-bit field would quietly mean GPIO
    debug_assert!(field.width == 2 || matches!(f, Function::Gpio | Function::Primary), "1-bit PAR field: GPIO or primary only");
    let mask = ((1u8 << field.width) - 1) << field.shift;
    let v = if field.width == 1 { matches!(f, Function::Primary) as u8 } else { f as u8 };
    (old & !mask) | ((v << field.shift) & mask)
}

fn gpio_base() -> usize {
    crate::pac::GPIO.as_ptr() as usize
}

/// Select `f` on one pad (read-modify-write: other pads in the register keep their functions).
pub(crate) fn set_function(field: ParField, f: Function) {
    let a = gpio_base() + field.offset as usize;
    critical_section::with(|_| unsafe { coldfire::mmio::w8(a, par_update(coldfire::mmio::r8(a), field, f)) });
}

/// The PAR_FEC group modes (bits 3:0) that give the nine non-RMII MII0 pads to the USB OTG's ULPI. PAR_FEC has no
/// per-pad fields: one mode covers all 18 MII0 pads, so it also sets the nine RMII0 pads (PI0, PI1, PJ1..PJ7) to RMII0
/// (or, with MDC/MDIO left out, two of them to GPIO), which the USB driver doesn't own: a board can't use those pads
/// as GPIO alongside USB.
#[cfg(any(feature = "mcf54415", feature = "mcf54416", feature = "mcf54417", feature = "mcf54418"))]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum UlpiMode {
    /// The RMII0 pads stay RMII0, MDC/MDIO included.
    WithRmii0 = 0b1000,
    /// As `WithRmii0`, but MDC and MDIO are GPIO (0x0A).
    WithRmii0NoMdio = 0b1010,
}

/// Write PAR_FEC's group mode.
#[cfg(any(feature = "mcf54415", feature = "mcf54416", feature = "mcf54417", feature = "mcf54418"))]
pub(crate) fn set_fec_mode(m: UlpiMode) {
    let a = gpio_base() + PAR_FEC as usize;
    critical_section::with(|_| unsafe { coldfire::mmio::w8(a, m as u8) });
}

/// SRCR slew rate (2-bit SREx fields: 00 lowest … 11 highest; 11 lets the interface run at its maximum).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Slew {
    Lowest = 0b00,
    Low = 0b01,
    High = 0b10,
    Highest = 0b11,
}

/// One SRCR field: the pads it covers are named after it (Table 15-15). UART4/5/6 use the UART0/1/2 pads'
/// field, UART3 `Dspi0`, UART7 `Ssi0`, UART8 `I2c0`, UART9 `Can1`.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum SlewGroup {
    Fb1,
    Fb2,
    Fb3,
    Fb4,
    Fb5,
    Owdat,
    Dspi0,
    Can1,
    I2c0,
    Irq0,
    Timer0,
    Timer1,
    Timer2,
    Timer3,
    Uart0,
    Uart1,
    Uart2,
    Rmii0,
    Rmii1,
    Sdhc,
    Simp0,
    Ssi0,
}

impl SlewGroup {
    /// (SRCR register offset from the GPIO base, field shift).
    pub(crate) const fn field(self) -> (u8, u8) {
        match self {
            SlewGroup::Fb1 => (0x64, 0),
            SlewGroup::Fb2 => (0x65, 0),
            SlewGroup::Fb3 => (0x66, 0),
            SlewGroup::Fb5 => (0x67, 2),
            SlewGroup::Fb4 => (0x67, 0),
            SlewGroup::Owdat => (0x68, 2),
            SlewGroup::Dspi0 => (0x68, 0),
            SlewGroup::Can1 => (0x69, 2),
            SlewGroup::I2c0 => (0x69, 0),
            SlewGroup::Irq0 => (0x6A, 0),
            SlewGroup::Timer3 => (0x6B, 6),
            SlewGroup::Timer2 => (0x6B, 4),
            SlewGroup::Timer1 => (0x6B, 2),
            SlewGroup::Timer0 => (0x6B, 0),
            SlewGroup::Uart2 => (0x6C, 4),
            SlewGroup::Uart1 => (0x6C, 2),
            SlewGroup::Uart0 => (0x6C, 0),
            SlewGroup::Rmii0 => (0x6D, 2),
            SlewGroup::Rmii1 => (0x6D, 0),
            SlewGroup::Sdhc => (0x6E, 0),
            SlewGroup::Simp0 => (0x6F, 0),
            SlewGroup::Ssi0 => (0x70, 0),
        }
    }
}

/// Set a pad group's slew rate (read-modify-write: the register's other field keeps its value).
pub fn set_slew(group: SlewGroup, slew: Slew) {
    let (offset, shift) = group.field();
    let a = gpio_base() + offset as usize;
    critical_section::with(|_| unsafe {
        let old = coldfire::mmio::r8(a);
        coldfire::mmio::w8(a, (old & !(0b11 << shift)) | ((slew as u8) << shift));
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg(any(feature = "mcf54415", feature = "mcf54416", feature = "mcf54417", feature = "mcf54418"))]
    #[test]
    fn ulpi_modes_are_the_par_fec_encodings() {
        assert_eq!((UlpiMode::WithRmii0 as u8, UlpiMode::WithRmii0NoMdio as u8), (0b1000, 0b1010));
    }

    #[test]
    fn par_update_sets_one_two_bit_field() {
        assert_eq!(par_update(0x00, ParField::new(0x4B, 0, 2), Function::Alt1), 0b10);
        assert_eq!(par_update(0x00, ParField::new(0x4B, 6, 2), Function::Primary), 0b1100_0000);
        assert_eq!(par_update(0xFF, ParField::new(0x4B, 2, 2), Function::Gpio), 0b1111_0011);
    }

    #[test]
    fn par_update_keeps_the_other_fields() {
        let tx9 = ParField::new(0x4B, 2, 2);
        let rx9 = ParField::new(0x4B, 0, 2);
        assert_eq!(par_update(par_update(0xA0, tx9, Function::Alt1), rx9, Function::Alt1), 0xAA);
        let tx8 = ParField::new(0x4B, 6, 2);
        let rx8 = ParField::new(0x4B, 4, 2);
        assert_eq!(par_update(par_update(0x0A, tx8, Function::Alt1), rx8, Function::Alt1), 0xAA);
    }

    #[test]
    #[should_panic(expected = "1-bit PAR field")]
    fn an_alternate_on_a_one_bit_field_is_refused() {
        par_update(0x55, ParField::new(0x5B, 2, 1), Function::Alt1);
    }

    #[test]
    fn par_update_one_bit_fields() {
        // PAR_DEBUGH0 resets to 0x55 (every PST pad primary): GPIO on PST1 (bit 2) clears that bit only
        let pst1 = ParField::new(0x5B, 2, 1);
        assert_eq!(par_update(0x55, pst1, Function::Gpio), 0x51);
        assert_eq!(par_update(0x51, pst1, Function::Primary), 0x55);
        // FB_R/W (bit 2 of PAR_FBCTL) next to the 2-bit FB_TA field at 1:0
        assert_eq!(par_update(0b0000_0111, ParField::new(0x48, 2, 1), Function::Gpio), 0b0000_0011);
    }

    #[test]
    fn slew_fields() {
        assert_eq!(SlewGroup::Uart0.field(), (0x6C, 0));
        assert_eq!(SlewGroup::Uart2.field(), (0x6C, 4));
        assert_eq!(SlewGroup::Can1.field(), (0x69, 2));
        assert_eq!(SlewGroup::I2c0.field(), (0x69, 0));
        assert_eq!(SlewGroup::Timer3.field(), (0x6B, 6));
        assert_eq!(SlewGroup::Rmii0.field(), (0x6D, 2));
        assert_eq!(SlewGroup::Ssi0.field(), (0x70, 0));
    }
}
