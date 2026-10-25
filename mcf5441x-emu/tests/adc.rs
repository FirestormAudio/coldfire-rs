//! The minimal ADC (RM ch. 29): reset values, read-back registers, and host-set results.

use mcf5441x_emu::soc::{Adc, ChipOptions, Mcf5441x};
use mcf5441x_emu::{Machine, Size};

const ADC: u32 = 0xFC09_4000;

#[test]
fn results_are_the_hosts_and_the_rest_reads_back() {
    let mut m = Machine::new();
    let chip = Mcf5441x::build(&mut m, ChipOptions::default());
    // RM Table 29-2 reset values; results 0
    assert_eq!(m.access(ADC, Size::Word, None), Ok(0x5005));
    assert_eq!(m.access(ADC + 0x32, Size::Word, None), Ok(0x7FF8));
    for n in 0..8 {
        assert_eq!(
            m.access(ADC + 0x12 + 2 * n, Size::Word, None),
            Ok(0),
            "RSLT{n}"
        );
    }
    // the host sets a result; a program's write does not change it, and a control register reads back
    m.device_mut::<Adc>(chip.adc).set_result(1, 0x0123);
    assert_eq!(m.access(ADC + 0x14, Size::Word, None), Ok(0x0123));
    m.access(ADC + 0x14, Size::Word, Some(0xFFFF)).unwrap();
    assert_eq!(m.access(ADC + 0x14, Size::Word, None), Ok(0x0123));
    m.access(ADC + 2, Size::Word, Some(0x1234)).unwrap();
    assert_eq!(m.access(ADC + 2, Size::Word, None), Ok(0x1234));
}

#[test]
fn rgpio_inputs_are_the_hosts_and_outputs_the_latch() {
    use mcf5441x_emu::soc::Rgpio;
    const R: u32 = 0x8C00_0000;
    let mut m = Machine::new();
    let chip = Mcf5441x::build(&mut m, ChipOptions::default());
    m.device_mut::<Rgpio>(chip.rgpio).set_input(0x00F0);
    // reset: all pins disabled, DATA reads the (zero) latch
    assert_eq!(m.access(R + 2, Size::Word, None), Ok(0));
    // enable the low byte, pins 0-3 outputs: they show the latch, pins 4-7 the host's levels
    m.access(R + 4, Size::Word, Some(0x00FF)).unwrap();
    m.access(R, Size::Word, Some(0x000F)).unwrap();
    m.access(R + 2, Size::Word, Some(0x0005)).unwrap();
    assert_eq!(m.access(R + 2, Size::Word, None), Ok(0x00F5));
    // SET, CLR (clears the bits written 0) and TOG act on the latch; the read views mirror DATA and DIR
    m.access(R + 0xA, Size::Word, Some(0x0002)).unwrap();
    assert_eq!(m.access(R + 6, Size::Word, None), Ok(0x00F7));
    m.access(R + 6, Size::Word, Some(0xFFFE)).unwrap();
    m.access(R + 0xE, Size::Word, Some(0x0008)).unwrap();
    assert_eq!(m.access(R + 0xE, Size::Word, None), Ok(0x00FE));
    assert_eq!(m.access(R + 8, Size::Word, None), Ok(0x000F));
}
