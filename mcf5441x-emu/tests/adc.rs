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
