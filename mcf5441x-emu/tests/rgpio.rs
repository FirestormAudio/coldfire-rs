//! The minimal RGPIO (RM ch. 16): host-set inputs, a latch for the outputs.

use mcf5441x_emu::soc::{ChipOptions, Mcf5441x};
use mcf5441x_emu::{Machine, Size};

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

#[test]
fn writes_past_the_registers_are_dropped() {
    const R: u32 = 0x8C00_0000;
    let mut m = Machine::new();
    Mcf5441x::build(&mut m, ChipOptions::default());
    m.access(R + 4, Size::Word, Some(0x00FF)).unwrap();
    m.access(R + 0x10, Size::Word, Some(0xFFFF)).unwrap();
    m.access(R + 0x12, Size::Word, Some(0xFFFF)).unwrap();
    assert_eq!(m.access(R, Size::Word, None), Ok(0));
    assert_eq!(m.access(R + 2, Size::Word, None), Ok(0));
    assert_eq!(m.access(R + 4, Size::Word, None), Ok(0x00FF));
}
