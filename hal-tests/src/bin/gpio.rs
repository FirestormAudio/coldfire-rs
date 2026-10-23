//! GPIO: outputs set, cleared and toggled, inputs read with each pull and with the host driving them, and a `Flex`
//! turned from input to output. PA0 and PB0 are outputs; PC2 is left undriven (it reads its pull); PD0 is driven
//! high by the host for the whole run.
#![no_std]
#![no_main]

use coldfire_rt::entry;
use embassy_mcf5441x::gpio::{Flex, Input, Level, Output, Pull};
use mcf5441x_hal_tests::{check, pass, say};

const REF_HZ: u32 = 24_000_000;

#[entry]
fn main() -> ! {
    let p = embassy_mcf5441x::init(embassy_mcf5441x::Config::new(REF_HZ));

    say!("step 1");
    let mut a = Output::new(p.PA0, Level::High);
    check!(a.is_set_high());
    a.set_low();
    check!(!a.is_set_high());
    a.toggle();
    check!(a.is_set_high());

    say!("step 2");
    let b = Output::new(p.PB0, Level::Low);
    check!(!b.is_set_high());

    say!("step 3");
    let mut c = Flex::new(p.PC2);
    c.set_pull(Pull::Up);
    c.set_as_input();
    check!(c.is_high(), "pull up");
    c.set_pull(Pull::Down);
    check!(c.is_low(), "pull down");
    c.set_pull(Pull::None);
    check!(c.is_high(), "no pull");

    say!("step 4");
    let d = Input::new(p.PD0, Pull::Down);
    check!(d.is_high(), "the host's drive beats the pull");

    say!("step 5");
    c.set_level(Level::Low);
    c.set_as_output();
    check!(!c.is_set_high());
    check!(c.is_low());
    pass()
}
