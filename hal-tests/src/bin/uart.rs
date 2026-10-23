//! The blocking UART on UART0 (TX PF3, RX PF4) at 115,200 baud, against a host peer that echoes each byte: a byte at
//! a time in and out, then a framing error (the peer's answer to the 11th byte), reported before its character is
//! read, and an overrun (its answer to the 13th byte, sent five times back to back while nothing reads).
#![no_std]
#![no_main]

use coldfire_rt::entry;
use embassy_mcf5441x::uart::{Config, Error, Uart};
use mcf5441x_hal_tests::{check, check_eq, pass, say};

const REF_HZ: u32 = 24_000_000;
/// USR's error bits (RM Table 41-5).
const OE: u8 = 0x10;
const FE: u8 = 0x40;

#[entry]
fn main() -> ! {
    let p = embassy_mcf5441x::init(embassy_mcf5441x::Config::new(REF_HZ));
    let mut uart = Uart::new(p.UART0, p.PF3, p.PF4, Config::new(115_200)).expect("115,200 baud");

    say!("echo");
    for &b in b"0123456789" {
        uart.blocking_write(&[b]);
        let mut got = [0];
        check_eq!(uart.blocking_read(&mut got), Ok(()));
        check_eq!(got[0], b);
    }

    say!("framing error");
    uart.blocking_write(b"X");
    let mut got = [0];
    let e = uart.blocking_read(&mut got);
    check!(matches!(e, Err(Error::Status(s)) if s & FE != 0), "{:?}", e);
    // the error is reported before the character it came with, which is still there to read
    check_eq!(uart.blocking_read(&mut got), Ok(()));
    uart.blocking_write(b"Y");
    check_eq!(uart.blocking_read(&mut got), Ok(()));
    check_eq!(got[0], b'Y');

    say!("overrun");
    uart.blocking_write(b"Z");
    // six character times and more with nothing reading: the fifth answer overruns the fourth, which waits behind
    // the three in the receiver FIFO
    for i in 0..40_000u32 {
        core::hint::black_box(i);
    }
    let e = uart.blocking_read(&mut got);
    check!(matches!(e, Err(Error::Status(s)) if s & OE != 0), "{:?}", e);
    pass()
}
