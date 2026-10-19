//! Blocking UART0 (TX PF3, RX PF4) at 115,200 baud: echo every byte back.
#![no_std]
#![no_main]

use coldfire_rt::entry;
use embassy_mcf5441x::uart::{Config, Uart};

const REF_HZ: u32 = 24_000_000; // the PLL reference: change for your board's crystal

#[entry]
fn main() -> ! {
    let p = embassy_mcf5441x::init(embassy_mcf5441x::Config::new(REF_HZ));
    let mut uart = Uart::new(p.UART0, p.PF3, p.PF4, Config::new(115_200)).expect("115,200 baud");
    uart.blocking_write(b"uart-echo: type something\r\n");
    let mut b = [0u8; 1];
    loop {
        if uart.blocking_read(&mut b).is_ok() {
            uart.blocking_write(&b);
        }
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        coldfire::asm::stop();
    }
}
