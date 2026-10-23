//! DSPI0 (SCK PD6, SOUT PD4, SIN PD5) as a blocking master, against a host part on PCS0 (PD7) that answers each byte
//! with its complement: a transaction of a write and a read, a transfer, the same with `wide_reads` (reads through
//! CTAR1), then the bare bus with a GPIO select on PC2. The host checks the frame log and the select's GPIO changes.
#![no_std]
#![no_main]

use coldfire_rt::entry;
use embassy_mcf5441x::gpio::{Level, Output};
use embassy_mcf5441x::spi::{Config, Ctar, Mode, PcsDevice, Spi};
use embedded_hal::spi::{Operation, SpiBus, SpiDevice};
use mcf5441x_hal_tests::{check_eq, pass, say};

const REF_HZ: u32 = 24_000_000;

#[entry]
fn main() -> ! {
    let mut p = embassy_mcf5441x::init(embassy_mcf5441x::Config::new(REF_HZ));
    let ctar = Ctar::new(Mode::Mode0);

    say!("8-bit frames");
    {
        let mut dev = PcsDevice::new_blocking(
            p.DSPI0.reborrow(),
            p.PD6.reborrow(),
            p.PD4.reborrow(),
            p.PD5.reborrow(),
            p.PD7.reborrow(),
            Config::new(ctar),
        );
        let mut r = [0u8; 2];
        check_eq!(dev.transaction(&mut [Operation::Write(&[1, 2, 3]), Operation::Read(&mut r)]), Ok(()));
        check_eq!(r, [0xFF, 0xFF]);
        let mut t = [0u8; 2];
        check_eq!(dev.transfer(&mut t, &[0x10, 0x20]), Ok(()));
        check_eq!(t, [0xEF, 0xDF]);
    }

    say!("wide reads");
    {
        let mut config = Config::new(ctar);
        config.wide_reads = true;
        let mut dev = PcsDevice::new_blocking(
            p.DSPI0.reborrow(),
            p.PD6.reborrow(),
            p.PD4.reborrow(),
            p.PD5.reborrow(),
            p.PD7.reborrow(),
            config,
        );
        let mut r = [0u8; 4];
        check_eq!(dev.transaction(&mut [Operation::Write(&[1, 2, 3]), Operation::Read(&mut r)]), Ok(()));
        check_eq!(r, [0xFF; 4]);
    }

    say!("bare bus");
    let mut cs = Output::new(p.PC2, Level::High);
    let mut bus = Spi::new_blocking(p.DSPI0, p.PD6, p.PD4, p.PD5, Config::new(ctar));
    cs.set_low();
    check_eq!(bus.write(&[0xA5, 0x5A]), Ok(()));
    let mut b = [1u8, 2];
    check_eq!(bus.transfer_in_place(&mut b), Ok(()));
    check_eq!(bus.flush(), Ok(()));
    cs.set_high();
    pass()
}
