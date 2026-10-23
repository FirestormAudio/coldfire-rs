//! eSDHC writes and TRIM against the host's 2048-sector card (byte i is `(i * 13 + i / 512) as u8`, erased memory
//! reading 0xFF): writes of 1, 8 and 200 sectors (the last two 64 KiB pieces) read back, a TRIM read back as 0xFF
//! between untouched neighbours, and the refusals (past the end, misaligned, an empty TRIM). The host fails the first
//! write's data once: it must succeed through its retry.
#![no_std]
#![no_main]

use coldfire_rt::entry;
use embassy_executor::Spawner;
use embassy_mcf5441x::bind_interrupts;
use embassy_mcf5441x::esdhc::{Config, Divider, Error, Esdhc, InterruptHandler};
use embassy_mcf5441x::executor::Executor;
use mcf5441x_hal_tests::{check, check_eq, pass, say};
use static_cell::StaticCell;

const REF_HZ: u32 = 24_000_000;
const SECTORS: u32 = 2048;
const MAX: usize = 200 * 512;

bind_interrupts!(struct Irqs { ESDHC => InterruptHandler; });

#[repr(C, align(4))]
struct Buf([u8; MAX + 4]);

static WRITE: StaticCell<Buf> = StaticCell::new();
static READ: StaticCell<Buf> = StaticCell::new();

/// The card's byte at `i` as the host made it.
fn original(i: usize) -> u8 {
    (i * 13 + i / 512) as u8
}

/// What the program writes at sector `lba`: unlike the original.
fn written(lba: u32, k: usize) -> u8 {
    (k as u32 * 7 + lba * 31 + 1) as u8
}

#[embassy_executor::task]
async fn main_task(mut sd: Esdhc<'static>) {
    let w = &mut WRITE.init(Buf([0; MAX + 4])).0;
    let r = &mut READ.init(Buf([0; MAX + 4])).0;
    for (lba, n) in [(3u32, 1usize), (100, 8), (500, 200)] {
        say!("write {} at {}", n, lba);
        let len = n * 512;
        for (k, b) in w[..len].iter_mut().enumerate() {
            *b = written(lba, k);
        }
        check_eq!(sd.write(lba, &w[..len]).await, Ok(()));
        check_eq!(sd.read(lba, &mut r[..len]).await, Ok(()));
        check!(r[..len] == w[..len], "sectors {} to {} read back", lba, lba as usize + n - 1);
    }

    say!("trim 3 at 101");
    check_eq!(sd.erase(101, 3).await, Ok(()));
    check_eq!(sd.read(100, &mut r[..5 * 512]).await, Ok(()));
    for (k, &b) in r[..5 * 512].iter().enumerate() {
        let want = match k / 512 {
            1..=3 => 0xFF,
            s => written(100, s * 512 + k % 512),
        };
        check_eq!(b, want);
    }

    say!("refusals");
    check_eq!(sd.write(SECTORS - 1, &w[..1024]).await, Err(Error::OutOfRange));
    check_eq!(sd.erase(SECTORS - 1, 2).await, Err(Error::OutOfRange));
    check_eq!(sd.erase(10, 0).await, Err(Error::OutOfRange));
    check_eq!(sd.write(10, &w[1..513]).await, Err(Error::Misaligned));
    // sector 10, untouched by all of that
    check_eq!(sd.read(10, &mut r[..512]).await, Ok(()));
    for (k, &b) in r[..512].iter().enumerate() {
        check_eq!(b, original(10 * 512 + k));
    }
    pass()
}

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[entry]
fn main() -> ! {
    let p = embassy_mcf5441x::init(embassy_mcf5441x::Config::new(REF_HZ));
    // SAFETY: start-up done; the time driver's interrupt bounds the bring-up's waits
    unsafe { coldfire::interrupt::enable() };
    let config = Config::new(Divider::new(0x80, 3), Divider::new(0x01, 1));
    let sd = match Esdhc::new(p.ESDHC, p.DMA_CH59, p.PG5, p.PG6, p.PG7, p.PF0, p.PF1, p.PF2, Irqs, config) {
        Ok(sd) => sd,
        Err(_) => mcf5441x_hal_tests::fail("bring-up"),
    };
    EXECUTOR.init(Executor::new()).run(|spawner: Spawner| {
        spawner.spawn(main_task(sd).unwrap());
    })
}
