//! The eSDHC with an eMMC behind it (CLK PG5, CMD PG6, DAT0 PG7, DAT1 PF0, DAT2 PF1, DAT3 PF2): bring-up, the
//! sector count and EXT_CSD, then multi-block reads at a few LBAs checked against the host card's pattern (byte i of
//! the card is `(i * 13 + i / 512) as u8`), on the thread executor.
#![no_std]
#![no_main]

use coldfire_rt::entry;
use embassy_executor::Spawner;
use embassy_mcf5441x::bind_interrupts;
use embassy_mcf5441x::esdhc::{Config, Divider, Esdhc, InterruptHandler};
use embassy_mcf5441x::executor::Executor;
use mcf5441x_hal_tests::{check_eq, pass, say};
use static_cell::StaticCell;

const REF_HZ: u32 = 24_000_000;
/// The host card's size in sectors.
const SECTORS: u32 = 2048;

bind_interrupts!(struct Irqs { ESDHC => InterruptHandler; });

/// A sector buffer the eDMA writes: word aligned.
#[repr(C, align(4))]
struct Buf([u8; 8 * 512]);

static BUF: StaticCell<Buf> = StaticCell::new();

#[embassy_executor::task]
async fn main_task(mut sd: Esdhc<'static>) {
    let buf = &mut BUF.init(Buf([0; 8 * 512])).0;
    for (lba, n) in [(0u32, 1usize), (7, 2), (2040, 8)] {
        say!("read {} at {}", n, lba);
        check_eq!(sd.read(lba, &mut buf[..n * 512]).await, Ok(()));
        for (k, &b) in buf[..n * 512].iter().enumerate() {
            let i = lba as usize * 512 + k;
            check_eq!(b, (i * 13 + i / 512) as u8);
        }
    }
    pass()
}

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[entry]
fn main() -> ! {
    let p = embassy_mcf5441x::init(embassy_mcf5441x::Config::new(REF_HZ));
    // SAFETY: start-up done; the time driver's interrupt bounds the bring-up's waits
    unsafe { coldfire::interrupt::enable() };
    say!("bring-up");
    let config = Config::new(Divider::new(0x80, 3), Divider::new(0x01, 1));
    let sd = Esdhc::new(p.ESDHC, p.DMA_CH59, p.PG5, p.PG6, p.PG7, p.PF0, p.PF1, p.PF2, Irqs, config);
    let sd = match sd {
        Ok(sd) => sd,
        Err(_) => mcf5441x_hal_tests::fail("bring-up"),
    };
    check_eq!(sd.sectors(), SECTORS);
    let sec_count = u32::from_le_bytes([sd.ext_csd(212), sd.ext_csd(213), sd.ext_csd(214), sd.ext_csd(215)]);
    check_eq!(sec_count, SECTORS);
    EXECUTOR.init(Executor::new()).run(|spawner: Spawner| {
        spawner.spawn(main_task(sd).unwrap());
    })
}
