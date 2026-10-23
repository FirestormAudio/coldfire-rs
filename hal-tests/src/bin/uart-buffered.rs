//! The buffered UART on UART0 (TX PF3, RX PF4) at 115,200 baud: receive DMA into the 1 KB ring and the TXRDY
//! interrupt, on the thread executor. 200 bytes written and flushed; three 40-byte commands sent with `write_atomic`
//! from a `second()` handle while another task writes, each arriving whole; then `!`, after which the host sends
//! 1,500 bytes (the ring wraps), read back in pieces and checked.
#![no_std]
#![no_main]

use coldfire_rt::entry;
use embassy_executor::Spawner;
use embassy_mcf5441x::bind_interrupts;
use embassy_mcf5441x::executor::Executor;
use embassy_mcf5441x::peripherals::UART0;
use embassy_mcf5441x::uart::{BufferedInterruptHandler, BufferedUart, BufferedUartRx, BufferedUartTx, Config, RxDmaInterruptHandler};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embedded_io_async::{Read, Write};
use mcf5441x_hal_tests::{check_eq, pass, say};
use static_cell::StaticCell;

const REF_HZ: u32 = 24_000_000;
/// The bytes the host sends after `!`: byte i is `(i * 7 + i / 256) as u8`.
const STREAM: usize = 1500;

bind_interrupts!(struct Irqs {
    UART0 => BufferedInterruptHandler<UART0>;
    EDMA_INTR_INT02 => RxDmaInterruptHandler<UART0>;
});

static COMMANDS_SENT: Signal<CriticalSectionRawMutex, ()> = Signal::new();

#[embassy_executor::task]
async fn commands(mut tx: BufferedUartTx<'static>) {
    for k in 0..3u8 {
        tx.write_atomic(&[b'A' + k; 40]).await;
    }
    COMMANDS_SENT.signal(());
}

#[embassy_executor::task]
async fn main_task(spawner: Spawner, mut rx: BufferedUartRx<'static>, mut tx: BufferedUartTx<'static>) {
    say!("write and flush");
    let block: [u8; 200] = core::array::from_fn(|i| i as u8);
    check_eq!(tx.write_all(&block).await, Ok(()));
    check_eq!(tx.flush().await, Ok(()));

    say!("whole commands");
    spawner.spawn(commands(tx.second()).unwrap());
    for _ in 0..10 {
        check_eq!(tx.write_all(b"0123456789").await, Ok(()));
    }
    COMMANDS_SENT.wait().await;
    check_eq!(tx.flush().await, Ok(()));

    say!("the ring wraps");
    check_eq!(tx.write_all(b"!").await, Ok(()));
    let mut buf = [0u8; 100];
    let mut seen = 0;
    while seen < STREAM {
        let want = buf.len().min(STREAM - seen);
        let n = rx.read(&mut buf[..want]).await.unwrap_or(0);
        for (k, &b) in buf[..n].iter().enumerate() {
            let i = seen + k;
            check_eq!(b, (i * 7 + i / 256) as u8);
        }
        seen += n;
    }
    pass()
}

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[entry]
fn main() -> ! {
    let p = embassy_mcf5441x::init(embassy_mcf5441x::Config::new(REF_HZ));
    let uart = BufferedUart::new(p.UART0, p.DMA_CH2, p.PF3, p.PF4, Irqs, Config::new(115_200)).expect("115,200 baud");
    let (rx, tx) = uart.split();
    // SAFETY: start-up done; the UART's interrupts are bound
    unsafe { coldfire::interrupt::enable() };
    EXECUTOR.init(Executor::new()).run(|spawner: Spawner| {
        spawner.spawn(main_task(spawner, rx, tx).unwrap());
    })
}
