//! A USB CDC-ACM device on the OTG port and its ULPI PHY (the MCF54415-18's 12 ULPI pads), echoing every packet.
#![no_std]
#![no_main]

use coldfire_rt::entry;
use embassy_executor::Spawner;
use embassy_mcf5441x::executor::Executor;
use embassy_mcf5441x::usb::{self, Driver, Ulpi};
use embassy_mcf5441x::bind_interrupts;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::{Builder, UsbDevice};
use static_cell::StaticCell;

const REF_HZ: u32 = 24_000_000; // the PLL reference: change for your board's crystal

bind_interrupts!(struct Irqs { USBOTG => usb::InterruptHandler; });

#[embassy_executor::task]
async fn device(mut dev: UsbDevice<'static, Driver<'static>>) {
    dev.run().await
}

#[embassy_executor::task]
async fn echo(mut class: CdcAcmClass<'static, Driver<'static>>) {
    let mut buf = [0u8; 512];
    loop {
        class.wait_connection().await;
        while let Ok(n) = class.read_packet(&mut buf).await {
            if class.write_packet(&buf[..n]).await.is_err() {
                break;
            }
            // a full-size packet doesn't end a transfer: without a zero-length packet after it, a host read waiting
            // for the end (Linux's cdc_acm holds a full packet) sees nothing until more arrives
            if n == class.max_packet_size() as usize && class.write_packet(&[]).await.is_err() {
                break;
            }
        }
    }
}

static EXECUTOR: StaticCell<Executor> = StaticCell::new();
static STATE: StaticCell<State> = StaticCell::new();
static CFG: StaticCell<[u8; 128]> = StaticCell::new();
static BOS: StaticCell<[u8; 32]> = StaticCell::new();
static CTRL: StaticCell<[u8; 64]> = StaticCell::new();

#[entry]
fn main() -> ! {
    let p = embassy_mcf5441x::init(embassy_mcf5441x::Config::new(REF_HZ));
    let ulpi = Ulpi {
        clkin: p.PC5, dir: p.PD2, nxt: p.PE7, stp: p.PJ0,
        data0: p.PK4, data1: p.PK5, data2: p.PK1, data3: p.PK2, data4: p.PK7, data5: p.PK6, data6: p.PK3, data7: p.PK0,
    };
    let driver = Driver::new(p.USBOTG, Irqs, ulpi, usb::Config::default());
    // pid.codes test IDs: use your own for anything that leaves your desk
    let mut config = embassy_usb::Config::new(0x1209, 0x0001);
    config.product = Some("usb-cdc example");
    config.max_packet_size_0 = 64;
    let mut builder = Builder::new(driver, config, CFG.init([0; 128]), BOS.init([0; 32]), &mut [], CTRL.init([0; 64]));
    let class = CdcAcmClass::new(&mut builder, STATE.init(State::new()), 512);
    let dev = builder.build();
    // SAFETY: start-up, before anything shares state with an interrupt
    unsafe { coldfire::interrupt::enable() };
    EXECUTOR.init(Executor::new()).run(|spawner: Spawner| {
        spawner.spawn(device(dev).unwrap());
        spawner.spawn(echo(class).unwrap());
    })
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        coldfire::asm::stop();
    }
}
