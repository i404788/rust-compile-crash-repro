#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]
#![feature(async_closure)]

extern crate alloc;
use core::mem::MaybeUninit;

use defmt::debug;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use embassy_usb::handlers::kbd::KbdHandler;
use embassy_usb::handlers::UsbHostHandler;
use embassy_usb::host::UsbHostBusExt;
use embassy_usb_driver::host::DeviceEvent::Connected;
use embassy_usb_driver::host::UsbHostDriver;
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    cpu_control::CpuControl,
    dma::*,
    dma_buffers,
    gpio::Io,
    otg_fs::asynch::host::{poll_usbhost, UsbHost},
    peripherals::Peripherals,
    prelude::*,
    spi::{master::Spi, SpiMode},
    timer::timg::TimerGroup,
};
use esp_println::println;

pub fn init_heap(allocator: &esp_alloc::EspHeap) {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        allocator.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

#[panic_handler]
fn panic_handler(info: &core::panic::PanicInfo) -> ! {
    println!("");
    println!("");

    println!("{:?}", info);
    if let Some(location) = info.location() {
        let (file, line, column) = (location.file(), location.line(), location.column());
        println!(
            "!! A panic occured in '{}', at line {}, column {}:",
            file, line, column
        );
    } else {
        println!("!! A panic occured at an unknown location:");
    }

    let backtrace = esp_backtrace::arch::backtrace();

    for addr in backtrace.into_iter().flatten() {
        // #[cfg(not(all(feature = "colors", feature = "println")))]
        println!("0x{:x}", addr - 3);
    }

    // println!("Reseting...");
    esp_hal::reset::software_reset();
    loop {}
}

// #[embassy_executor::task]
// async fn usb_softpoller() {
//     loop {
//         poll_usbhost();
//         Timer::after_millis(1).await;
//     }
// }

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    init_heap(&ALLOCATOR);
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    // #[cfg(feature = "log")]
    // esp_println::logger::init_logger(log::LevelFilter::Debug);

    let closure = async || {
        println!("Pre-hostbus init");
        let mut usbhost = UsbHost::new(peripherals.USB0, io.pins.gpio20, io.pins.gpio19);

        // Temporary solution until interrupts are fixed
        // let _ = spawner.spawn(usb_softpoller());

        println!("Detecting device");
        // Wait for root-port to detect device
        let speed = loop {
            match usbhost.bus.wait_for_device_event().await {
                Connected(speed) => break speed,
                _ => {}
            }
        };

        println!("Found device with speed = {:?}", speed);

        let enum_info = usbhost.bus.enumerate_root(speed, 1).await.unwrap();
        Timer::after_micros(20).await;
        let mut kbd = KbdHandler::try_register(&usbhost.bus, enum_info)
            .await
            .expect("Couldn't register keyboard");
        loop {
            let result = kbd.wait_for_event().await;
            debug!("{}", result);
        }
    };
    closure().await;
}
