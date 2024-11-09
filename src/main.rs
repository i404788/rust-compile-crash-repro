#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]
#![feature(async_closure)]
// Disable unused import warn
#![allow(unused_imports)]

extern crate alloc;
use core::mem::MaybeUninit;

// For the MGHz macro
use esp_hal::prelude::*;

use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::gpio::{GpioPin, Io, Output};
use esp_println::println;

use alloc::format;
use embassy_usb::handlers::{kbd::KbdHandler, UsbHostHandler};
use embassy_usb::host::UsbHostBusExt;
use embassy_usb_driver::host::DeviceEvent::Connected;
use embassy_usb_driver::host::UsbHostDriver;
use esp_hal::otg_fs::asynch::host::UsbHost;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

pub fn init_heap(allocator: &esp_alloc::EspHeap) {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        allocator.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}

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

use anyhow::{anyhow, bail, ensure, Ok};
use core::str::FromStr;
use defmt::unwrap;
use embassy_time::{with_timeout, Delay};
use esp_hal::{dma_buffers, gpio::Input, spi::SpiMode};

#[esp_hal_embassy::main]
async fn main(spawner: embassy_executor::Spawner) {
    {
        init_heap(&ALLOCATOR);
        let p = esp_hal::init(esp_hal::Config::default());
        let timg0 = esp_hal::timer::timg::TimerGroup::new(p.TIMG0);
        esp_hal_embassy::init(timg0.timer0);
        let io = Io::new(p.GPIO, p.IO_MUX);
        let usb_a = async || -> anyhow::Result<()> {
            {
                defmt::info!("Pre-hostbus init");

                use embassy_usb::handlers::kbd::KbdHandler;
                use embassy_usb::handlers::UsbHostHandler;
                use embassy_usb::host::UsbHostBusExt;
                use embassy_usb_driver::host::UsbHostDriver;
                let mut usbhost = esp_hal::otg_fs::asynch::host::UsbHost::new(
                    p.USB0,
                    io.pins.gpio20,
                    io.pins.gpio19,
                );
                defmt::info!("Detecting device. Waiting 10s before quitting...");
                let speed = loop {
                    match with_timeout(Duration::from_secs(10), usbhost.bus.wait_for_device_event())
                        .await
                    {
                        core::result::Result::Ok(
                            embassy_usb_driver::host::DeviceEvent::Connected(speed),
                        ) => {
                            break speed;
                        }
                        core::result::Result::Ok(..) => {
                            bail!("Got disconnected before connected event!");
                        }
                        core::result::Result::Err(e) => {
                            bail!("Error waiting for device: {:?}", e);
                        }
                    }
                };
                defmt::info!("Found device with speed = {}", speed);
                let enum_info = match usbhost.bus.enumerate_root(speed, 1).await {
                    core::result::Result::Ok(info) => info,
                    Err(e) => {
                        esp_println::println!("Failed to enumerate: {:?}", e);
                        bail!("Failed to enumerate");
                    }
                };
                Timer::after_micros(20).await;
                let mut _kbd = KbdHandler::try_register(&usbhost.bus, enum_info)
                    .await
                    .map_err(|e| anyhow!("Failed register: {:?}", e))?;
                defmt::info!("Registered succesfully keyboard");
                return anyhow::Ok(());
            }
        };
        if let Err(e) = usb_a().await {
            use crate::alloc::string::ToString;
            let error = e.to_string();
            defmt::info!("[Test {}] Failed: {}", "usb_a", error);
        };
    }
}
