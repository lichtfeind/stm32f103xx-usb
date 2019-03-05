#![no_std]
#![no_main]

extern crate panic_semihosting;

use cortex_m_rt::entry;
use stm32f0xx_hal as hal;
use hal::{prelude::*, stm32};

use usb_device::test_class::TestClass;
use stm32f042xx_usb::UsbBus;

#[entry]
fn main() -> ! {
    let mut dp = stm32::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.configure().sysclk(48.mhz()).freeze(&mut dp.FLASH);

    let usb_bus = UsbBus::usb(
        dp.USB, &mut rcc);

    let mut test = TestClass::new(&usb_bus);

    let mut usb_dev = { test.make_device(&usb_bus) };

    usb_dev.force_reset().expect("reset failed");

    loop {
        if usb_dev.poll(&mut [&mut test]) {
            test.poll();
        }
    }
}
