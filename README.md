stm32f042xx-usb
===============

This is a port of mvirkkunen's [stm32f103xx-usb](https://github.com/mvirkkunen/stm32f103xx-usb) crate.
It implements hardware support for the stm32f042 for the [usb-device](https://github.com/mvirkkunen/usb-device) crate.
As the USB interface is similar between STM32F04x, STM32F072 and STM32F078 t might be possible that this crate works for this to.

So far the crate has been tested on:
 - STM32F042K6
 - STM32F072C8T6

See the examples for an example of a custom class and a minimalistic USB serial port device.
