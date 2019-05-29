use core::cell::RefCell;
use core::mem;
use usb_device::{Result, UsbDirection, UsbError};
use usb_device::bus::{UsbBusAllocator, PollResult};
use usb_device::endpoint::{EndpointType, EndpointAddress};
use cortex_m::asm;
use cortex_m::interrupt;
use cortex_m::interrupt::Mutex;
extern crate stm32f0xx_hal as hal;
use hal::prelude::*;
use hal::rcc;
use hal::gpio::{Pin, Output, PushPull};
use hal::stm32::{USB, RCC, CRS};
use crate::endpoint::{NUM_ENDPOINTS, Endpoint, EndpointStatus, calculate_count_rx};

struct Reset {
    delay: u32,
    pin: Mutex<RefCell<Pin<Output<PushPull>>>>,
}

/// USB peripheral driver for STM32F103 microcontrollers.
pub struct UsbBus {
    regs: Mutex<USB>,
    endpoints: [Endpoint; NUM_ENDPOINTS],
    next_ep_mem: usize,
    max_endpoint: usize,
    reset: Option<Reset>,
}

impl UsbBus {
    fn new(regs: USB, rcc: &mut rcc::Rcc, reset: Option<Reset>) -> UsbBusAllocator<Self> {
        // TODO: apb1.enr is not public, figure out how this should really interact with the HAL
        // crate
        let _ = rcc;
        interrupt::free(|_| {
            let rcc = unsafe { (&*RCC::ptr()) };
            rcc.cr2.modify(|_,w| w.hsi48on().set_bit());
            // Wait for internel 48MHz osci to get stable
            while rcc.cr2.read().hsi48rdy().bit_is_clear() {};

            rcc.apb1enr.modify(|_, w| w
                               .usben().set_bit() // enable clock for USB interface
                               .crsen().set_bit() // enable clock for Clock Recovery System
                               );

            rcc.cfgr3.modify(|_, w| w.usbsw().clear_bit()); // select internel 48Mhz osci for USB

            // TODO pass crs register bank
            let crs = unsafe { (&*CRS::ptr()) };

            crs.cr.modify(|_,w| w.autotrimen().set_bit()); // enable trim for internel 48MHz osci
            crs.cr.modify(|_,w| w.cen().set_bit()); // enable trim for internel 48MHz osci
        });

        let bus = UsbBus {
            regs: Mutex::new(regs),
            next_ep_mem: Endpoint::MEM_START,
            max_endpoint: 0,
            endpoints: unsafe {
                let mut endpoints: [Endpoint; NUM_ENDPOINTS] = mem::uninitialized();

                for i in 0..NUM_ENDPOINTS {
                    endpoints[i] = Endpoint::new(i as u8);
                }

                endpoints
            },
            reset,
        };

        UsbBusAllocator::new(bus)
    }

    /// Constructs a new USB peripheral driver.
    pub fn usb(regs: USB, apb1: &mut rcc::Rcc) -> UsbBusAllocator<Self> {
        UsbBus::new(regs, apb1, None)
    }

    /// Constructs a new USB peripheral driver with the "reset" method enabled.
    pub fn usb_with_reset(
        regs: USB,
        apb1: &mut rcc::Rcc,
        clocks: &rcc::Clocks,
        reset_pin: Pin<Output<PushPull>>) -> UsbBusAllocator<Self>
    {
        UsbBus::new(
            regs,
            apb1,
            Some(Reset {
                delay: clocks.sysclk().0,
                pin: Mutex::new(RefCell::new(reset_pin)),
            }))
    }

    fn alloc_ep_mem(next_ep_mem: &mut usize, size: usize) -> Result<usize> {
        assert!(size & 1 == 0);

        let addr = *next_ep_mem;
        if addr + size > Endpoint::MEM_SIZE {
            return Err(UsbError::EndpointMemoryOverflow);
        }

        *next_ep_mem += size;

        Ok(addr)
    }
}

impl usb_device::bus::UsbBus for UsbBus {
    fn alloc_ep(
        &mut self,
        ep_dir: UsbDirection,
        ep_addr: Option<EndpointAddress>,
        ep_type: EndpointType,
        max_packet_size: u16,
        _interval: u8) -> Result<EndpointAddress>
    {
        for index in ep_addr.map(|a| a.index()..a.index()+1).unwrap_or(1..NUM_ENDPOINTS) {
            let ep = &mut self.endpoints[index];

            match ep.ep_type() {
                None => { ep.set_ep_type(ep_type); },
                Some(t) if t != ep_type => { continue; },
                _ => { },
            };

            match ep_dir {
                UsbDirection::Out if !ep.is_out_buf_set() => {
                    let (out_size, bits) = calculate_count_rx(max_packet_size as usize)?;

                    let addr = Self::alloc_ep_mem(&mut self.next_ep_mem, out_size)?;

                    ep.set_out_buf(addr, (out_size, bits));

                    return Ok(EndpointAddress::from_parts(index, ep_dir));
                },
                UsbDirection::In if !ep.is_in_buf_set() => {
                    let size = (max_packet_size as usize + 1) & !0x01;

                    let addr = Self::alloc_ep_mem(&mut self.next_ep_mem, size as usize)?;

                    ep.set_in_buf(addr, size as usize);

                    return Ok(EndpointAddress::from_parts(index, ep_dir));
                }
                _ => { }
            }
        }

        Err(match ep_addr {
            Some(_) => UsbError::InvalidEndpoint,
            None => UsbError::EndpointOverflow,
        })
    }

    fn enable(&mut self) {
        let mut max = 0;
        for (index, ep) in self.endpoints.iter().enumerate() {
            if ep.is_out_buf_set() || ep.is_in_buf_set() {
                max = index;
            }
        }

        self.max_endpoint = max;

        interrupt::free(|cs| {
            let regs = self.regs.borrow(cs);

            regs.cntr.modify(|_, w| w.pdwn().clear_bit());

            // There is a chip specific startup delay. For STM32F103xx it's 1µs and this should wait for
            // at least that long.
            //delay(72); // Not supported for cortex m3
            for _ in 0..(1 << 8) { //FIXME get something that will wait the correct time
                asm::nop();
            }

            regs.btable.modify(|_, w| { w.btable().bits(0) });
            regs.cntr.modify(|_, w| w
                .fres().clear_bit()
                .resetm().set_bit()
                .suspm().set_bit()
                .wkupm().set_bit()
                .errm().set_bit()
                .pmaovrm().set_bit()
                .ctrm().set_bit());
            regs.istr.modify(|_, w| unsafe { w.bits(0) });
            regs.bcdr.modify(|_, w| w.dppu().set_bit()); // Enable embedded Pull-up
        });
    }

    fn reset(&self) {
        interrupt::free(|cs| {
            let regs = self.regs.borrow(cs);

            regs.istr.modify(|_, w| unsafe { w.bits(0) });
            regs.daddr.modify(|_, w| { w.ef().set_bit().add().bits(0) });

            for ep in self.endpoints.iter() {
                ep.configure(cs);
            }
        });
    }

    fn set_device_address(&self, addr: u8) {
        interrupt::free(|cs| {
            self.regs.borrow(cs).daddr.modify(|_, w| { w.add().bits(addr as u8) });
        });
    }

    fn poll(&self) -> PollResult {
        interrupt::free(|cs| {
            let regs = self.regs.borrow(cs);

            let istr = regs.istr.read();

            if istr.wkup().bit_is_set() {
                regs.istr.modify(|_, w| w.wkup().clear_bit());

                let fnr = regs.fnr.read();
                //let bits = (fnr.rxdp().bit_is_set() as u8) << 1 | (fnr.rxdm().bit_is_set() as u8);

                match (fnr.rxdp().bit_is_set(), fnr.rxdm().bit_is_set()) {
                    (false, false) | (false, true) => {
                        PollResult::Resume
                    },
                    _ => {
                        // Spurious wakeup event caused by noise
                        PollResult::Suspend
                    }
                }
            } else if istr.reset().bit_is_set() {
                regs.istr.modify(|_, w| w.reset().clear_bit());

                PollResult::Reset
            } else if istr.susp().bit_is_set() {
                regs.istr.modify(|_, w| w.susp().clear_bit());

                PollResult::Suspend
            } else if istr.ctr().bit_is_set() {
                let mut ep_out = 0;
                let mut ep_in_complete = 0;
                let mut ep_setup = 0;
                let mut bit = 1;

                for ep in &self.endpoints[0..=self.max_endpoint] {
                    let v = ep.read_reg();

                    if v.ctr_rx().bit_is_set() {
                        ep_out |= bit;

                        if v.setup().bit_is_set() {
                            ep_setup |= bit;
                        }
                    }

                    if v.ctr_tx().bit_is_set() {
                        ep_in_complete |= bit;

                        interrupt::free(|cs| {
                            ep.clear_ctr_tx(cs);
                        });
                    }

                    bit <<= 1;
                }

                PollResult::Data { ep_out, ep_in_complete, ep_setup }
            } else {
                PollResult::None
            }
        })
    }

    fn write(&self, ep_addr: EndpointAddress, buf: &[u8]) -> Result<usize> {
        if !ep_addr.is_in() {
            return Err(UsbError::InvalidEndpoint);
        }

        self.endpoints[ep_addr.index()].write(buf)
    }

    fn read(&self, ep_addr: EndpointAddress, buf: &mut [u8]) -> Result<usize> {
        if !ep_addr.is_out() {
            return Err(UsbError::InvalidEndpoint);
        }

        self.endpoints[ep_addr.index()].read(buf)
    }

    fn set_stalled(&self, ep_addr: EndpointAddress, stalled: bool) {
        interrupt::free(|cs| {
            if self.is_stalled(ep_addr) == stalled {
                return
            }

            let ep = &self.endpoints[ep_addr.index()];

            match (stalled, ep_addr.direction()) {
                (true, UsbDirection::In) => ep.set_stat_tx(cs, EndpointStatus::Stall),
                (true, UsbDirection::Out) => ep.set_stat_rx(cs, EndpointStatus::Stall),
                (false, UsbDirection::In) => ep.set_stat_tx(cs, EndpointStatus::Nak),
                (false, UsbDirection::Out) => ep.set_stat_rx(cs, EndpointStatus::Valid),
            };
        });
    }

    fn is_stalled(&self, ep_addr: EndpointAddress) -> bool {
        let ep = &self.endpoints[ep_addr.index()];
        let reg_v = ep.read_reg();

        let status = match ep_addr.direction() {
            UsbDirection::In => reg_v.stat_tx().bits(),
            UsbDirection::Out => reg_v.stat_rx().bits(),
        };

        status == (EndpointStatus::Stall as u8)
    }

    fn suspend(&self) {
        interrupt::free(|cs| {
             self.regs.borrow(cs).cntr.modify(|_, w| w
                .fsusp().set_bit()
                .lpmode().set_bit());
        });
    }

    fn resume(&self) {
        interrupt::free(|cs| {
            self.regs.borrow(cs).cntr.modify(|_, w| w
                .fsusp().clear_bit()
                .lpmode().clear_bit());
        });
    }

    fn force_reset(&self) -> Result<()> {
        interrupt::free(|cs| {
            let regs = self.regs.borrow(cs);

            match self.reset {
                Some(ref reset) => {
                    let pdwn = regs.cntr.read().pdwn().bit_is_set();
                    regs.cntr.modify(|_, w| w.pdwn().set_bit());

                    reset.pin.borrow(cs).borrow_mut().set_low();
                    //delay(reset.delay);
                    for _ in 0..(1 << 8) { //FIXME get something that will wait the correct time
                        asm::nop();
                    }

                    regs.cntr.modify(|_, w| w.pdwn().bit(pdwn));

                    Ok(())
                },
                None => {
                    regs.bcdr.modify(|_, w| w.dppu().clear_bit()); // Disable embedded Pull-up
//                    delay(100);
                    for _ in 0..(1 << 10) { //FIXME get something that will wait the correct time
                        asm::nop();
                    }
                    regs.bcdr.modify(|_, w| w.dppu().set_bit()); // Enable embedded Pull-up
                    Ok(())
                },
            }
        })
    }
}
