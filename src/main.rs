//! # pass-pico
//!
//! This is a simple program to turn an Adafruit Trinkey QT2040 into a password entry device.
//!
//! The tool runs completely from memory, so we're able to get the status of the BOOTSEL button. The BOOTSEL button is used as almost every RP2040 board contains one.

#![no_std]
#![no_main]
#![warn(clippy::pedantic)]
#![warn(clippy::correctness)]
#![warn(clippy::perf)]
#![warn(clippy::style)]
#![warn(clippy::suspicious)]
#![warn(clippy::nursery)]
#![warn(clippy::correctness)]
#![allow(clippy::missing_panics_doc)]
#![allow(clippy::missing_errors_doc)]
#![allow(clippy::module_name_repetitions)]
#![allow(clippy::similar_names)]

pub mod codes;
pub mod keyboard;

use core::iter::once;

use adafruit_trinkey_qt2040::{
    entry,
    hal::{
        self,
        pac,
        prelude::_rphal_pio_PIOExt,
        Clock, Timer,
    },
};

use embedded_hal::digital::v2::InputPin;
use keyboard::{keyboard_println, Keyboard};
use panic_halt as _;
use smart_leds::{brightness, SmartLedsWrite, RGB8};
use usb_device::{
    class_prelude::UsbBusAllocator,
    prelude::{UsbDeviceBuilder, UsbVidPid},
};
use usbd_hid::{
    descriptor::{KeyboardReport, SerializedDescriptor},
    hid_class::HIDClass,
};
use ws2812_pio::Ws2812;

// configure to load program from flash, and run entirely from memory
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_RAM_MEMCPY;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        adafruit_trinkey_qt2040::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = hal::Sio::new(pac.SIO);
    let pins = adafruit_trinkey_qt2040::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let qspi_pins = adafruit_trinkey_qt2040::hal::gpio::qspi::Pins::new(
        pac.IO_QSPI,
        pac.PADS_QSPI,
        sio.gpio_qspi,
        &mut pac.RESETS,
    );

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut ws = Ws2812::new(
        pins.neopixel.into_mode(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    Keyboard::set_usb_bus(usb_bus);

    let bus_ref = Keyboard::get_usb_bus_ref();

    let usb_hid = HIDClass::new(bus_ref, KeyboardReport::desc(), 60);

    Keyboard::set_hid_device(usb_hid);

    // create a usb device
    let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x0000, 0x0000))
        .manufacturer("brxken128")
        // should include current version within the product
        .product("PassPico")
        .serial_number("0x0001")
        .device_class(3)
        .build();

    Keyboard::set_usb_device(usb_dev);
    Keyboard::enable_interrupts();

    let core = pac::CorePeripherals::take().unwrap();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    ws.write(brightness(once(blue()), 32)).unwrap();
    delay.delay_ms(500);
    ws.write(brightness(once(off()), 0)).unwrap();

    let bootsel = qspi_pins.cs.into_pull_up_input();

    loop {
        if bootsel.is_low().unwrap() {
            keyboard_println("text", &mut delay).unwrap();

            ws.write(brightness(once(blue()), 32)).unwrap();

            delay.delay_ms(1000);
        } else {
            ws.write(brightness(once(off()), 0)).unwrap();
        }

        delay.delay_ms(100);
    }
}

fn blue() -> RGB8 {
    (26, 94, 163).into()
}

fn off() -> RGB8 {
    (0, 0, 0).into()
}
