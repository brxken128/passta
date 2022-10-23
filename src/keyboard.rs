use adafruit_trinkey_qt2040::hal;
use adafruit_trinkey_qt2040::hal::pac::interrupt;
use adafruit_trinkey_qt2040::hal::usb::UsbBus;
use cortex_m::delay::Delay;
use usb_device::{class_prelude::UsbBusAllocator, prelude::UsbDevice};
use usbd_hid::{descriptor::KeyboardReport, hid_class::HIDClass};

use crate::keycodes::{char_to_keycode, get_modifier, KEY_ENTER, KEY_MOD_NONE, KEY_NONE};

pub static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;
pub static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;
pub static mut USB_HID: Option<HIDClass<hal::usb::UsbBus>> = None;

pub struct Keyboard;

impl Keyboard {
    pub fn set_usb_device(dev: UsbDevice<'static, UsbBus>) {
        unsafe {
            USB_DEVICE = Some(dev);
        }
    }

    pub fn set_usb_bus(bus: UsbBusAllocator<UsbBus>) {
        unsafe {
            USB_BUS = Some(bus);
        }
    }

    pub fn set_hid_device(dev: HIDClass<'static, UsbBus>) {
        unsafe {
            USB_HID = Some(dev);
        }
    }

    pub fn enable_interrupts() {
        unsafe {
            hal::pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
        }
    }

    #[must_use]
    pub fn get_usb_bus_ref() -> &'static UsbBusAllocator<UsbBus> {
        unsafe { USB_BUS.as_ref().unwrap() }
    }

    pub fn push_report(report: KeyboardReport) -> Result<usize, usb_device::UsbError> {
        critical_section::with(|_| unsafe { USB_HID.as_mut().map(|hid| hid.push_input(&report)) })
            .unwrap()
    }

    // keep track of the last char to reduce amount of blank reports sent
    // this lets us type strings faster
    // not too sure how much of a perf impact Option has here (it's probably negligible)
    pub fn print(text: &str, delay: &mut Delay) -> Result<(), usb_device::UsbError> {
        let mut last_char: Option<char> = None;
        for c in text.chars() {
            let modifier = get_modifier(c);

            // if current char differs from the previous, we can send a new report instantly
            // if they match, we need a blank report to say that it's another input of the char
            if let Some(lc) = last_char {
                if c == lc {
                    Self::push_report(BLANK_REPORT)?;

                    delay.delay_ms(35);
                }
            }

            let codes = [
                char_to_keycode(c),
                KEY_NONE,
                KEY_NONE,
                KEY_NONE,
                KEY_NONE,
                KEY_NONE,
            ];

            let report = KeyboardReport {
                modifier,
                reserved: 0x00,
                leds: 0x00,
                keycodes: codes,
            };

            Self::push_report(report)?;

            last_char = Some(c);

            delay.delay_ms(35);
        }

        Self::push_report(BLANK_REPORT)?;

        Ok(())
    }

    pub fn println(text: &str, delay: &mut Delay) -> Result<(), usb_device::UsbError> {
        Self::print(text, delay)?;

        // we need this delay as `keyboard_write()` does not include an ending one
        delay.delay_ms(35);

        let codes = [KEY_ENTER, KEY_NONE, KEY_NONE, KEY_NONE, KEY_NONE, KEY_NONE];

        let report = KeyboardReport {
            modifier: KEY_MOD_NONE,
            reserved: 0x00,
            leds: 0x00,
            keycodes: codes,
        };

        Self::push_report(report)?;

        delay.delay_ms(35);

        Self::push_report(BLANK_REPORT)?;

        Ok(())
    }
}

// this is called whenever the USB device generates an interrupt request
#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let usb_hid = USB_HID.as_mut().unwrap();
    usb_dev.poll(&mut [usb_hid]);
}

const BLANK_KEYCODES: [u8; 6] = [KEY_NONE, KEY_NONE, KEY_NONE, KEY_NONE, KEY_NONE, KEY_NONE];

const BLANK_REPORT: KeyboardReport = KeyboardReport {
    modifier: KEY_MOD_NONE,
    reserved: 0x00,
    leds: 0x00,
    keycodes: BLANK_KEYCODES,
};
