//! # PassPico
//! 
//! This is a simple program to turn an Adafruit Trinkey QT2040 into a password entry device.
//! 
//! The tool runs completely from memory, so we're able to get the status of the BOOTSEL button. The BOOTSEL button is used as almost every RP2040 board contains one.

#![no_std]
#![no_main]

pub mod codes;

use core::iter::once;

use adafruit_trinkey_qt2040::entry;
use adafruit_trinkey_qt2040::hal::Timer;
use adafruit_trinkey_qt2040::hal::pac::interrupt;
use cortex_m::delay::Delay;
use embedded_hal::digital::v2::InputPin;
use panic_halt as _;
use adafruit_trinkey_qt2040::hal::prelude::*;
use adafruit_trinkey_qt2040::hal::pac;
use adafruit_trinkey_qt2040::hal;
use smart_leds::brightness;
use smart_leds::SmartLedsWrite;
use smart_leds::RGB8;
use usb_device::{class_prelude::*, prelude::*};
use usbd_hid::descriptor::KeyboardReport;
use usbd_hid::descriptor::generator_prelude::*;
use usbd_hid::hid_class::HIDClass;
use ws2812_pio::Ws2812;
use crate::codes::*;

static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;
static mut USB_HID: Option<HIDClass<hal::usb::UsbBus>> = None;

// configure to load program from flash, and run entirely from memory
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_RAM_MEMCPY;

#[entry]
fn main() -> ! {
    unsafe {
        hal::sio::spinlock_reset();
    }

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
    unsafe {
        USB_BUS = Some(usb_bus);
    }

    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };

    let usb_hid = HIDClass::new(bus_ref, KeyboardReport::desc(), 60);
    unsafe {
        USB_HID = Some(usb_hid);
    }

    // create a usb device
    let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x0000, 0x0000))
        .manufacturer("brxken128")
        .product("PassPico")
        .serial_number("0x0001")
        .device_class(3)
        .build();
    unsafe {
        USB_DEVICE = Some(usb_dev);
    }

    unsafe {
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    };
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

/// This submits a new KeyboardReport to the USB stack
fn keyboard_push(report: KeyboardReport) -> Result<usize, usb_device::UsbError> {
    critical_section::with(|_| unsafe {
        USB_HID.as_mut().map(|hid| hid.push_input(&report))
    })
    .unwrap()
}

const BLANK_KEYCODES: [u8; 6] = [KEY_NONE, KEY_NONE, KEY_NONE, KEY_NONE, KEY_NONE, KEY_NONE];

const BLANK_REPORT: KeyboardReport = KeyboardReport {
    modifier: 0x00,
    reserved: 0x00,
    leds: 0x00,
    keycodes: BLANK_KEYCODES,
};

pub fn get_modifier(c: &char) -> u8 {
    if c.is_uppercase() {
        return KEY_MOD_LSHIFT;
    }

    // needs double checking
    match c {
        '!' => KEY_MOD_LSHIFT,
        '@' => KEY_MOD_LSHIFT,
        '#' => KEY_MOD_LSHIFT,
        '$' => KEY_MOD_LSHIFT,
        '%' => KEY_MOD_LSHIFT,
        '^' => KEY_MOD_LSHIFT,
        '&' => KEY_MOD_LSHIFT,
        '*' => KEY_MOD_LSHIFT,
        '(' => KEY_MOD_LSHIFT,
        ')' => KEY_MOD_LSHIFT,
        '_' => KEY_MOD_LSHIFT,
        '+' => KEY_MOD_LSHIFT,
        '{' => KEY_MOD_LSHIFT,
        '}' => KEY_MOD_LSHIFT,
        '|' => KEY_MOD_LSHIFT,
        '~' => KEY_MOD_LSHIFT,
        ':' => KEY_MOD_LSHIFT,
        '"' => KEY_MOD_LSHIFT,
        '<' => KEY_MOD_LSHIFT,
        '>' => KEY_MOD_LSHIFT,
        '?' => KEY_MOD_LSHIFT,
        _ => 0x00,
    }
}

// keep track of the last char to reduce amount of blank reports sent
// this lets us type strings faster
// not too sure how much of a perf impact Option has here (it's probably negligible)
fn keyboard_write(text: &str, delay: &mut Delay) -> Result<(), usb_device::UsbError> {
    let mut last_char: Option<char> = None;
    for c in text.chars() {
        let modifier = get_modifier(&c);

        // if current char differs from the previous, we can send a new report instantly
        // if they match, we need a blank report to say that it's another input of the char
        if let Some(lc) = last_char {
            if c == lc {
                keyboard_push(BLANK_REPORT)?;

                delay.delay_ms(35);
            }
        }

        let codes = [char_to_keycode(&c), KEY_NONE, KEY_NONE, KEY_NONE, KEY_NONE, KEY_NONE];

        let report = KeyboardReport {
            modifier: modifier,
            reserved: 0x00,
            leds: 0x00,
            keycodes: codes,
        };

        keyboard_push(report)?;

        last_char = Some(c);

        delay.delay_ms(35);
    }

    keyboard_push(BLANK_REPORT)?;

    Ok(())
}

fn keyboard_println(text: &str, delay: &mut Delay) -> Result<(), usb_device::UsbError> {
    keyboard_write(text, delay)?;

    // we need this delay as `keyboard_write()` does not include an ending one
    delay.delay_ms(35);

    let codes = [codes::KEY_ENTER, KEY_NONE, KEY_NONE, KEY_NONE, KEY_NONE, KEY_NONE];

    let report = KeyboardReport {
        modifier: 0x00,
        reserved: 0x00,
        leds: 0x00,
        keycodes: codes,
    };

    keyboard_push(report)?;

    delay.delay_ms(35);

    keyboard_push(BLANK_REPORT)?;

    Ok(())
}

// this converts a char to a keycode
fn char_to_keycode(c: &char) -> u8 {
    match c {
        'a' | 'A' => KEY_A,
        'b' | 'B' => KEY_B,
        'c' | 'C' => KEY_C,
        'd' | 'D' => KEY_D,
        'e' | 'E' => KEY_E,
        'f' | 'F' => KEY_F,
        'g' | 'G' => KEY_G,
        'h' | 'H' => KEY_H,
        'i' | 'I' => KEY_I,
        'j' | 'J' => KEY_J,
        'k' | 'K' => KEY_K,
        'l' | 'L' => KEY_L,
        'm' | 'M' => KEY_M,
        'n' | 'N' => KEY_N,
        'o' | 'O' => KEY_O,
        'p' | 'P' => KEY_P,
        'q' | 'Q' => KEY_Q,
        'r' | 'R' => KEY_R,
        's' | 'S' => KEY_S,
        't' | 'T' => KEY_T,
        'u' | 'U' => KEY_U,
        'v' | 'V' => KEY_V,
        'w' | 'W' => KEY_W,
        'x' | 'X' => KEY_X,
        'y' | 'Y' => KEY_Y,
        'z' | 'Z' => KEY_Z,
        '1' | '!' => KEY_1,
        '2' | '@' => KEY_2,
        '3' | '#' => KEY_3,
        '4' | '$' => KEY_4,
        '5' | '%' => KEY_5,
        '6' | '^' => KEY_6,
        '7' | '&' => KEY_7,
        '8' | '*' => KEY_8,
        '9' | '(' => KEY_9,
        '0' | ')' => KEY_0,
        '-' | '_' => KEY_MINUS,
        '=' | '+' => KEY_EQUAL,
        '[' | '{' => KEY_LEFTBRACE,
        ']' | '}' => KEY_RIGHTBRACE,
        '\\' | '|' => KEY_BACKSLASH,
        ';' | ':' => KEY_SEMICOLON,
        '\'' | '"' => KEY_APOSTROPHE,
        '`' | '~' => KEY_GRAVE,
        ',' | '<' => KEY_COMMA,
        '.' | '>' => KEY_DOT,
        '/' | '?' => KEY_SLASH,
        ' ' => KEY_SPACE,
        _ => KEY_E,
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
