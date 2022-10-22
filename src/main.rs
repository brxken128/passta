//! # Pico USB 'Twitchy' Mouse Example
//!
//! Creates a USB HID Class Pointing device (i.e. a virtual mouse) on a Pico
//! board, with the USB driver running in the main thread.
//!
//! It generates movement reports which will twitch the cursor up and down by a
//! few pixels, several times a second.
//!
//! See the `Cargo.toml` file for Copyright and license details.
//!
//! This is a port of
//! https://github.com/atsamd-rs/atsamd/blob/master/boards/itsybitsy_m0/examples/twitching_usb_mouse.rs

#![no_std]
#![no_main]

pub mod codes;

use core::iter::once;

// The macro for our start-up function
use adafruit_trinkey_qt2040::entry;

use adafruit_trinkey_qt2040::hal::Timer;
// The macro for marking our interrupt functions
use adafruit_trinkey_qt2040::hal::pac::interrupt;

use codes::KEY_MOD_LCTRL;
use cortex_m::delay::Delay;
use embedded_hal::digital::v2::InputPin;
// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use adafruit_trinkey_qt2040::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use adafruit_trinkey_qt2040::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use adafruit_trinkey_qt2040::hal;

use smart_leds::brightness;
use smart_leds::SmartLedsWrite;
use smart_leds::RGB8;
// USB Device support
use usb_device::{class_prelude::*, prelude::*};

use usbd_hid::descriptor::KeyboardReport;
// USB Human Interface Device (HID) Class support
use usbd_hid::descriptor::generator_prelude::*;
use usbd_hid::hid_class::HIDClass;
use ws2812_pio::Ws2812;

use crate::codes::*;

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

/// The USB Human Interface Device Driver (shared with the interrupt).
static mut USB_HID: Option<HIDClass<hal::usb::UsbBus>> = None;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_RAM_MEMCPY;

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then submits cursor movement
/// updates periodically.
#[entry]
fn main() -> ! {
    unsafe {
        // Release spinlocks since they are not freed on soft reset
        hal::sio::spinlock_reset();
    }

    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
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

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_BUS = Some(usb_bus);
    }

    // Grab a reference to the USB Bus allocator. We are promising to the
    // compiler not to take mutable access to this global variable whilst this
    // reference exists!
    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };

    // Set up the USB HID Class Device driver, providing Mouse Reports
    let usb_hid = HIDClass::new(bus_ref, KeyboardReport::desc(), 60);
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet.
        USB_HID = Some(usb_hid);
    }

    // Create a USB device with a fake VID and PID
    let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27da))
        .manufacturer("brxken's lab")
        .product("password enterer")
        .serial_number("001")
        .device_class(0)
        .build();
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_DEVICE = Some(usb_dev);
    }

    unsafe {
        // Enable the USB interrupt
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
            keyboard_writeln("text", &mut delay).unwrap();

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

/// Submit a new keyboard report to the USB stack.
///
/// We do this with interrupts disabled, to avoid a race hazard with the USB IRQ.
fn keyboard_push(report: KeyboardReport) -> Result<usize, usb_device::UsbError> {
    critical_section::with(|_| unsafe {
        // Now interrupts are disabled, grab the global variable and, if
        // available, send it a HID report
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

// keep track of the last char to reduce amount of blank reports sent
// this lets us type strings faster
// not too sure how much of a perf impact Option has here
fn keyboard_writeln(text: &str, delay: &mut Delay) -> Result<(), usb_device::UsbError> {
    let mut last_char: Option<char> = None;
    for c in text.chars() {
        let mut modifier = 0x00;
        if c.is_ascii_uppercase() {
            modifier = KEY_MOD_LSHIFT;
        }

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
    keyboard_writeln(text, delay)?;

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

// currently inexhaustive
fn char_to_keycode(c: &char) -> u8 {
    match c {
        'a' => KEY_A,
        'b' => KEY_B,
        'c' => KEY_C,
        'd' => KEY_D,
        'e' => KEY_E,
        'f' => KEY_F,
        'g' => KEY_G,
        'h' => KEY_H,
        'i' => KEY_I,
        'j' => KEY_J,
        'k' => KEY_K,
        'l' => KEY_L,
        'm' => KEY_M,
        'n' => KEY_N,
        'o' => KEY_O,
        'p' => KEY_P,
        'q' => KEY_Q,
        'r' => KEY_R,
        's' => KEY_S,
        't' => KEY_T,
        'u' => KEY_U,
        'v' => KEY_V,
        'w' => KEY_W,
        'x' => KEY_X,
        'y' => KEY_Y,
        'z' => KEY_Z,
        ' ' => KEY_SPACE,
        _ => unreachable!(),
    }
}

/// This function is called whenever the USB Hardware generates an Interrupt
/// Request.
#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    // Handle USB request
    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let usb_hid = USB_HID.as_mut().unwrap();
    usb_dev.poll(&mut [usb_hid]);
}

// End of file
