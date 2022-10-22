use cortex_m::delay::Delay;
use usbd_hid::descriptor::KeyboardReport;

use crate::{
    codes::{char_to_keycode, get_modifier, KEY_ENTER, KEY_MOD_NONE, KEY_NONE},
    USB_HID,
};

const BLANK_KEYCODES: [u8; 6] = [KEY_NONE, KEY_NONE, KEY_NONE, KEY_NONE, KEY_NONE, KEY_NONE];

const BLANK_REPORT: KeyboardReport = KeyboardReport {
    modifier: KEY_MOD_NONE,
    reserved: 0x00,
    leds: 0x00,
    keycodes: BLANK_KEYCODES,
};

/// This submits a new `KeyboardReport` to the USB stack
fn keyboard_push(report: KeyboardReport) -> Result<usize, usb_device::UsbError> {
    critical_section::with(|_| unsafe { USB_HID.as_mut().map(|hid| hid.push_input(&report)) })
        .unwrap()
}

// keep track of the last char to reduce amount of blank reports sent
// this lets us type strings faster
// not too sure how much of a perf impact Option has here (it's probably negligible)
pub fn keyboard_write(text: &str, delay: &mut Delay) -> Result<(), usb_device::UsbError> {
    let mut last_char: Option<char> = None;
    for c in text.chars() {
        let modifier = get_modifier(c);

        // if current char differs from the previous, we can send a new report instantly
        // if they match, we need a blank report to say that it's another input of the char
        if let Some(lc) = last_char {
            if c == lc {
                keyboard_push(BLANK_REPORT)?;

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

        keyboard_push(report)?;

        last_char = Some(c);

        delay.delay_ms(35);
    }

    keyboard_push(BLANK_REPORT)?;

    Ok(())
}

pub fn keyboard_println(text: &str, delay: &mut Delay) -> Result<(), usb_device::UsbError> {
    keyboard_write(text, delay)?;

    // we need this delay as `keyboard_write()` does not include an ending one
    delay.delay_ms(35);

    let codes = [KEY_ENTER, KEY_NONE, KEY_NONE, KEY_NONE, KEY_NONE, KEY_NONE];

    let report = KeyboardReport {
        modifier: KEY_MOD_NONE,
        reserved: 0x00,
        leds: 0x00,
        keycodes: codes,
    };

    keyboard_push(report)?;

    delay.delay_ms(35);

    keyboard_push(BLANK_REPORT)?;

    Ok(())
}
