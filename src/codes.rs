/**
 * USB HID Keyboard scan codes as per USB spec 1.11
 * plus some additional codes
 * 
 * Created by MightyPork, 2016
 * Modified for Rust by brxken128, 2022
 * Public domain
 * 
 * Adapted from:
 * https://source.android.com/devices/input/keyboard-devices.html
 */

/**
 * Modifier masks - used for the first byte in the HID report.
 * NOTE: The second byte in the report is reserved, 0x00
 */
pub const KEY_MOD_LCTRL: u8 =  0x01;
pub const KEY_MOD_LSHIFT: u8 = 0x02;
pub const KEY_MOD_LALT: u8 =   0x04;
pub const KEY_MOD_LMETA: u8 =  0x08;
pub const KEY_MOD_RCTRL: u8 =  0x10;
pub const KEY_MOD_RSHIFT: u8 = 0x20;
pub const KEY_MOD_RALT: u8 =   0x40;
pub const KEY_MOD_RMETA: u8 =  0x80;

/**
 * Scan codes - last N slots in the HID report (usually 6).
 * 0x00 if no key pressed.
 * 
 * If more than N keys are pressed, the HID reports 
 * KEY_ERR_OVF in all slots to indicate this condition.
 */

pub const KEY_NONE: u8 = 0x00; // No key pressed
pub const KEY_ERR_OVF: u8 = 0x01; //  Keyboard Error Roll Over - used for all slots if too many keys are pressed ("Phantom key")
pub const KEY_A: u8 = 0x04; // Keyboard a and A
pub const KEY_B: u8 = 0x05; // Keyboard b and B
pub const KEY_C: u8 = 0x06; // Keyboard c and C
pub const KEY_D: u8 = 0x07; // Keyboard d and D
pub const KEY_E: u8 = 0x08; // Keyboard e and E
pub const KEY_F: u8 = 0x09; // Keyboard f and F
pub const KEY_G: u8 = 0x0a; // Keyboard g and G
pub const KEY_H: u8 = 0x0b; // Keyboard h and H
pub const KEY_I: u8 = 0x0c; // Keyboard i and I
pub const KEY_J: u8 = 0x0d; // Keyboard j and J
pub const KEY_K: u8 = 0x0e; // Keyboard k and K
pub const KEY_L: u8 = 0x0f; // Keyboard l and L
pub const KEY_M: u8 = 0x10; // Keyboard m and M
pub const KEY_N: u8 = 0x11; // Keyboard n and N
pub const KEY_O: u8 = 0x12; // Keyboard o and O
pub const KEY_P: u8 = 0x13; // Keyboard p and P
pub const KEY_Q: u8 = 0x14; // Keyboard q and Q
pub const KEY_R: u8 = 0x15; // Keyboard r and R
pub const KEY_S: u8 = 0x16; // Keyboard s and S
pub const KEY_T: u8 = 0x17; // Keyboard t and T
pub const KEY_U: u8 = 0x18; // Keyboard u and U
pub const KEY_V: u8 = 0x19; // Keyboard v and V
pub const KEY_W: u8 = 0x1a; // Keyboard w and W
pub const KEY_X: u8 = 0x1b; // Keyboard x and X
pub const KEY_Y: u8 = 0x1c; // Keyboard y and Y
pub const KEY_Z: u8 = 0x1d; // Keyboard z and Z

pub const KEY_1: u8 = 0x1e; // Keyboard 1 and !
pub const KEY_2: u8 = 0x1f; // Keyboard 2 and @
pub const KEY_3: u8 = 0x20; // Keyboard 3 and #
pub const KEY_4: u8 = 0x21; // Keyboard 4 and $
pub const KEY_5: u8 = 0x22; // Keyboard 5 and %
pub const KEY_6: u8 = 0x23; // Keyboard 6 and ^
pub const KEY_7: u8 = 0x24; // Keyboard 7 and &
pub const KEY_8: u8 = 0x25; // Keyboard 8 and *
pub const KEY_9: u8 = 0x26; // Keyboard 9 and (
pub const KEY_0: u8 = 0x27; // Keyboard 0 and )

pub const KEY_ENTER: u8 = 0x28; // Keyboard Return (ENTER)
pub const KEY_ESC: u8 = 0x29; // Keyboard ESCAPE
pub const KEY_BACKSPACE: u8 = 0x2a; // Keyboard DELETE (Backspace)
pub const KEY_TAB: u8 = 0x2b; // Keyboard Tab
pub const KEY_SPACE: u8 = 0x2c; // Keyboard Spacebar
pub const KEY_MINUS: u8 = 0x2d; // Keyboard - and _
pub const KEY_EQUAL: u8 = 0x2e; // Keyboard = and +
pub const KEY_LEFTBRACE: u8 = 0x2f; // Keyboard [ and {
pub const KEY_RIGHTBRACE: u8 = 0x30; // Keyboard ] and }
pub const KEY_BACKSLASH: u8 = 0x31; // Keyboard \ and |
pub const KEY_HASHTILDE: u8 = 0x32; // Keyboard Non-US # and ~
pub const KEY_SEMICOLON: u8 = 0x33; // Keyboard ; and :
pub const KEY_APOSTROPHE: u8 = 0x34; // Keyboard ' and "
pub const KEY_GRAVE: u8 = 0x35; // Keyboard ` and ~
pub const KEY_COMMA: u8 = 0x36; // Keyboard , and <
pub const KEY_DOT: u8 = 0x37; // Keyboard . and >
pub const KEY_SLASH: u8 = 0x38; // Keyboard / and ?
pub const KEY_CAPSLOCK: u8 = 0x39; // Keyboard Caps Lock
