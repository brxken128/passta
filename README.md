# passpico

`passpico` is a project that allows you to enter text with a Pi Pico, entirely in Rust. The board of choice is the Adafruit Trinkey QT2040 - due to an extremely suitable form factor, USB-A connector and the RGB LED for a personalized touch.

The main use-case was originally for passwords, but it will allow you to enter any text via HID emulation.

Most Pico boards come with a button, and the RP2040 has HID support, so they're extremely suitable OOTB. Master passwords, encryption keys, etc. can all be extremely challenging to type, especially if they're symbol-heavy (and sometimes USB security tokens just can't work for a certain task).

I may branch the `KeyboardReport` and entering off into a separate crate at some point.

I have strong plans to add `OneButton`-esque functionality to this project, so that users can enter up to 4 (or more!) passwords with one Pico and no extra hardware.