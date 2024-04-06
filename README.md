# passta

`passta` (formery `passpico`) is a project that allows you to enter passwords (such as a LUKS encryption key or a password manager's master password) automatically, and it's written entirely in Rust!

The original board of choice was the Adafruit Trinkey QT2040 due to the extremely suitable form factor (it fits directly into a USB-A port) and has an RGB LED which can act as a status indicator. For a more secure experience, an ESP32-S3 based board should be chosen.

I also plan to support many different boards from many different brands (all that will likely need changing is some GPIO pins). We can do this by having an index of files that correspond to each board, and all you need to do is select the right one!

### Physical Security and longevity

Please ensure that the physical security of the device is considered, and backups should also be in place. Similarly, data is not retained _forever_ on "flash" cell-based storage of any kind, especially when left unpowered for extended periods of time - keep that in mind when selecting your board of choice, as well as any backups. You may maintan lesser-used boards by powering them up once in a while, and running the provided validation function to ensure that the data hasn't corrupted/malformed in any way.

### Downsides to a standalone RP2040

Most Pico boards come with a button, and the RP2040 has HID support, so they're extremely suitable OOTB. However, they sadly lack the cryptographic extensions and some security features which are provided by the ESP32-S3, but they are still a good choice as long as you can keep the device physically secure (either on your person, stashed away somewhere, or in a good quality safe). HSM support will be added (e.g. something like Adafruit's ATTEC608) in order to provide comparable levels of cryptographic security.

### Cryptographic Security and a Companion App

The ESP32-S3 will make use of as many security features as `esp-hal` allows, and I also plan to create a companion app which communicates with the device over BLE (via a fully end-to-end encrypted channel), in order to provide a "master" password, so that each item stored is encrypted on-device and protected with only a password that _you_ know.

This will be hashed and stored in the chip's memory, and will be irrecoverable once power to the board is lost (i.e. unplugged, or by using the "lock" feature in the companion app (which will `zeroize` the key to ensure that the memory is empty)).

The ESP32-S3 offers: native flash encryption, secure user-configurable key storage, secure boot, key attestation, OTA updates, cryptographic primitives, and even more. That's a recipe for a secure device when done correctly, and pretty much a _perfect_ fit for this appliation.

### Custom PCB

I've also been designing a dedicated board for this. It contains an ESP32-S3 (although an RP2040 variant will be just as good). It's going to contain a dedicated HSM (hardware security module) for secure key storage/management and handle all of the cryptographic functionality, as well as dedicated EEPROM which will be entirely encrypted.

I also plan to add some tamper-proofing/resistance where possible to the final boards (both on the PCB design itself, and maybe even physically), and all files will be fully open-sourced once I'm both: fully confident in the design, and have manufactured a couple to validate them for myself. Development of `passta` will be done using off-the-shelf boards for the time being.

I will also 3D print a case for this, and provide all files for that too!

## Disclaimer

I personally use a device that is similar to this. I took an off-the-shelf Adafruit ESP32-S3 TFT Feather board, and designed a PCB that added room for 3x buttons. I used the display to show the name of the entry (e.g. "1Password Secret Key"), and the list could be scrolled through via the left/right buttons, and the password could be entered via the centre ("enter") button. I have since lost the original source code, and it lacked a lot of security features which I hope to bring to an official project such as this, but it definitely sparked my motivation and desire to create a project that was both more secure and better thought-out.

The RP2040 alone may end up as a "use at your own risk" configuration, unless you use a HSM or are comfortable without solid encryption in place (e.g. it's not used for anything critical, your physical security for the device is pretty good, etc). The ESP32-S3 has many, many more security features (see the [cryptographic security](#cryptographic-security-and-a-companion-app) section for more information).
