# Rust and Cargo program for Arduino Uno

Features:

1. Uses timer interrupts to blink the LED.
1. Echos back characters sent via the built-in serial device.
1. Echos back characters sent via the built-in serial device using futures.

The code is written completely in Rust and a small amount of assembly!
We avoid using any of the GCC startup files.

See the [official blink example][blink], which makes use of some
GCC-provided code.

[blink]: https://github.com/avr-rust/blink

## High-level instructions

1. Follow the [official instructions][book] to install `avr-gcc` and
   `avrdude`. Additionally install picocom. For example, using
   Homebrew:

    ```
    brew install picocom
    ```

    `avr-gcc` is used as the linker, `avrdude` uploads the finished
    code, and picocom is used as the serial terminal.

1. Build the code: `make`

1. Upload the code: `make program`

1. Connect the serial terminal: `make connect-terminal`

[book]: https://book.avr-rust.com/
[avr-rust]: https://github.com/avr-rust/rust
