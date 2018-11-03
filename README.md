# Barebones Rust and Cargo program for Arduino Uno

Features:

1. Uses timer interrupts to blink the LED.
1. Echos back characters sent via the built-in serial device.

The code is written completely in Rust and a small amount of assembly!

See also the [official blink example][blink], which makes use of some
GCC-provided code.

[blink]: https://github.com/avr-rust/blink

## High-level instructions

1. Install avr-gcc, avrdude, and picocom. For example, using Homebrew:

    ```
    brew tap osx-cross/avr
    brew install avr-gcc avrdude picocom
    ```

    avr-gcc is used as the linker, avrdude uploads the finished code,
    and picocom is used as the serial terminal.

1. Compile the [fork of Rust with AVR support][avr-rust] and switch to it.

1. Configure Xargo:

    ```
    export RUST_TARGET_PATH=$(pwd)
    export XARGO_RUST_SRC=/path/to/avr-rust/src
    ```

1. Build the code: `make`

1. Upload the code: `make program`

1. Connect the serial terminal: `make connect-terminal`

[avr-rust]: https://github.com/avr-rust/rust
