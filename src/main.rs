#![feature(lang_items)]
#![feature(no_core)]
#![feature(asm)]
#![feature(naked_functions)]
#![feature(abi_avr_interrupt)]
#![feature(panic_handler)]

#![no_core]
#![no_main]

extern crate core;
extern crate ruduino;

use core::prelude::v1::*;
use core::ptr::{read_volatile, write_volatile};
// Let's pretend to have a standard library
pub use core::{option, iter, fmt, ops};
use core::fmt::Write;

use ruduino::*;
use ruduino::prelude::*;

#[lang = "eh_personality"]
extern fn eh_personality() {}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}

#[no_mangle]
pub unsafe extern "avr-interrupt" fn _ivr_timer1_compare_a() {
    let prev_value = read_volatile(PORTB);
    write_volatile(PORTB, prev_value ^ PINB5);
}

const CPU_FREQUENCY_HZ: u64 = 16_000_000;
const CPU_RAM_BYTES: u16 = 2048;

const DESIRED_HZ_TIM1: f64 = 2.0;
const TIM1_PRESCALER: u64 = 1024;
const INTERRUPT_EVERY_1_HZ_1024_PRESCALER: u16 =
    ((CPU_FREQUENCY_HZ as f64 / (DESIRED_HZ_TIM1 * TIM1_PRESCALER as f64)) as u64 - 1) as u16;

// const DESIRED_HZ_TIM0: f64 = 30.0;
// const TIM0_PRESCALER: u64 = 1024;
// const INTERRUPT_EVERY_30_HZ_1024_PRESCALER: u8 = ((CPU_FREQUENCY_HZ as f64 / (DESIRED_HZ_TIM0 * TIM0_PRESCALER as f64)) as u64 - 1) as u8;

const BAUD: u64 = 9600;
const MYUBRR: u16 = (CPU_FREQUENCY_HZ / 16 / BAUD - 1) as u16;

extern {
    #[naked]
    fn __initialize_memory();
}

fn initialize_memory() {
    unsafe {
        __initialize_memory();
    }
}

#[no_mangle]
pub extern fn main() -> ! {
    without_interrupts(|| { // TODO: we know interrupts are off, don't we?
        // The ABI requires that r1 starts as zero
        unsafe { asm!("eor r1, r1"); }

        unsafe {
            write_volatile(SP, CPU_RAM_BYTES);
        }

        initialize_memory();

        unsafe {
            // Configure all Port B pins as outputs
            write_volatile(DDRB, 0xFF);
            // Turn on all Port B pins
            write_volatile(PORTB, 0xFF);
        }

        timer1::Timer::new()
            .waveform_generation_mode(timer1::WaveformGenerationMode::ClearOnTimerMatchOutputCompare)
            .clock_source(timer1::ClockSource::Prescale1024)
            .output_compare_1(Some(INTERRUPT_EVERY_1_HZ_1024_PRESCALER))
            .configure();

        serial::Serial::new(MYUBRR)
            .character_size(serial::CharacterSize::EightBits)
            .mode(serial::Mode::Asynchronous)
            .parity(serial::Parity::Disabled)
            .stop_bits(serial::StopBits::OneBit)
            .configure();
    });

    serial::transmit(b'A');
    write_newline();

    for i in 0..4 {
        let b = unsafe { *TO_HEX.get_unchecked(i) };
        serial::transmit(b);
    }
    write_newline();

    write_u8_hex(0x00);
    write_newline();
    write_u8_hex(0x05);
    write_newline();
    write_u8_hex(0x50);
    write_newline();
    write_u8_hex(0xFF);
    write_newline();

    write_slice_hex(&[0x00, 0x05, 0x50, 0xFF]);
    write_newline();

    write_raw("write_raw");

    SuperSerial.write_str("write_str\r\n").unwrap();

    // Formatting code disabled in libcore
    // writeln!(SuperSerial, "writeln!").unwrap();
    // write_newline();

    serial::transmit(b'Z');
    write_newline();

    loop {
        if let Some(b) = serial::try_receive() {
            serial::transmit(b'>');
            serial::transmit(b);
            serial::transmit(b'<');
        }
        // forever!
    }
}

pub static TO_HEX: &'static [u8; 16] = b"0123456789ABCDEF";

#[inline(never)]
pub fn write_slice_hex(v: &[u8]) {
    for &b in v {
        write_u8_hex(b);
    }
}

#[inline(never)]
pub fn write_u8_hex(v: u8) {
    let top_idx = (v >> 4) & 0b1111;
    let bot_idx = (v >> 0) & 0b1111;

    let top = unsafe { *TO_HEX.get_unchecked(top_idx as usize) };
    let bot = unsafe { *TO_HEX.get_unchecked(bot_idx as usize) };

    serial::transmit(top);
    serial::transmit(bot);
}

#[inline(never)]
fn write_newline() {
    serial::transmit(b'\r');
    serial::transmit(b'\n');
}

#[inline(never)]
fn write_raw(s: &str) {
    for b in s.bytes() {
        serial::transmit(b);
    }
    write_newline();
}

struct SuperSerial;

impl fmt::Write for SuperSerial {
    fn write_str(&mut self, s: &str) -> Result<(), fmt::Error> {
        for b in s.bytes() {
            serial::transmit(b);
        }
        Ok(())
    }
}
