#![feature(lang_items)]
#![feature(no_core)]
#![feature(asm)]
#![feature(naked_functions)]
#![feature(abi_avr_interrupt)]

#![no_core]
#![no_main]

#[macro_use]
extern crate core;
extern crate arduino;
extern crate rustc_builtins;

use core::prelude::v1::*;
use core::ptr::{read_volatile, write_volatile};
// Let's pretend to have a standard library
pub use core::{option, iter, fmt, ops};
use core::fmt::Write;

use arduino::*;
use arduino::prelude::*;

#[lang = "eh_personality"]
extern fn eh_personality() {}

#[lang = "panic_fmt"]
extern fn panic_fmt() -> ! { loop {} }

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

// const MEMORY_MAP_START: u16 = MEMORY_MAP_REGISTERS_START;
// const MEMORY_MAP_REGISTERS_START: u16 = 0x0000; // - 0x001F
// const MEMORY_MAP_IO_REGISTERS_START: u16 = 0x0020; // - 0x005F
// const MEMORY_MAP_EXTENDED_IO_REGISTERS_START: u16 = 0x0060; // - 0x00FF
// const MEMORY_MAP_SRAM_START: u16 = 0x0100;
// const MEMORY_MAP_SRAM_END: u16 = 0x0900; // EXCLUSIVE! DEPENDS ON PART!
    // 0x02FF/0x04FF/0x4FF/0x08FF



// extern {
//     static __data_start: u16;
//     static __data_end: u16;
//     static __data_load_start: u16;
//     static __data_load_end: u16;
// }

extern {
    #[naked]
    fn __initialize_memory();
}

fn initialize_memory() {
    unsafe {
        __initialize_memory();
    }

    // let len = __data_load_end - __data_load_start;
    // if len == 0 { return }

    // fn hi_lo(v: u16) -> (u8, u8) {
    //     (((v >> 8) & 0xFF) as u8, ((v >> 0) & 0xFF) as u8)
    // }

    // let (len_hi, len_lo) = hi_lo(len);
    // let (z_hi, z_lo) = hi_lo(__data_load_start);
    // let (x_hi, x_lo) = hi_lo(__data_start);

//     unsafe {
//         asm!("
//             sub r28, r30
//             sbc r29, r31    ; Y now contains the length of bytes
// entrypoint:
//             lpm r0, Z+      ; Load from program memory, increment pointer
//             st X+, r0       ; Store to RAM, increment pointer

//             subi r28, 1     ; Decrement the count
//             sbci r29, 0
// check:
//             brne entrypoint ; Exit when all bytes copied"
//              : // output operands
//              : // input operands
//              "{r31r30}"(__data_load_start) // Z
//              "{r27r26}"(__data_start)      // X
//              "{r29r28}"(__data_load_end)   // Y
//              : // clobbers
//              "cc r0"
//              : // options
//         );

        // let src = __data_start as *mut u8;
        // let dest = __data_load_start as *mut u8;
        // let len = __data_load_end - __data_load_start;

        // let src = 00800100 as *mut u8;
        // let dest = 00000126 as *mut u8;
        // let len = 00000136 - 00000124;

        // let dest = __data_start as *mut u8;
        // let src = __data_load_start as *mut u8;
        // let end = __data_load_end as *mut u8;

        // 0x0118
        //     0x0128


}

#[no_mangle]
pub extern fn main() -> ! {
    without_interrupts(|| { // TODO: we know interrupts are off, don't we?
        // r1 is assumed to be always zero
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
    writeln!(SuperSerial, "writeln!").unwrap();
    write_newline();

    serial::transmit(b'Z');
    write_newline();

    loop {
        if let Some(b) = serial::try_receive() {
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
