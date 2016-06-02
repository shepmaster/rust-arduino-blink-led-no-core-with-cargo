#![feature(lang_items)]
#![feature(no_core)]
#![feature(core_intrinsics)]
#![feature(asm)]

#![no_core]
#![no_main]

extern crate core as avr_core;

use avr_core::prelude::v1::*;
use avr_core::intrinsics::{volatile_load, volatile_store};

pub mod prelude;
pub mod avr;
pub mod timer1;

use avr::*;
use prelude::*;

#[lang = "eh_personality"]
extern fn eh_personality() {}

#[lang = "panic_fmt"]
extern fn panic_fmt() -> ! { loop {} }

#[no_mangle]
pub unsafe extern "avr-interrupt" fn _ivr_timer1_compare_a() {
    let prev_value = volatile_load(PORTB);
    volatile_store(PORTB, prev_value ^ PINB5);
}

const CPU_FREQUENCY_HZ: u64 = 16_000_000;
const DESIRED_HZ: f64 = 10.0/3.0;
const PRESCALER: u64 = 1024;
const INTERRUPT_EVERY_1_HZ_1024_PRESCALER: u16 = ((CPU_FREQUENCY_HZ as f64 / (DESIRED_HZ * PRESCALER as f64)) as u64 - 1) as u16;

#[no_mangle]
pub extern fn main() {
    unsafe {
        without_interrupts(|| {
            // Configure all Port B pins as outputs
            volatile_store(DDRB, 0xFF);
            // Turn on all Port B pins
            volatile_store(PORTB, 0xFF);

            timer1::Timer::new()
                .waveform_generation_mode(timer1::WaveformGenerationMode::ClearOnTimerMatchOutputCompare)
                .clock_source(timer1::ClockSource::Prescale1024)
                .output_compare_1(Some(INTERRUPT_EVERY_1_HZ_1024_PRESCALER))
                .configure();
        });

        loop {
            // forever!
        }
    }
}
