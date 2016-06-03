#![feature(lang_items)]
#![feature(no_core)]
#![feature(core_intrinsics)]
#![feature(asm)]

#![no_core]
#![no_main]

extern crate core as avr_core;
extern crate arduino;

use avr_core::prelude::v1::*;
use avr_core::intrinsics::{volatile_load, volatile_store};

pub mod prelude;
pub mod timer0;
pub mod timer1;

use arduino::*;
use prelude::*;

#[lang = "eh_personality"]
extern fn eh_personality() {}

#[lang = "panic_fmt"]
extern fn panic_fmt() -> ! { loop {} }

#[no_mangle]
pub unsafe extern "avr-interrupt" fn _ivr_timer0_compare_a() {
    let prev_value = volatile_load(PORTB);
    volatile_store(PORTB, prev_value ^ PINB5);
}

const CPU_FREQUENCY_HZ: u64 = 16_000_000;
const DESIRED_HZ_TIM1: f64 = 1.0;
const TIM1_PRESCALER: u64 = 1024;
const INTERRUPT_EVERY_1_HZ_1024_PRESCALER: u16 = ((CPU_FREQUENCY_HZ as f64 / (DESIRED_HZ_TIM1 * TIM1_PRESCALER as f64)) as u64 - 1) as u16;

const DESIRED_HZ_TIM0: f64 = 30.0;
const TIM0_PRESCALER: u64 = 1024;
const INTERRUPT_EVERY_30_HZ_1024_PRESCALER: u8 = ((CPU_FREQUENCY_HZ as f64 / (DESIRED_HZ_TIM0 * TIM0_PRESCALER as f64)) as u64 - 1) as u8;

#[no_mangle]
pub extern fn main() {
    unsafe {
        without_interrupts(|| {
            // Configure all Port B pins as outputs
            volatile_store(DDRB, 0xFF);
            // Turn on all Port B pins
            volatile_store(PORTB, 0xFF);

            timer0::Timer::new()
                .waveform_generation_mode(timer0::WaveformGenerationMode::ClearOnTimerMatchOutputCompare)
                .clock_source(timer0::ClockSource::Prescale1024)
                .output_compare_1(Some(INTERRUPT_EVERY_30_HZ_1024_PRESCALER))
                .configure();

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
