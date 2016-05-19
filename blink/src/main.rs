#![feature(no_core)]
#![feature(asm)]

#![no_core]
#![no_main]

extern crate avr_core;

use avr_core::prelude::*;
use avr_core::intrinsics::{volatile_load, volatile_store};
use avr_core::marker::PhantomData;

pub mod avr {
    pub const DDRB:   *mut u8 = 0x24 as *mut u8;
    pub const PORTB:  *mut u8 = 0x25 as *mut u8;

    pub const TCCR0A: *mut u8 = 0x44 as *mut u8;
    pub const TCCR0B: *mut u8 = 0x45 as *mut u8;
    pub const TCNT0:  *mut u8 = 0x46 as *mut u8;
    pub const OCR0A:  *mut u8 = 0x47 as *mut u8;
    pub const TIMSK0: *mut u8 = 0x6E as *mut u8;

    pub const TCCR1A: *mut u8 = 0x80 as *mut u8;
    pub const TCCR1B: *mut u8 = 0x81 as *mut u8;
    pub const TCCR1L: *mut u8 = 0x84 as *mut u8;
    pub const TCCR1H: *mut u8 = 0x85 as *mut u8;
    pub const OCR1AL: *mut u8 = 0x88 as *mut u8;
    pub const OCR1AH: *mut u8 = 0x89 as *mut u8;

    pub const TCNT1L: *mut u8 = 0x84 as *mut u8;
    pub const TCNT1H: *mut u8 = 0x85 as *mut u8;

    pub const TIMSK1: *mut u8 = 0x6F as *mut u8;

    // Should pins be represented in this way?
    pub const PINB5:  u8 = 0b0010_0000;

    pub const WGM12:  u8 = 0b0000_1000;
    pub const CS12:   u8 = 0b0000_0100;
    pub const CS10:   u8 = 0b0000_0001;
    pub const OCIE1A: u8 = 0b0000_0010;
}

use avr::*;

#[no_mangle]
pub unsafe extern "avr-interrupt" fn _ivr_timer1_compare_a() {
    let prev_value = volatile_load(PORTB);
    volatile_store(PORTB, prev_value ^ PINB5);
}

struct DisableInterrupts(PhantomData<()>);

impl DisableInterrupts {
    fn new() -> DisableInterrupts {
        unsafe { asm!("CLI") }
        DisableInterrupts(PhantomData)
    }
}

impl Drop for DisableInterrupts {
    fn drop(&mut self) {
        unsafe { asm!("SEI") }
    }
}

fn without_interrupts<F, T>(f: F) -> T
    where F: FnOnce() -> T
{
    let _disabled = DisableInterrupts::new();
    f()
}

#[no_mangle]
pub extern fn main() {
    unsafe {
        without_interrupts(|| {
            // Configure all Port B pins as outputs
            volatile_store(DDRB, 0xFF);
            // Turn on all Port B pins
            volatile_store(PORTB, 0xFF);

            // Initialize timer settings
            volatile_store(TCCR1A, 0);
            volatile_store(TCCR1B, 0);

            // Initialize timer counter
            volatile_store(TCNT1L, 0);
            volatile_store(TCNT1H, 0);

            // Set compare match register for 1hz increments (must be <65536)
            // (16*10^6) / (1*1024) - 1 => 15624 => 0x3D08
            volatile_store(OCR1AH, 0x3D);
            volatile_store(OCR1AL, 0x08);

            // Configure timer
            let ctc_mode = WGM12;
            let prescaler_1024x = CS12 | CS10;
            volatile_store(TCCR1B, ctc_mode | prescaler_1024x);

            // Enable timer compare interrupt
            volatile_store(TIMSK1, OCIE1A);
        });

        loop {
            // forever!
        }
    }
}
