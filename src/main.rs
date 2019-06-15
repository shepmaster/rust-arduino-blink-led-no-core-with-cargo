#![feature(lang_items)]
#![feature(asm)]
#![feature(naked_functions)]
#![feature(abi_avr_interrupt)]
#![feature(never_type)]
#![feature(async_await)]
#![feature(generators)]

#![no_std]
#![no_main]

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
fn panic(info: &core::panic::PanicInfo) -> ! {
    loop {
        crate::write_str("PANIC ");
        if let Some(l) = info.location() {
            crate::write_slice_hex(&l.line().to_le_bytes());
            crate::write_str(" / ");
            crate::write_slice_hex(&l.column().to_le_bytes());
//            crate::write_strln(l.file());
        }
        crate::write_strln("");
    }
}

#[no_mangle]
pub unsafe extern "avr-interrupt" fn _ivr_timer1_compare_a() {
    let prev_value = read_volatile(PORTB);
    write_volatile(PORTB, prev_value ^ PINB5);
}

// #[no_mangle]
// pub unsafe extern "avr-interrupt" fn _ivr_usart_rx_complete() {
//     fut::rx_interrupt_handler();
// }

#[no_mangle]
pub unsafe extern "avr-interrupt" fn _ivr_usart_udr_empty() {
    fut::tx_empty_interrupt_handler();
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

    // serial::transmit(b'A');
    // write_newline();

    // for i in 0..4 {
    //     let b = unsafe { *TO_HEX.get_unchecked(i) };
    //     serial::transmit(b);
    // }
    // write_newline();

    // write_u8_hex(0x00);
    // write_newline();
    // write_u8_hex(0x05);
    // write_newline();
    // write_u8_hex(0x50);
    // write_newline();
    // write_u8_hex(0xFF);
    // write_newline();

    write_slice_hex(&[0x00, 0x05, 0x50, 0xFF]);
    // write_newline();

    // write_raw("write_raw");

//    crate::write_strln("Booted");

    // // Formatting code disabled in libcore
    // // writeln!(SuperSerial, "writeln!").unwrap();
    // // write_newline();

    // serial::transmit(b'Z');
    // write_newline();

    use ruduino::Bit::*;

    unsafe {
                // Configure all Port D pins as outputs
            write_volatile(DDRD, 0xFF);
            // Turn off all Port D pins
            write_volatile(PORTD, 0x00);
    }
//    PORT_D.configuration().set_all_as_output().configure();

    PORT_D.data().set(0);

    fut::do_futures();

    //loop {
    //     fn alpha(i: i32) -> i32 { i + 1 }
    //     static FOO: fn(i32) -> i32 = alpha;
//}

/*
    loop {
        if let Some(b) = serial::try_receive() {
            serial::transmit(b'>');
            serial::transmit(b);
            serial::transmit(b'<');
        }
        // forever!
    }
*/
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

// #[inline(never)]
// fn write_newline() {
//     serial::transmit(b'\r');
//     serial::transmit(b'\n');
// }

// #[inline(never)]
// fn write_raw(s: &str) {
//     for b in s.bytes() {
//         serial::transmit(b);
//     }
//     write_newline();
// }


fn write_str(s: &str) {
    SuperSerial.write_str(s).unwrap();
}

fn write_strln(s: &str) {
    SuperSerial.write_str(s).unwrap();
    SuperSerial.write_str("\r\n").unwrap();
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

mod fut {
    use core::{pin::Pin, future::Future, task::{Context, Poll, Waker}};
    use ruduino::{UCSR0B, RXCIE0, UDRIE0, serial};
    use embrio_async::embrio_async;
//    use embrio_executor::executor::block_on;

    fn set_bit_in(register: *mut u8, bit: u8) {
        use core::ptr::{read_volatile, write_volatile};
        unsafe {
            let old = read_volatile(register);
            let new = old | bit;
            write_volatile(register, new);
        }
    }

    fn unset_bit_in(register: *mut u8, bit: u8) {
        use core::ptr::{read_volatile, write_volatile};
        unsafe {
            let old = read_volatile(register);
            let new = old ^ bit;
            write_volatile(register, new);
        }
    }


    struct Serial;

    impl Serial {
        // TODO: Maybe take a slice instead?
        fn tx(&self, byte: u8) -> SerialTx {
            SerialTx(byte)
        }

        // TODO: Maybe take a slice instead?
        // fn rx<'a>(&self, byte: &'a mut u8) -> SerialRx<'a> {
        //     SerialRx(byte)
        // }
    }

    // struct SerialRx<'a>(&'a mut u8);

    // static mut RX_WAKER: Option<Waker> = None;

    // #[inline(always)]
    // pub fn rx_interrupt_handler() {
    //     // Safety:
    //     // We are on a single-threaded CPU, so static mutable shoudn't matter.
    //     unsafe {
    //         if let Some(waker) = RX_WAKER.take() {
    //             // Notify our waker to poll the future again
    //             waker.wake();

    //             // We must either read from the buffer or disable the
    //             // interrupt to prevent re-invoking the interrupt
    //             // handler immediately.
    //             disable_serial_rx_interrupt();
    //         }
    //     }
    // }

    // fn enable_serial_rx_interrupt() {
    //     set_bit_in(UCSR0B, RXCIE0);
    // }

    // fn disable_serial_rx_interrupt() {
    //     unset_bit_in(UCSR0B, RXCIE0);
    // }

    // impl<'a> Future for SerialRx<'a> {
    //     type Output = ();

    //     fn poll(self: Pin<&mut Self>, ctx: &mut Context) -> Poll<Self::Output> {
    //         match serial::try_receive() {
    //             Some(v) => {
    //                 *Pin::get_mut(self).0 = v;
    //                 Poll::Ready(())
    //             },
    //             None => {
    //                 // Safety:
    //                 // We are on a single-threaded CPU, so static mutable shoudn't matter.
    //                 unsafe {
    //                     RX_WAKER = Some(ctx.waker().clone());
    //                 }
    //                 enable_serial_rx_interrupt();

    //                 Poll::Pending
    //             }
    //         }
    //     }
    // }

    struct SerialTx(u8);

    static mut TX_WAKER: Option<Waker> = None;

    #[inline(always)]
    pub fn tx_empty_interrupt_handler() {
        // Safety:
        // We are on a single-threaded CPU, so static mutable shoudn't matter.
        unsafe {
            if let Some(waker) = TX_WAKER.take() {
                // Notify our waker to poll the future again
                waker.wake();

                // We must either read from the buffer or disable the
                // interrupt to prevent re-invoking the interrupt
                // handler immediately.
                disable_serial_tx_empty_interrupt();
            }
        }
    }

    fn enable_serial_tx_empty_interrupt() {
        set_bit_in(UCSR0B, UDRIE0);
    }

    fn disable_serial_tx_empty_interrupt() {
        unset_bit_in(UCSR0B, UDRIE0);
    }

    impl Future for SerialTx {
        type Output = ();

        fn poll(self: Pin<&mut Self>, ctx: &mut Context) -> Poll<Self::Output> {
//            use ruduino::{Bit::*, io::PORT_D};
  //          PORT_D.data().toggle_bit(Bit3);

            match serial::try_transmit(self.0) {
                Ok(()) => {
                    Poll::Ready(())
                },
                Err(()) => {
                    // Safety:
                    // We are on a single-threaded CPU, so static mutable shoudn't matter.
                    unsafe {
                        TX_WAKER = Some(ctx.waker().clone());
                    }
                    enable_serial_tx_empty_interrupt();

                    Poll::Pending
                }
            }
        }
    }

    #[embrio_async]
    async fn example() {
        Serial.tx(b'X').await;

        // let mut buf = 0;
        // Serial.rx(&mut buf).await;
        // Serial.tx(b'>').await;
        // Serial.tx(buf).await;
        // Serial.tx(b'<').await;
    }

    pub fn do_futures() -> ! {
        use embrio_executor::Executor;

        //crate::write_strln("do_futures starting");

        static mut EXECUTOR: Executor = Executor::new();
        loop {
            let executor = unsafe { &mut EXECUTOR };
            executor.block_on(example());
        }
    }
}

#[no_mangle]
extern "C" fn abort() -> ! {
    loop {
        crate::write_strln("ABORT");
    }
}

#[no_mangle]
extern "C" fn __sync_lock_test_and_set_1(ptr: *mut u8, desired: u8) -> u8 {
    without_interrupts(|| {
        unsafe {
            let old = *ptr;
            *ptr = desired;
            old
        }
    })
}
