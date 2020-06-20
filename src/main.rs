#![feature(lang_items)]
#![feature(llvm_asm)]
#![feature(naked_functions)]
#![feature(abi_avr_interrupt)]
#![feature(never_type)]
#![feature(generators, generator_trait)]
#![no_std]
#![no_main]

use core::{prelude::v1::*, ptr};
use ruduino::{prelude::*, *};

#[allow(unused_macros)]
macro_rules! println {
    ($f: literal, $($arg:expr),*$(,)?) => {
        writeln!(crate::write::SERIAL, $f, $($arg,)*).unwrap();
    };
}

#[lang = "eh_personality"]
extern "C" fn eh_personality() {}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {
        write::strln("PANIC");

        // if let Some(s) = info.payload().downcast_ref::<&str>() {
        //     crate::write::str(" ");
        //     crate::write::str(s);
        // }

        // if let Some(l) = info.location() {
        //     crate::write::str(l.file());
        //     crate::write::str(" (");
        //     crate::write::slice_hex(&l.line().to_be_bytes());
        //     crate::write::str(", ");
        //     crate::write::slice_hex(&l.column().to_be_bytes());
        //     crate::write::str(")");
        // }
        // crate::strln("");
    }
}

#[no_mangle]
pub unsafe extern "avr-interrupt" fn _ivr_timer1_compare_a() {
    let prev_value = ptr::read_volatile(PORTB);
    ptr::write_volatile(PORTB, prev_value ^ PINB5);
}

// #[no_mangle]
// pub unsafe extern "avr-interrupt" fn _ivr_usart_rx_complete() {
//     fut::rx_interrupt_handler();
// }

// #[no_mangle]
// pub unsafe extern "avr-interrupt" fn _ivr_usart_udr_empty() {
//     fut::tx_empty_interrupt_handler();
// }

const CPU_FREQUENCY_HZ: u64 = 16_000_000;
const CPU_REGISTER_BYTES: u16 = 256;
const CPU_RAM_BYTES: u16 = 2048;
const CPU_INITIAL_STACK_POINTER: u16 = CPU_REGISTER_BYTES + CPU_RAM_BYTES - 1;

const DESIRED_HZ_TIM1: f64 = 2.0;
const TIM1_PRESCALER: u64 = 1024;
const INTERRUPT_EVERY_1_HZ_1024_PRESCALER: u16 =
    ((CPU_FREQUENCY_HZ as f64 / (DESIRED_HZ_TIM1 * TIM1_PRESCALER as f64)) as u64 - 1) as u16;

// const DESIRED_HZ_TIM0: f64 = 30.0;
// const TIM0_PRESCALER: u64 = 1024;
// const INTERRUPT_EVERY_30_HZ_1024_PRESCALER: u8 = ((CPU_FREQUENCY_HZ as f64 / (DESIRED_HZ_TIM0 * TIM0_PRESCALER as f64)) as u64 - 1) as u8;

const BAUD: u64 = 9600;
const MYUBRR: u16 = (CPU_FREQUENCY_HZ / 16 / BAUD - 1) as u16;

mod initialize {
    use core::ptr;

    extern "C" {
        #[naked]
        fn __initialize_memory();

        #[no_mangle]
        static mut __data_start: u8;
        #[no_mangle]
        static mut __data_end: u8;
        #[no_mangle]
        static mut __data_load_start: u8;

        #[no_mangle]
        static mut __bss_start: u8;
        #[no_mangle]
        static mut __bss_end: u8;
    }

    unsafe fn zero_out_bss() {
        let mut bss: *mut u8 = &mut __bss_start;
        let bss_end: *mut u8 = &mut __bss_end;

        while bss != bss_end {
            ptr::write_volatile(bss, 0);
            bss = bss.offset(1);
        }
    }

    unsafe fn load_data() {
        let mut data: *mut u8 = &mut __data_start;
        let data_end: *mut u8 = &mut __data_end;
        let mut data_load: *mut u8 = &mut __data_load_start;

        while data != data_end {
            let d;
            llvm_asm! {"lpm $0, $1+"
                : /* output */ "=r"(d), "=Z"(data_load)
                : /* input */  "Z"(data_load)
                : /* clobber */
                : /* options */
            };
            ptr::write_volatile(data, d);
            data = data.offset(1);
        }
    }

    pub unsafe fn memory() {
        load_data();
        zero_out_bss();

        //__initialize_memory();
    }

    #[allow(unused)]
    pub fn info() -> (usize, usize) {
        unsafe {
            let data_len = &__data_end as *const u8 as usize - &__data_start as *const u8 as usize;
            let bss_len = &__bss_end as *const u8 as usize - &__bss_start as *const u8 as usize;
            (data_len, bss_len)
        }
    }
}

#[no_mangle]
pub extern "C" fn main() -> ! {
    // TODO: we know interrupts are off, don't we?
    without_interrupts(|| {
        unsafe {
            // The ABI requires that r1 starts as zero
            llvm_asm!("eor r1, r1");
            ptr::write_volatile(SP, CPU_INITIAL_STACK_POINTER);
            initialize::memory();
        }

        unsafe {
            // Configure all Port B pins as outputs
            ptr::write_volatile(DDRB, 0xFF);
            // Turn on all Port B pins
            // ptr::write_volatile(PORTB, 0xFF);
        }

        timer1::Timer::new()
            .waveform_generation_mode(
                timer1::WaveformGenerationMode::ClearOnTimerMatchOutputCompare,
            )
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

    // undefined reference to `__udivmodhi4'
    // println!("Hello, {}", "world");

    // serial::transmit(b'O');
    // serial::transmit(b'K');
    // serial::transmit(b'\r');
    // serial::transmit(b'\n');
    // serial::receive();

    // busy_sleep_100ms();

    // exercise::serial();
    // exercise::reference();
    // exercise::i32();
    // exercise::generator();

    // fut::do_futures();

    // spin_loop();
    bracketed_echo();
}

#[inline(never)]
fn busy_sleep_100ms() {
    // Numbers from trial and error; measured at 102.8ms
    for _ in 0u8..5 {
        for _ in 0x00u8..0xFF {
            for _ in 0x00u8..0xFF {
                unsafe { llvm_asm!("NOP") };
            }
        }
    }
}

#[allow(unused)]
fn spin_loop() -> ! {
    loop {
        write::strln("....loooooooooping...");
        for i in b'0'..=b'9' {
            serial::transmit(i);
            serial::transmit(b' ');
        }
        serial::transmit(b'\r');
        serial::transmit(b'\n');
    }
}

#[allow(unused)]
fn bracketed_echo() -> ! {
    loop {
        if let Some(b) = serial::try_receive() {
            serial::transmit(b'>');
            serial::transmit(b);
            serial::transmit(b'<');
        }
        // forever!
    }
}

#[allow(unused)]
mod exercise {
    use crate::write;
    use core::{fmt::Write, ptr};
    use ruduino::serial;

    pub fn serial() {
        serial::transmit(b'A');
        write::newline();

        for i in 0..4 {
            let b = unsafe { *write::TO_HEX.get_unchecked(i) };
            serial::transmit(b);
        }
        write::newline();

        write::u8_hex(0x00);
        write::newline();
        write::u8_hex(0x05);
        write::newline();
        write::u8_hex(0x50);
        write::newline();
        write::u8_hex(0xFF);
        write::newline();

        write::slice_hex(&[0x00, 0x05, 0x50, 0xFF]);
        write::newline();

        write::raw("raw");

        // writeln!(write::SERIAL, "writeln!").unwrap();
        // write::newline();

        serial::transmit(b'Z');
        write::newline();
    }

    #[inline(never)]
    pub fn reference() -> ! {
        let mut on = false;
        let on = &mut on;
        loop {
            if *on {
                write::strln("off");
            } else {
                write::strln("on");
            }
            *on = !*on;
        }
    }

    #[inline(never)]
    pub fn i32() -> ! {
        let mut a: i32 = 0;
        unsafe { ptr::write_volatile(&mut a, 0x56_AB_CD_EF) };
        let b = unsafe { ptr::read_volatile(&mut a) };
        loop {
            write::slice_hex(&b.to_be_bytes());
            crate::busy_sleep_100ms();
        }
    }

    #[inline(never)]
    pub fn generator() -> ! {
        use core::ops::Generator;

        let mut gen = static move || {
            let mut on = false;
            let on = &mut on;
            loop {
                if *on {
                    crate::write::strln("on");
                } else {
                    crate::write::strln("off");
                }
                *on = !*on;
                yield;
            }
        };
        let mut gen = unsafe { core::pin::Pin::new_unchecked(&mut gen) };
        loop {
            match gen.as_mut().resume(()) {
                core::ops::GeneratorState::Yielded(()) => {}
                core::ops::GeneratorState::Complete(_) => unreachable!(),
            }
        }
    }
}

#[allow(unused)]
mod write {
    use core::fmt::{self, Write};
    use ruduino::serial;

    pub static TO_HEX: &[u8; 16] = b"0123456789ABCDEF";
    pub const SERIAL: SuperSerial = SuperSerial(());

    pub fn slice_hex(v: &[u8]) {
        for &b in v {
            u8_hex(b);
        }
    }

    pub fn u8_hex(v: u8) {
        let top_idx = (v >> 4) & 0b1111;
        let bot_idx = (v >> 0) & 0b1111;

        let top = unsafe { *TO_HEX.get_unchecked(top_idx as usize) };
        let bot = unsafe { *TO_HEX.get_unchecked(bot_idx as usize) };

        // Useful for avoiding global variables

        // let c = |v| match v {
        //     0..=9 => b'0' + v,
        //     _ => v - 10 + b'A',
        // };
        // let top = c(top_idx);
        // let bot = c(bot_idx);

        serial::transmit(top);
        serial::transmit(bot);
    }

    pub fn newline() {
        serial::transmit(b'\r');
        serial::transmit(b'\n');
    }

    pub fn raw(s: &str) {
        for b in s.bytes() {
            serial::transmit(b);
        }
        newline();
    }

    pub fn str(s: &str) {
        SERIAL.write_str(s).unwrap();
    }

    pub fn strln(s: &str) {
        SERIAL.write_str(s).unwrap();
        SERIAL.write_str("\r\n").unwrap();
    }

    pub struct SuperSerial(());

    impl fmt::Write for SuperSerial {
        fn write_str(&mut self, s: &str) -> Result<(), fmt::Error> {
            for b in s.bytes() {
                serial::transmit(b);
            }
            Ok(())
        }
    }
}

// mod fut {
//     use core::{
//         future::Future,
//         pin::Pin,
//         ptr,
//         task::{Context, Poll, Waker},
//     };
//     use embrio_async::embrio_async;
//     use ruduino::{serial, RXCIE0, UCSR0B, UDRIE0};

//     fn set_bit_in(register: *mut u8, bit: u8) {
//         unsafe {
//             let old = ptr::read_volatile(register);
//             let new = old | bit;
//             ptr::write_volatile(register, new);
//         }
//     }

//     fn unset_bit_in(register: *mut u8, bit: u8) {
//         unsafe {
//             let old = ptr::read_volatile(register);
//             let new = old ^ bit;
//             ptr::write_volatile(register, new);
//         }
//     }

//     // Nemo157:
//     //
//     // I’m pretty sure that implementation is unsound for spurious
//     // wakeups
//     //
//     // You could poll the future once, set the waker and enable the
//     // interrupt, then poll the future again and have the interrupt
//     // trigger during setting the waker again
//     //
//     // If the interrupt occurs mid-write to TX_WAKER then you will be
//     // creating an &mut Option<Waker> for .take() during that
//     // concurrent modification

//     #[allow(unused)]
//     struct Serial;

//     #[allow(unused)]
//     impl Serial {
//         // TODO: Maybe take a slice instead?
//         fn tx(&self, byte: u8) -> SerialTx {
//             SerialTx(byte)
//         }

//         // TODO: Maybe take a slice instead?
//         fn rx<'a>(&self, byte: &'a mut u8) -> SerialRx<'a> {
//             SerialRx(byte)
//         }
//     }

//     struct SerialRx<'a>(&'a mut u8);

//     static mut RX_WAKER: Option<Waker> = None;

//     #[inline(always)]
//     pub fn rx_interrupt_handler() {
//         // Safety:
//         // We are on a single-threaded CPU, so static mutable shoudn't matter.
//         unsafe {
//             if let Some(waker) = RX_WAKER.take() {
//                 // Notify our waker to poll the future again
//                 waker.wake();

//                 // We must either read from the buffer or disable the
//                 // interrupt to prevent re-invoking the interrupt
//                 // handler immediately.
//                 disable_serial_rx_interrupt();
//             }
//         }
//     }

//     fn enable_serial_rx_interrupt() {
//         set_bit_in(UCSR0B, RXCIE0);
//     }

//     fn disable_serial_rx_interrupt() {
//         unset_bit_in(UCSR0B, RXCIE0);
//     }

//     impl<'a> Future for SerialRx<'a> {
//         type Output = ();

//         fn poll(self: Pin<&mut Self>, ctx: &mut Context) -> Poll<Self::Output> {
//             match serial::try_receive() {
//                 Some(v) => {
//                     *Pin::get_mut(self).0 = v;
//                     Poll::Ready(())
//                 }
//                 None => {
//                     // Safety:
//                     // We are on a single-threaded CPU, so static mutable shoudn't matter.
//                     unsafe {
//                         RX_WAKER = Some(ctx.waker().clone());
//                     }
//                     enable_serial_rx_interrupt();

//                     Poll::Pending
//                 }
//             }
//         }
//     }
//     mod hack {
//         pub struct RawWaker {
//             pub data: *const (),
//             pub vtable: &'static mut RawWakerVTable,
//         }

//         pub struct RawWakerVTable {
//             pub clone: unsafe fn(*const ()) -> RawWaker,
//             pub wake: unsafe fn(*const ()),
//             pub wake_by_ref: unsafe fn(*const ()),
//             pub drop: unsafe fn(*const ()),
//         }

//         #[repr(transparent)]
//         pub struct Waker {
//             pub waker: RawWaker,
//         }
//     }

//     struct SerialTx(u8);

//     static mut TX_WAKER: Option<Waker> = None;

//     #[inline(always)]
//     pub fn tx_empty_interrupt_handler() {
//         // Safety:
//         // We are on a single-threaded CPU, so static mutable shoudn't matter.

//         unsafe {
//             if let Some(waker) = TX_WAKER.take() {
//                 // Notify our waker to poll the future again
//                 use ruduino::{io::PORT_D, Bit::*};
//                 PORT_D.data().toggle_bit(Bit2);

//                 //let hw: &mut hack::Waker = &mut *(&mut waker as *mut Waker as *mut hack::Waker);

//                 // woooooeeeeee
//                 //use core::mem;
//                 //hw.waker.vtable.wake = mem::transmute(hw.waker.data);

//                 // f901 (swapped?)
//                 // let w = hw.waker.vtable.wake;
//                 // crate::write::slice_hex(&(w as usize).to_be_bytes());

//                 // // 01f9 Executor address (right)
//                 // let d = hw.waker.data;
//                 // crate::write::slice_hex(&(d as usize).to_be_bytes());

//                 // crate::busy_sleep_100ms();

//                 waker.wake();
//                 // We must either write to the buffer or disable the
//                 // interrupt to prevent re-invoking the interrupt
//                 // handler immediately.
//                 disable_serial_tx_empty_interrupt();
//             }
//         }
//     }

//     fn enable_serial_tx_empty_interrupt() {
//         set_bit_in(UCSR0B, UDRIE0);
//     }

//     fn disable_serial_tx_empty_interrupt() {
//         unset_bit_in(UCSR0B, UDRIE0);
//     }

//     impl Future for SerialTx {
//         type Output = ();

//         fn poll(self: Pin<&mut Self>, ctx: &mut Context) -> Poll<Self::Output> {
//             use ruduino::{io::PORT_D, Bit::*};

//             match serial::try_transmit(self.0) {
//                 Ok(()) => Poll::Ready(()),
//                 Err(()) => {
//                     // Safety:
//                     // We are on a single-threaded CPU, so static mutable shoudn't matter.
//                     unsafe {
//                         TX_WAKER = Some(ctx.waker().clone());
//                     }
//                     enable_serial_tx_empty_interrupt();

//                     Poll::Pending
//                 }
//             }
//         }
//     }

//     #[embrio_async]
//     async fn bracketed_echo() -> ! {
//         loop {
//             serial::transmit(b'A');

//             // let mut buf = 0;
//             // Serial.rx(&mut buf).await;

//             Serial.tx(b'>').await;
//             // Serial.tx(buf).await;
//             Serial.tx(b'<').await;

//             serial::transmit(b'B');
//         }
//     }

//     #[allow(unused)]
//     pub fn do_futures() -> ! {
//         use embrio_executor::Executor;

//         static mut EXECUTOR: Executor = Executor::new();
//         let executor = unsafe { &mut EXECUTOR };
//         executor.block_on(bracketed_echo())
//     }
// }

#[no_mangle]
extern "C" fn abort() -> ! {
    loop {
        write::strln("ABORT");
    }
}

// #[no_mangle]
// extern "C" fn __sync_lock_test_and_set_1(ptr: *mut u8, desired: u8) -> u8 {
//     without_interrupts(|| unsafe {
//         let old = *ptr;
//         *ptr = desired;
//         old
//     })
// }
