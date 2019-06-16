#![feature(lang_items)]
#![feature(asm)]
#![feature(naked_functions)]
#![feature(abi_avr_interrupt)]
#![feature(never_type)]
#![feature(async_await)]
#![feature(generators)]

#![allow(unused_imports)]

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
            crate::write_str(l.file());
            crate::write_str(" (");
            crate::write_slice_hex(&l.line().to_be_bytes());
            crate::write_str(" / ");
            crate::write_slice_hex(&l.column().to_be_bytes());
            crate::write_str(" )");
        }
        crate::write_strln("");
    }
}

#[no_mangle]
pub unsafe extern "avr-interrupt" fn _ivr_timer1_compare_a() {
    let prev_value = read_volatile(PORTB);
    write_volatile(PORTB, prev_value ^ PINB5);
}

#[no_mangle]
pub unsafe extern "avr-interrupt" fn _ivr_usart_rx_complete() {
    fut::rx_interrupt_handler();
}

#[no_mangle]
pub unsafe extern "avr-interrupt" fn _ivr_usart_udr_empty() {
    fut::tx_empty_interrupt_handler();
}

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
    use core::ptr::{read_volatile, write_volatile};

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

    #[inline(never)]
    unsafe fn zero_out_bss() {
        let mut bss: *mut u8 = &mut __bss_start;
        let bss_end: *mut u8 = &mut __bss_end;

        while bss != bss_end {
            write_volatile(bss, 0);
            bss = bss.offset(1);
        }
    }

    #[inline(never)]
    unsafe fn load_data() {
        let mut data: *mut u8 = &mut __data_start;
        let data_end: *mut u8 = &mut __data_end;
        let mut data_load: *mut u8 = &mut __data_load_start;

        while data != data_end {
            let d;
            asm! {"lpm $0, $1+"
                : /* output */ "=r"(d), "=Z"(data_load)
                : /* input */  "Z"(data_load)
                : /* clobber */
                : /* options */
            };
            write_volatile(data, d);
            data = data.offset(1);
        }
    }

    #[inline(never)]
    pub unsafe fn memory() {
        load_data();
        zero_out_bss();

        //__initialize_memory();
    }

    #[inline(never)]
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
pub extern fn main() -> ! {
    without_interrupts(|| { // TODO: we know interrupts are off, don't we?
        unsafe {
            // The ABI requires that r1 starts as zero
            asm!("eor r1, r1");
            write_volatile(SP, CPU_INITIAL_STACK_POINTER);
            initialize::memory();
        }

        unsafe {
            // Configure all Port B pins as outputs
            write_volatile(DDRB, 0xFF);
            // Turn on all Port B pins
            // write_volatile(PORTB, 0xFF);
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

    serial::transmit(b'O');
    serial::transmit(b'K');
    serial::transmit(b'\r');
    serial::transmit(b'\n');
    // serial::receive();

    // exercise_serial();

    fut::do_futures();

    // spin_loop();
    // bracketed_echo();
}

#[allow(unused)]
fn exercise_serial() {
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

    // Formatting code disabled in libcore
    // writeln!(SuperSerial, "writeln!").unwrap();
    // write_newline();

    serial::transmit(b'Z');
    write_newline();
}

#[allow(unused)]
fn spin_loop() {
    loop {
        crate::write_strln("....loooooooooping...");
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

pub static TO_HEX: &[u8; 16] = b"0123456789ABCDEF";

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

#[inline(never)]
pub fn write_newline() {
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

    // Nemo157:
    //
    // I’m pretty sure that implementation is unsound for spurious
    // wakeups
    //
    // You could poll the future once, set the waker and enable the
    // interrupt, then poll the future again and have the interrupt
    // trigger during setting the waker again
    //
    // If the interrupt occurs mid-write to TX_WAKER then you will be
    // creating an &mut Option<Waker> for .take() during that
    // concurrent modification

    struct Serial;

    impl Serial {
        // TODO: Maybe take a slice instead?
        fn tx(&self, byte: u8) -> SerialTx {
            SerialTx(byte)
        }

        // TODO: Maybe take a slice instead?
        fn rx<'a>(&self, byte: &'a mut u8) -> SerialRx<'a> {
            SerialRx(byte)
        }
    }

    struct SerialRx<'a>(&'a mut u8);

    static mut RX_WAKER: Option<Waker> = None;

    #[inline(always)]
    pub fn rx_interrupt_handler() {
        // Safety:
        // We are on a single-threaded CPU, so static mutable shoudn't matter.
        unsafe {
            if let Some(waker) = RX_WAKER.take() {
                // Notify our waker to poll the future again
                waker.wake();

                // We must either read from the buffer or disable the
                // interrupt to prevent re-invoking the interrupt
                // handler immediately.
                disable_serial_rx_interrupt();
            }
        }
    }

    fn enable_serial_rx_interrupt() {
        set_bit_in(UCSR0B, RXCIE0);
    }

    fn disable_serial_rx_interrupt() {
        unset_bit_in(UCSR0B, RXCIE0);
    }

    impl<'a> Future for SerialRx<'a> {
        type Output = ();

        fn poll(self: Pin<&mut Self>, ctx: &mut Context) -> Poll<Self::Output> {
            match serial::try_receive() {
                Some(v) => {
                    *Pin::get_mut(self).0 = v;
                    Poll::Ready(())
                },
                None => {
                    // Safety:
                    // We are on a single-threaded CPU, so static mutable shoudn't matter.
                    unsafe {
                        RX_WAKER = Some(ctx.waker().clone());
                    }
                    enable_serial_rx_interrupt();

                    Poll::Pending
                }
            }
        }
    }

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

                // We must either write to the buffer or disable the
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
            use ruduino::{Bit::*, io::PORT_D};
            PORT_D.data().toggle_bit(Bit3);

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
        // Serial.tx(b'X').await;
        // Serial.tx(b'\r').await;
        // Serial.tx(b'\n').await;

        let mut buf = 0;
        Serial.rx(&mut buf).await;
        Serial.tx(b'>').await;
        Serial.tx(buf).await;
        Serial.tx(b'<').await;
    }

    pub fn do_futures() -> ! {
        use embrio_executor::Executor;

        static mut EXECUTOR: Executor = Executor::new();
        loop {
            crate::write_strln("loop ->");
            let executor = unsafe { &mut EXECUTOR };
            executor.block_on(example());
            crate::write_strln("<- loop");
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
