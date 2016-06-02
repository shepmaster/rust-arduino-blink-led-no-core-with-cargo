use avr_core::prelude::v1::*;
use avr_core::marker::PhantomData;

pub struct DisableInterrupts(PhantomData<()>);

impl DisableInterrupts {
    pub fn new() -> DisableInterrupts {
        unsafe { asm!("CLI") }
        DisableInterrupts(PhantomData)
    }
}

impl Drop for DisableInterrupts {
    fn drop(&mut self) {
        unsafe { asm!("SEI") }
    }
}

pub fn without_interrupts<F, T>(f: F) -> T
    where F: FnOnce() -> T
{
    let _disabled = DisableInterrupts::new();
    f()
}
