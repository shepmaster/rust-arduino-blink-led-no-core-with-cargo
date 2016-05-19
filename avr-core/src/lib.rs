#![feature(no_core)]
#![feature(lang_items)]
#![feature(fundamental)]
#![feature(intrinsics)]
#![feature(on_unimplemented)]
#![feature(optin_builtin_traits)]
#![feature(reflect)]
#![feature(unboxed_closures)]
#![feature(associated_type_defaults)]

#![no_core]

pub mod prelude {
    pub use option::Option::*;
    pub use ops::*;
}

#[lang = "eh_personality"]
extern fn eh_personality() {}

#[lang = "panic"]
pub fn panic(expr_file_line: &(&'static str, &'static str, u32)) -> ! {
    loop {}
}

// All this is copied from libcore
pub mod option;
pub mod intrinsics;
pub mod clone;
pub mod cmp;
pub mod marker;
pub mod ops;
