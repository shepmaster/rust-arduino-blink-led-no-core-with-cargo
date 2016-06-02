pub const DDRB:   *mut u8 = 0x24 as *mut u8;
pub const PORTB:  *mut u8 = 0x25 as *mut u8;

pub const TCCR0A: *mut u8 = 0x44 as *mut u8;
pub const TCCR0B: *mut u8 = 0x45 as *mut u8;
pub const TCNT0:  *mut u8 = 0x46 as *mut u8;
pub const OCR0A:  *mut u8 = 0x47 as *mut u8;
pub const TIMSK0: *mut u8 = 0x6E as *mut u8;

pub const TCCR1A: *mut u8 = 0x80 as *mut u8;
pub const TCCR1B: *mut u8 = 0x81 as *mut u8;
pub const TCCR1C: *mut u8 = 0x82 as *mut u8;
pub const TCCR1L: *mut u8 = 0x84 as *mut u8;
pub const TCCR1H: *mut u8 = 0x85 as *mut u8;

pub const OCR1A:  *mut u16 = 0x88 as *mut u16;
pub const OCR1AL: *mut u8 = 0x88 as *mut u8;
pub const OCR1AH: *mut u8 = 0x89 as *mut u8;

pub const TCNT1:  *mut u16 = 0x84 as *mut u16;

pub const TCNT1L: *mut u8 = 0x84 as *mut u8;
pub const TCNT1H: *mut u8 = 0x85 as *mut u8;

pub const TIMSK1: *mut u8 = 0x6F as *mut u8;

// Should pins be represented in this way?
pub const PINB5:  u8 = 0b0010_0000;

pub const WGM13:  u8 = 0b0001_0000;
pub const WGM12:  u8 = 0b0000_1000;
pub const WGM11:  u8 = 0b0000_0010;
pub const WGM10:  u8 = 0b0000_0001;

pub const CS12:   u8 = 0b0000_0100;
pub const CS11:   u8 = 0b0000_0010;
pub const CS10:   u8 = 0b0000_0001;
pub const OCIE1A: u8 = 0b0000_0010;
