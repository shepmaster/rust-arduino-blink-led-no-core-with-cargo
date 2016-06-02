use avr_core::prelude::v1::*;
use avr_core::intrinsics::volatile_store;

use ::avr::*;

pub enum ClockSource {
    None,
    Prescale1,
    Prescale8,
    Prescale64,
    Prescale256,
    Prescale1024,
    ExternalFalling,
    ExternalRising,
}

impl ClockSource {
    pub fn bits(&self) -> u8 {
        use self::ClockSource::*;

        match *self {
            None            =>    0 |    0 |    0,
            Prescale1       =>    0 |    0 | CS10,
            Prescale8       =>    0 | CS11 |    0,
            Prescale64      =>    0 | CS11 | CS10,
            Prescale256     => CS12 |    0 |    0,
            Prescale1024    => CS12 |    0 | CS10,
            ExternalFalling => CS12 | CS11 |    0,
            ExternalRising  => CS12 | CS11 | CS10,
        }
    }

    pub fn mask() -> u8 {
        !(CS12 | CS11 | CS10)
    }
}

pub enum WaveformGenerationMode {
    Normal,
    PwmPhaseCorrect8Bit,
    PwmPhaseCorrect9Bit,
    PwmPhaseCorrect10Bit,
    ClearOnTimerMatchOutputCompare,
    FastPwm8Bit,
    FastPwm9Bit,
    FastPwm10Bit,
    PwmPhaseAndFrequencyCorrectInputCapture,
    PwmPhaseAndFrequencyCorrectOutputCompare,
    PwmPhaseCorrectInputCapture,
    PwmPhaseCorrectOutputCompare,
    ClearOnTimerMatchInputCapture,
    FastPwmInputCapture,
    FastPwmOutputCompare,
}

impl WaveformGenerationMode {
    /// Returns bits for TCCR1A, TCCR1B
    pub fn bits(&self) -> (u8, u8) {
        use self::WaveformGenerationMode::*;

        // It makes more sense to return bytes (A,B), but the manual
        // lists the table as (B,A). We match the manual here for
        // inspection purposes and flip the values for sanity
        // purposes.
        let (b, a) = match *self {
            Normal                                   => (    0 |    0,      0 |     0),
            PwmPhaseCorrect8Bit                      => (    0 |    0,      0 | WGM10),
            PwmPhaseCorrect9Bit                      => (    0 |    0,  WGM11 |     0),
            PwmPhaseCorrect10Bit                     => (    0 |    0,  WGM11 | WGM10),
            ClearOnTimerMatchOutputCompare           => (    0 | WGM12,     0 |     0),
            FastPwm8Bit                              => (    0 | WGM12,     0 | WGM10),
            FastPwm9Bit                              => (    0 | WGM12, WGM11 |     0),
            FastPwm10Bit                             => (    0 | WGM12, WGM11 | WGM10),
            PwmPhaseAndFrequencyCorrectInputCapture  => (WGM13 |    0,      0 |     0),
            PwmPhaseAndFrequencyCorrectOutputCompare => (WGM13 |    0,      0 | WGM10),
            PwmPhaseCorrectInputCapture              => (WGM13 |    0,  WGM11 |     0),
            PwmPhaseCorrectOutputCompare             => (WGM13 |    0,  WGM11 | WGM10),
            ClearOnTimerMatchInputCapture            => (WGM13 | WGM12,     0 |     0),
            // Reserved                              => (WGM13 | WGM12,     0 | WGM10),
            FastPwmInputCapture                      => (WGM13 | WGM12, WGM11 |     0),
            FastPwmOutputCompare                     => (WGM13 | WGM12, WGM11 | WGM10),
        };

        (a, b)
    }

    pub fn mask() -> (u8, u8) {
        (!(WGM10 | WGM11), !(WGM12 | WGM13))
    }
}

pub struct Timer {
    a: u8,
    b: u8,
    c: u8,
    output_compare_1: Option<u16>,
}

impl Timer {
    pub fn new() -> Self {
        Timer {
            a: 0,
            b: 0,
            c: 0,
            output_compare_1: None,
        }
    }

    pub fn clock_source(mut self, source: ClockSource) -> Self {
        self.b &= ClockSource::mask();
        self.b |= source.bits();
        self
    }

    pub fn waveform_generation_mode(mut self, mode: WaveformGenerationMode) -> Self {
        let (a, b) = WaveformGenerationMode::mask();
        self.a &= a;
        self.b &= b;

        let (a, b) = mode.bits();
        self.a |= a;
        self.b |= b;

        self
    }

    pub fn output_compare_1(mut self, value: Option<u16>) -> Self {
        self.output_compare_1 = value;
        self
    }

    pub fn configure(self) {
        unsafe {
            volatile_store(TCCR1A, self.a);
            volatile_store(TCCR1B, self.b);
            volatile_store(TCCR1C, self.c);

            // Reset counter to zero
            volatile_store(TCNT1, 0);

            if let Some(v) = self.output_compare_1 {
                // Set the match
                volatile_store(OCR1A, v);

                // Enable compare interrupt
                volatile_store(TIMSK1, OCIE1A);
            }
        }
    }
}
