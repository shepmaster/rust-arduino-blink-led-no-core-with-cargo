.section .ivr, "ax", @progbits
.align 1

ivr:
        jmp _ivr_reset
        jmp _ivr_irq0
        jmp _ivr_irq1
        jmp _ivr_pin_change_0
        jmp _ivr_pin_change_1
        jmp _ivr_pin_change_2
        jmp _ivr_watchdog_timer
        jmp _ivr_timer2_compare_a
        jmp _ivr_timer2_compare_b
        jmp _ivr_timer2_overflow
        jmp _ivr_timer1_capture
        jmp _ivr_timer1_compare_a
        jmp _ivr_timer1_compare_b
        jmp _ivr_timer1_overflow
        jmp _ivr_timer0_compare_a
        jmp _ivr_timer0_compare_b
        jmp _ivr_timer0_overflow
        jmp _ivr_spi_transfer_complete
        jmp _ivr_usart_rx_complete
        jmp _ivr_usart_udr_empty
        jmp _ivr_usart_tx_complete
        jmp _ivr_adc_conversion_complete
        jmp _ivr_eeprom_ready
        jmp _ivr_analog_comparator
        jmp _ivr_two_wire_serial_interface
        jmp _ivr_store_program_memory_ready

_ivr_undefined:
        reti

;;; By default, start all interrupt handlers as doing a no-op
;;; return. Since these symbols are weakly linked, user code can
;;; override each one on demand.
.macro _ivr_default name target=_ivr_undefined
.weak \name
.set  \name, \target
.endm

_ivr_default _ivr_reset main    ; Default to `main` here, no need for indirection
_ivr_default _ivr_irq0
_ivr_default _ivr_irq1
_ivr_default _ivr_pin_change_0
_ivr_default _ivr_pin_change_1
_ivr_default _ivr_pin_change_2
_ivr_default _ivr_watchdog_timer
_ivr_default _ivr_timer2_compare_a
_ivr_default _ivr_timer2_compare_b
_ivr_default _ivr_timer2_overflow
_ivr_default _ivr_timer1_capture
_ivr_default _ivr_timer1_compare_a
_ivr_default _ivr_timer1_compare_b
_ivr_default _ivr_timer1_overflow
_ivr_default _ivr_timer0_compare_a
_ivr_default _ivr_timer0_compare_b
_ivr_default _ivr_timer0_overflow
_ivr_default _ivr_spi_transfer_complete
_ivr_default _ivr_usart_rx_complete
_ivr_default _ivr_usart_udr_empty
_ivr_default _ivr_usart_tx_complete
_ivr_default _ivr_adc_conversion_complete
_ivr_default _ivr_eeprom_ready
_ivr_default _ivr_analog_comparator
_ivr_default _ivr_two_wire_serial_interface
_ivr_default _ivr_store_program_memory_ready
