.section .text
.globl __initialize_memory
.type __initialize_memory, @function

;;; Does not handle the region pointer!
__initialize_memory:

__initialize_memory_init_data:
        ldi     r30, lo8(__data_load_start)
        ldi     r31, hi8(__data_load_start) ; Z
        ldi     r24, lo8(__data_end)
        ldi     r25, hi8(__data_end)
        ldi     r26, lo8(__data_start)
        ldi     r27, hi8(__data_start)      ; X
        rjmp    __initialize_memory_init_data_check

__initialize_memory_init_data_copy:
        lpm     r18, Z+
        st      X+, r18

__initialize_memory_init_data_check:
        cp      r26, r24
        cpc     r27, r25
        brne    __initialize_memory_init_data_copy

__initialize_memory_init_bss:
         ldi     r24, lo8(__bss_end)
         ldi     r25, hi8(__bss_end)
         ldi     r26, lo8(__data_end) ; X
         ldi     r27, hi8(__data_end)
         ldi     r18, 0x00
         rjmp    __initialize_memory_init_bss_check

__initialize_memory_init_bss_copy:
        st      X+, r18

__initialize_memory_init_bss_check:
        cp      r26, r24
        cpc     r27, r25
        brne    __initialize_memory_init_bss_copy

__initialize_memory_finish:
        ret
