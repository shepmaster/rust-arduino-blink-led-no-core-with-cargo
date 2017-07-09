.section .text
.globl __initialize_memory
.type __initialize_memory, @function

;;; Does not handle the region pointer!
__initialize_memory:
        ldi r31, hi8(__data_load_start)
        ldi r30, lo8(__data_load_start)         ; Z

        ldi r29, hi8(__data_load_end)
        ldi r28, lo8(__data_load_end)           ; Y

        ldi r27, hi8(__data_start)
        ldi r26, lo8(__data_start)              ; X

        sub r28, r30
        sbc r29, r31    ; Y now contains the length of bytes
        rjmp check

entrypoint:
        lpm r0, Z+      ; Load from program memory, increment pointer
        st X+, r0       ; Store to RAM, increment pointer

        subi r28, 1     ; Decrement the count
        sbci r29, 0

check:
        brne entrypoint ; Exit when all bytes copied
        ret
