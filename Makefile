ELF:=target/avr-atmega328p/release/blink.elf
HEX:=blink.hex
SERIAL_PORT:=/dev/cu.usbmodem143201

RUSTFLAGS:=-Zverify-llvm-ir

all: ${HEX}

.PHONY: ${ELF}
${ELF}:
	RUSTFLAGS=${RUSTFLAGS} cargo build -Z build-std=core --release

# Download the ELF to the board
.PHONY: program
program: ${ELF}
	avrdude -p atmega328p -c arduino -P ${SERIAL_PORT} -D -U flash:w:$<

.PHONY: connect-terminal
connect-terminal:
	picocom ${SERIAL_PORT}

.PHONY: simulate-avr
simulate-avr: ${ELF}
	simavr \
	  -m atmega328p \
	  -f 16000000 \
	  -g \
	  -v -v -v -v -v -v -v \
	  $<

.PHONY: simulate-gdb
simulate-gdb: ${ELF}
	avr-gdb -x simulate.gdbinit -tui $<
