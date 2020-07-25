COMPILED:=target/avr-atmega328p/release/blink.elf
HEX:=blink.hex
SERIAL_PORT:=/dev/cu.usbmodem143201

RUSTFLAGS:=-Zverify-llvm-ir

all: ${HEX}

.PHONY: ${COMPILED}
${COMPILED}:
	RUSTFLAGS=${RUSTFLAGS} cargo build -Z build-std=core --target avr-atmega328p.json --release

# Convert binary to an Intel HEX file for upload
${HEX}: ${COMPILED}
	avr-objcopy -O ihex -R .eeprom $< $@

# Download the HEX to the board
.PHONY: program
program: ${HEX}
	avrdude -p atmega328p -c arduino -P ${SERIAL_PORT} -U flash:w:$<:i

.PHONY: connect-terminal
connect-terminal:
	picocom ${SERIAL_PORT}


.PHONY: simulate-avr
simulate-avr: ${COMPILED}
	simavr \
	  -m atmega328p \
	  -f 16000000 \
	  -g \
	  -v -v -v -v -v -v -v \
	  $<

.PHONY: simulate-gdb
simulate-gdb: ${COMPILED}
	avr-gdb -x simulate.gdbinit -tui $<
