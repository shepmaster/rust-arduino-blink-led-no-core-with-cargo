COMPILED:=target/avr-atmega328p-new/release/blink.elf
HEX:=blink.hex
SERIAL_PORT:=/dev/cu.usbmodem14301

all: ${HEX}

.PHONY: ${COMPILED}
${COMPILED}:
	cargo +local build -Z build-std=core --target avr-atmega328p-new.json --release

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
