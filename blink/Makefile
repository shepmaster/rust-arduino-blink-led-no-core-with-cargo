COMPILED:=target/arduino/release/blink.elf
HEX:=blink.hex
SERIAL_PORT:=/dev/cu.usbmodem1411

all: ${HEX}

.PHONY: ${COMPILED}
${COMPILED}:
	cargo build --release --target=./arduino.json

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
