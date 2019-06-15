COMPILED:=avr-atmega328p/release/blink.elf
HEX:=blink.hex
SERIAL_PORT:=/dev/cu.usbmodem143201

all: ${HEX}

.PHONY: ${COMPILED}
${COMPILED}:
	xargo build --target ${PWD}/avr-atmega328p --release

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
