COMPILED:=target/arduino/release/blink.elf
HEX:=blink.hex

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
	avrdude -p atmega328p -c arduino -P /dev/cu.usbmodem1411 -U flash:w:$<:i
