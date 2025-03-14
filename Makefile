BUILD_DIR=build
SRC_DIR=examples

UART_INCLUDE=include
I2C_INCLUDE=include

CC=avr-gcc
CFLAGS=-Wall -Wextra -Werror -Wshadow -Wpedantic
CSTD=-std=gnu99
COPTS=-O3
CROSS_FLAGS=-mmcu=atmega2560 -DF_CPU=16000000UL -D__AVR_3_BYTE_PC__
INCLUDE_DIRS=-I$(UART_INCLUDE)/ -I$(I2C_INCLUDE)/

AVROBJCOPY=avr-objcopy
AVROBJCOPY_FLAGS=-O ihex -R .eeprom

AVRDUDE=avrdude
AVRDUDE_PORT=/dev/ttyACM0
AVRDUDE_BAUDRATE = 115200
AVRDUDE_BOOTLOADER = wiring
AVRDUDE_FLAGS += -p m2560 -P $(AVRDUDE_PORT) -b $(AVRDUDE_BAUDRATE)
AVRDUDE_FLAGS += -D -q -V -C /usr/share/arduino/hardware/tools/avr/../avrdude.conf -c $(AVRDUDE_BOOTLOADER)

.phony: clean all

all: build \
	$(BUILD_DIR)/i2c_basic_test.elf \

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) $(CFLAGS) $(CSTD) $(COPTS) $(CROSS_FLAGS) -c -o $@ $< $(INCLUDE_DIRS)

$(BUILD_DIR)/%.elf: $(BUILD_DIR)/%.o $(BUILD_DIR)/uart.o
	$(CC) $(CFLAGS) $(CSTD) $(COPTS) $(CROSS_FLAGS) -o $@ $^

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf
	$(AVROBJCOPY) $(AVROBJCOPY_FLAGS) $< $@
	$(AVRDUDE) $(AVRDUDE_FLAGS) -U flash:w:$@:i

build:
	mkdir -p $(BUILD_DIR)

clean:
	rm -rf $(BUILD_DIR)
