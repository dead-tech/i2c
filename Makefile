BUILD_DIR=build
SRC_DIR=examples

UART_INCLUDE=include
I2C_INCLUDE=include

CC=avr-gcc
CFLAGS=-Wall -Wextra -Werror -Wshadow -Wpedantic
CSTD=-std=gnu99
COPTS=-O3
CROSS_FLAGS=-mmcu=atmega2560 -DF_CPU=16000000UL -D__AVR_3_BYTE_PC__
CROSS_FLAGS_AT_MEGA_328P=-mmcu=atmega328p -DF_CPU=16000000UL -D__AVR_3_BYTE_PC__
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
	$(BUILD_DIR)/slave.elf \
	$(BUILD_DIR)/master.elf \

$(BUILD_DIR)/uart_at_mega_328p.o: $(SRC_DIR)/uart.c
	@echo $(CROSS_FLAGS_AT_MEGA_328P)
	$(CC) $(CFLAGS) $(CSTD) $(COPTS) $(CROSS_FLAGS_AT_MEGA_328P) $(INCLUDE_DIRS) -c -o $@ $<

$(BUILD_DIR)/slave_simple.o: $(SRC_DIR)/slave_simple.c
	$(CC) $(CFLAGS) $(CSTD) $(COPTS) $(CROSS_FLAGS_AT_MEGA_328P) $(INCLUDE_DIRS) -c -o $@ $<

$(BUILD_DIR)/slave_simple.elf: $(BUILD_DIR)/slave_simple.o $(BUILD_DIR)/uart_at_mega_328p.o
	$(CC) $(CFLAGS) $(CSTD) $(COPTS) $(CROSS_FLAGS_AT_MEGA_328P) -o $@ $^

$(BUILD_DIR)/slave_simple.hex: $(BUILD_DIR)/slave_simple.elf
	avr-objcopy $(AVROBJCOPY_FLAGS) $< $@
	avrdude -p m328p -P $(AVRDUDE_PORT) -b $(AVRDUDE_BAUDRATE) -D -q -V -C /usr/share/arduino/hardware/tools/avr/../avrdude.conf -c arduino -U flash:w:$@:i

$(BUILD_DIR)/slave.o: $(SRC_DIR)/slave.c
	$(CC) $(CFLAGS) $(CSTD) $(COPTS) $(CROSS_FLAGS_AT_MEGA_328P) $(INCLUDE_DIRS) -c -o $@ $<

$(BUILD_DIR)/slave.elf: $(BUILD_DIR)/slave.o $(BUILD_DIR)/uart_at_mega_328p.o
	$(CC) $(CFLAGS) $(CSTD) $(COPTS) $(CROSS_FLAGS_AT_MEGA_328P) -o $@ $^

$(BUILD_DIR)/slave.hex: $(BUILD_DIR)/slave.elf
	avr-objcopy $(AVROBJCOPY_FLAGS) $< $@
	avrdude -p m328p -P $(AVRDUDE_PORT) -b $(AVRDUDE_BAUDRATE) -D -q -V -C /usr/share/arduino/hardware/tools/avr/../avrdude.conf -c arduino -U flash:w:$@:i

$(BUILD_DIR)/uart_at_mega_2560.o: $(SRC_DIR)/uart.c
	$(CC) $(CFLAGS) $(CSTD) $(COPTS) $(CROSS_FLAGS) -c -o $@ $< $(INCLUDE_DIRS)

$(BUILD_DIR)/master_simple.o: $(SRC_DIR)/master_simple.c
	$(CC) $(CFLAGS) $(CSTD) $(COPTS) $(CROSS_FLAGS) -c -o $@ $< $(INCLUDE_DIRS)

$(BUILD_DIR)/master_simple.elf: $(BUILD_DIR)/master_simple.o $(BUILD_DIR)/uart_at_mega_2560.o
	$(CC) $(CFLAGS) $(CSTD) $(COPTS) $(CROSS_FLAGS) -o $@ $^

$(BUILD_DIR)/master_simple.hex: $(BUILD_DIR)/master_simple.elf
	$(AVROBJCOPY) $(AVROBJCOPY_FLAGS) $< $@
	$(AVRDUDE) $(AVRDUDE_FLAGS) -U flash:w:$@:i

$(BUILD_DIR)/master.o: $(SRC_DIR)/master.c
	$(CC) $(CFLAGS) $(CSTD) $(COPTS) $(CROSS_FLAGS) -c -o $@ $< $(INCLUDE_DIRS)

$(BUILD_DIR)/master.elf: $(BUILD_DIR)/master.o $(BUILD_DIR)/uart_at_mega_2560.o
	$(CC) $(CFLAGS) $(CSTD) $(COPTS) $(CROSS_FLAGS) -o $@ $^

$(BUILD_DIR)/master.hex: $(BUILD_DIR)/master.elf
	$(AVROBJCOPY) $(AVROBJCOPY_FLAGS) $< $@
	$(AVRDUDE) $(AVRDUDE_FLAGS) -U flash:w:$@:i

build:
	mkdir -p $(BUILD_DIR)

clean:
	rm -rf $(BUILD_DIR)
