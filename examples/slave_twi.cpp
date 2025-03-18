extern "C" {
#include <uart.h>
}

#include <stdio.h>

#define TWO_WIRE_IMPLEMENTATION
#include <TwoWire.hpp>

constexpr static auto SLAVE_ADDRESS = 0x10;

auto main(void) -> int
{
    printf_init();
    printf("[DEBUG] Slave initialized\n");

    two_wire.enable_as_slave(SLAVE_ADDRESS);
    two_wire.on_request([](){
        two_wire.write((const uint8_t*)"Hello, world\n", 13);
    });

    for (;;) {
        _delay_ms(100);
    }
}
