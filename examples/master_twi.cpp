extern "C" {
#include <uart.h>
}

#include <stdio.h>
#include <util/delay.h>

#define TWO_WIRE_IMPLEMENTATION
#include <TwoWire.hpp>

constexpr static auto SLAVE_ADDRESS = 0x10;

auto main() -> int
{
    printf_init();
    printf("[DEBUG] Master initialized\n");

    two_wire.enable();

    for (;;) {
        // Request from slave "Hello, world\n" which is 13 chars
        two_wire.request_from(SLAVE_ADDRESS, 13);
        printf("requested bytes\n");
        while (!two_wire.eos()) {
            printf("%c", two_wire.read());
        }
        _delay_ms(500);
    }
}
