#include <uart.h>
#include <stdio.h>
#include <util/delay.h>

#define TWI_IMPLEMENTATION
#include <twi.h>

#define SLAVE_ADDRESS 0x10

#define CONTENT "Hello, world!"
#define CONTENT_LENGTH 13

int main(void)
{
    printf_init();
    sei();

    printf("[TWI] Transmitter master init...");
    twi_master_init();
    printf("OK\n");

    for (;;) {
        printf("[TWI] Writing to slave...");
        twi_master_write_to(SLAVE_ADDRESS, (uint8_t*)CONTENT, CONTENT_LENGTH);
        printf("OK\n");

        _delay_ms(100);
    }

    return 0;
}
