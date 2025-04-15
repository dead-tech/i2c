#include <uart.h>
#include <stdio.h>
#include <util/delay.h>

#define TWI_IMPLEMENTATION
#include <twi.h>

#define SLAVE_ADDRESS 0x10

#define CONTENT "Hello from transmitter slave"
#define CONTENT_LENGTH 28

void transmit_callback(void)
{
    twi_slave_prepare_transmission((uint8_t*)CONTENT, CONTENT_LENGTH);
}

int main(void)
{
    printf_init();
    sei();

    printf("[TWI] Transmitter Slave init...");
    twi_slave_init(SLAVE_ADDRESS);
    printf("OK\n");

    printf("[TWI] Set transmit callback...");
    twi_set_slave_transmit_callback(transmit_callback);
    printf("OK\n");

    for (;;) {
        _delay_ms(100);
    }

    return 0;
}
