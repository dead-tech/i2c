#include <uart.h>
#include <stdio.h>
#include <util/delay.h>

#define TWI_IMPLEMENTATION
#include <twi.h>

void transmit_callback(void)
{
    twi_slave_prepare_transmission((uint8_t*)"hello ", 6);
}

int main(void)
{
    printf_init();
    sei();

    twi_slave_init(0x10);
    printf("Twi Slave initialized...\n");

    twi_set_slave_transmit_callback(transmit_callback);
    printf("Twi Slave Transmit callback was set...\n");

    for (;;) {
        _delay_ms(100);
    }

    return 0;
}
