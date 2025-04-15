#include <uart.h>
#include <stdio.h>
#include <util/delay.h>

#define TWI_IMPLEMENTATION
#include <twi.h>

void receive_callback(volatile uint8_t* buffer, volatile uint8_t size)
{
    for (uint8_t i = 0; i < size; ++i) {
        printf("%c", buffer[i]);
    }

    printf("\n");
}

int main(void)
{
    printf_init();
    sei();

    printf("Initializing Twi Slave...");
    twi_slave_init(0x10);
    printf("OK\n");

    printf("Setting slave receive callback...");
    twi_set_slave_receive_callback(receive_callback);
    printf("OK\n");

    for (;;) {
        _delay_ms(100);
    }

    return 0;
}
