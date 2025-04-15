#include <uart.h>
#include <stdio.h>
#include <util/delay.h>

#define TWI_IMPLEMENTATION
#include <twi.h>

#define SLAVE_ADDRESS 0x10

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

    printf("[TWI] Receiver Slave init...");
    twi_slave_init(SLAVE_ADDRESS);
    printf("OK\n");

    printf("[TWI] Setting receive callback...");
    twi_set_slave_receive_callback(receive_callback);
    printf("OK\n");

    printf("Read data from master transmitter\n");
    for (;;) {
        _delay_ms(100);
    }

    return 0;
}
