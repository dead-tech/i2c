#include <uart.h>
#include <stdio.h>
#include <util/delay.h>

#define TWI_IMPLEMENTATION
#include <twi.h>

#define SLAVE_ADDRESS 0x10
#define CONTENT_LENGTH 28

int main(void)
{
    printf_init();
    sei();

    printf("[TWI] Receiver Master init...");
    twi_master_init();
    printf("OK\n");

    printf("Read data from slave transmitter\n");
    for (;;) {
        uint8_t data[CONTENT_LENGTH] = {0};
        twi_master_read_from(SLAVE_ADDRESS, data, sizeof(data));

        for (uint8_t i = 0; i < sizeof(data); ++i) {
            printf("%c", (char)data[i]);
        }

        printf("\n");
        _delay_ms(100);
    }

    return 0;
}
