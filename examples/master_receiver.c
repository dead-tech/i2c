#include <uart.h>
#include <stdio.h>

#define TWI_IMPLEMENTATION
#include <twi.h>

int main(void)
{
    printf_init();
    sei();

    twi_master_init();
    printf("Twi Master initialized...\n");

    uint8_t data[6] = {0};
    twi_master_read_from(0x10, data, sizeof(data));
    printf("Read data from slave...\n");

    for (uint8_t i = 0; i < sizeof(data); ++i) {
        printf("%c", (char)data[i]);
    }

    return 0;
}
