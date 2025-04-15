#include <uart.h>
#include <stdio.h>

#define TWI_IMPLEMENTATION
#include <twi.h>

int main(void)
{
    printf_init();
    sei();

    printf("Initializing Twi Master...");
    twi_master_init();
    printf("OK\n");

    printf("Writing to slave...");
    twi_master_write_to(0x10, (uint8_t*)"Hello, world!", 13);
    printf("OK\n");

    return 0;
}
