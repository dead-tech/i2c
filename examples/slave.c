#include <stdio.h>

#define AVR_AT_MEGA_328P
#define I2C_IMPLEMENTATION
#include <i2c.h>
#include <uart.h>

#define SLAVE_ADDRESS 0x10
#define REGISTER_ADDRESS 0x1

// Emulating registers
uint8_t register_map[256] = {0};

int main(void)
{
    printf_init();
    int ret = 0;

    I2C i2c;
    i2c_init(&i2c, SLAVE_ADDRESS);

    printf("I2C Slave Ready!\n");
    for (;;) {
        printf("SDA: %d, SCL: %d\n", (PIN_GENERAL & i2c.sda_mask) != 0, (PIN_GENERAL & i2c.scl_mask) != 0);

        const OpRequest request = i2c_slave_listen(&i2c);

        if (request == OP_REQUEST_READ) {
            ret = i2c_slave_send(&i2c, register_map);
            if (ret != I2C_ERROR_SUCCESS) {
                printf("[ERROR] Failed to send data to master\n");
            }
        } else if (request == OP_REQUEST_WRITE) {
            ret = i2c_slave_receive(&i2c, register_map);
            if (ret != I2C_ERROR_SUCCESS) {
                printf("[ERROR] Failed to receive data from master\n");
            }
        }
    }

    return 0;
}
