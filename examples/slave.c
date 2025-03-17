#include <stdio.h>

#define AVR_AT_MEGA_328P
#define I2C_IMPLEMENTATION
#include <i2c.h>
#include <uart.h>

#define SLAVE_ADDRESS 0x10
#define REGISTER_ADDRESS 0x1

uint8_t register_map[256] = {0};

int main(void)
{
    printf_init();
    int ret = 0;

    I2C i2c;
    i2c_init(&i2c, SLAVE_ADDRESS);
    printf("[INFO] I2C Slave Ready!\n");

    for (;;) {

        uint8_t register_address = 0;
        uint8_t payload = 0;
        const OpRequest request = i2c_slave_listen(&i2c, &register_address, &payload);

        if (request == OP_REQUEST_READ) {
            printf("[INFO] master asked to read { .slave_address, = %d, .register_address = %d, .data = %d }\n", payload, register_address, register_map[register_address]);
            ret = i2c_slave_send(&i2c, register_map, register_address, payload);
            if (ret != I2C_ERROR_SUCCESS) {
                printf("[ERROR] Failed to send data to master: %s\n", i2c_error_str(ret));
            }
        } else if (request == OP_REQUEST_WRITE) {
            ret = i2c_slave_receive(&i2c, register_map, register_address, payload);
            printf("[INFO] master asked to write { .slave_address = %d, .register_address = %d, .data = %d }\n", SLAVE_ADDRESS, register_address, payload);
            if (ret != I2C_ERROR_SUCCESS) {
                printf("[ERROR] Failed to receive data from master: %s\n", i2c_error_str(ret));
            }
        }
    }

    return 0;
}
