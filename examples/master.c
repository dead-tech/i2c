#include <stdint.h>
#include <stdio.h>

#define I2C_IMPLEMENTATION
#include <i2c.h>
#include <uart.h>

#define SLAVE_ADDRESS 0x10
#define REGISTER_ADDRESS 0x1

int main(void)
{
    printf_init();
    int ret = 0;

    I2C i2c;
    i2c_init(&i2c, SLAVE_ADDRESS);

    // Send data
    const uint8_t payload = 0x69;
    ret = i2c_master_write_to_slave_register(&i2c, REGISTER_ADDRESS, payload);
    if (ret != I2C_ERROR_SUCCESS)
    {
        printf("[ERROR] Failed to write to slave = { address = %d, register = %d, payload = 0x%02x } because: %s\n", SLAVE_ADDRESS, REGISTER_ADDRESS, payload, i2c_error_str(ret));
        return 1;
    }
    printf("Sent: 0x%02x\n", payload);

    // Read data
    uint8_t received;
    ret = i2c_master_read_from_slave_register(&i2c, REGISTER_ADDRESS, &received);
    if (ret == I2C_ERROR_SUCCESS) {
        printf("Received: 0x%02x\n", received);
    } else {
        printf("[ERROR] Failed to read from slave = { address = %d, register = %d } because: %s\n", SLAVE_ADDRESS, REGISTER_ADDRESS, i2c_error_str(ret));
        return 1;
    }

    return 0;
}
