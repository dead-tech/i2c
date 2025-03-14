#ifndef I2C_H
#define I2C_H

#define I2C_IMPLEMENTATION

#include <avr/io.h>
#include <stdint.h>
#include <stdbool.h>

#define VERIFY(cond) \
    do { \
        if (!cond) return cond; \
    } while (0)

// TODO: Implement clock stretching
// Allow slave to adjust clock speed if the master one is too fast
// by waiting for some time for the scl line to go high when requesting it
// as high.

// We are going to assume for now that you will use PD0 (SCL) & PD1 (SDA)
// This is because we don't have at the moment a simple way to converting
// arbitrary ports to their physical mapping on hardware like pinMode in
// arduino 'standard library' does.
typedef struct
{
    uint8_t scl_mask;
    uint8_t sda_mask;
    uint8_t slave_address;
} I2C;


void i2c_init(I2C* i2c, const uint8_t slave_address);
void i2c_scl_high(I2C* i2c);
void i2c_scl_low(I2C* i2c);
void i2c_sda_low(I2C* i2c);

void i2c_start_condition(I2C* i2c);
void i2c_write_byte(I2C* i2c, const uint8_t byte);
uint8_t i2c_read_byte(I2C* i2c);
bool i2c_check_ack(I2C* i2c);
void i2c_send_nack(I2C* i2c);
void i2c_stop_condition(I2C* i2c);

void i2c_write_slave_address(I2C* i2c, bool read_or_write);
void i2c_write_register_address(I2C* i2c, const uint8_t register_address);
bool i2c_write_to_slave_register(I2C* i2c, const uint8_t register_address, const uint8_t payload);
bool i2c_read_from_slave_register(I2C* i2c, const uint8_t register_address, uint8_t* result);

#ifdef I2C_IMPLEMENTATION

void i2c_init(I2C* i2c, const uint8_t slave_address)
{
    i2c->scl_mask = 1 << 0;
    i2c->sda_mask = 1 << 1;
    i2c->slave_address = slave_address;

    // Set SCL & SDA to INPUT PULLUP & LOW
    // Basically the PORTD is always going to stay low
    // but we are going to change 'state' by changing
    // the data direction register.
    DDRD  &= ~(i2c->scl_mask | i2c->sda_mask);
    PORTD &= ~(i2c->scl_mask | i2c->sda_mask);
}

void i2c_scl_high(I2C* i2c)
{
    DDRD &= ~(i2c->scl_mask);
}

void i2c_scl_low(I2C* i2c)
{
    DDRD |= i2c->scl_mask;
}

void i2c_sda_high(I2C* i2c)
{
    DDRD &= ~(i2c->sda_mask);
}

void i2c_sda_low(I2C* i2c)
{
    DDRD |= i2c->sda_mask;
}

// From: https://en.wikipedia.org/wiki/I%C2%B2C
// Assumes SDA is HIGH
// Data transfer is initiated with a start condition (S) signalled by SDA being pulled low while SCL stays high.
void i2c_start_condition(I2C* i2c)
{
    i2c_scl_high(i2c);
    i2c_sda_low(i2c);
    i2c_scl_low(i2c);
}

void i2c_write_byte(I2C* i2c, const uint8_t byte)
{
    // i2c_start_signal has set scl to low & sda to low
    // we write bits msb order
    // we check if the bit is zero => sda high else sda low
    // send the info with scl high then lower scl again
    for (uint8_t bit = 0x80; bit != 0 ; bit >>= 1) {
        if (byte & bit) {
            i2c_sda_high(i2c);
        } else {
            i2c_sda_low(i2c);
        }

        i2c_scl_high(i2c);
        i2c_scl_low(i2c);
    }

    // Release data line
    i2c_sda_high(i2c);
}

uint8_t i2c_read_byte(I2C* i2c)
{
    // Reading from MSB onwards
    // We just control the clock line
    uint8_t ret = 0;
    for (uint8_t bit = 0x80; bit != 0; bit >>= 1) {
        i2c_scl_high(i2c);
        if (PIND & i2c->sda_mask) {
            ret += bit;
        }
        i2c_scl_low(i2c);
    }

    return ret;
}

bool i2c_check_ack(I2C* i2c)
{
    bool ack = false;

    // Release data line so the receiver can transmit ACK
    i2c_sda_high(i2c);

    i2c_scl_high(i2c);
    ack = (PIND & i2c->sda_mask) != 0;
    i2c_scl_low(i2c);

    return ack;
}

void i2c_send_nack(I2C* i2c)
{
    i2c_sda_low(i2c);
    i2c_scl_high(i2c);
    i2c_scl_low(i2c);

    // Release data line
    i2c_sda_high(i2c);
}

void i2c_stop_condition(I2C* i2c)
{
    i2c_sda_low(i2c);
    i2c_scl_high(i2c);
    i2c_sda_high(i2c);
}

// Write = 0, Read = 1
void i2c_write_slave_address(I2C* i2c, bool read_or_write)
{
    for (uint8_t bit = 0x40; bit != 0; bit >>= 1) {
        if (i2c->slave_address & bit) {
            i2c_sda_high(i2c);
        } else {
            i2c_sda_low(i2c);
        }

        i2c_scl_high(i2c);
        i2c_scl_low(i2c);
    }

    // Read or write bit
    if (read_or_write) {
        i2c_sda_high(i2c);
    } else {
        i2c_sda_high(i2c);
    }

    i2c_scl_high(i2c);
    i2c_scl_low(i2c);

    // Release SDA line for ACK
    i2c_sda_high(i2c);
}

void i2c_write_register_address(I2C* i2c, const uint8_t register_address)
{
    i2c_write_byte(i2c, register_address);
}

bool i2c_write_to_slave_register(I2C* i2c, const uint8_t register_address, const uint8_t payload)
{
    i2c_start_condition(i2c);

    i2c_write_slave_address(i2c, 0);
    VERIFY(i2c_check_ack(i2c));

    i2c_write_register_address(i2c, register_address);
    VERIFY(i2c_check_ack(i2c));

    i2c_write_byte(i2c, payload);
    VERIFY(i2c_check_ack(i2c));

    i2c_stop_condition(i2c);
    return true;
}

bool i2c_read_from_slave_register(I2C* i2c, const uint8_t register_address, uint8_t* result)
{
    i2c_start_condition(i2c);

    i2c_write_slave_address(i2c, 0);
    VERIFY(i2c_check_ack(i2c));

    i2c_write_register_address(i2c, register_address);
    VERIFY(i2c_check_ack(i2c));

    // Repeated start
    i2c_start_condition(i2c);
    i2c_write_slave_address(i2c, 1);
    VERIFY(i2c_check_ack(i2c));

    *result = i2c_read_byte(i2c);

    // NACK used to tell the slave we are done reading
    i2c_send_nack(i2c);
    i2c_stop_condition(i2c);
    return true;
}

#endif // I2C_IMPLEMENTATION

#endif // I2C_H
