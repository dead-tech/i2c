#ifndef I2C_H
#define I2C_H

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>

#include <stdio.h>

#define I2C_IMPLEMENTATION

#ifdef AVR_AT_MEGA_328P
    #define DDR_GENERAL DDRC
    #define PORT_GENERAL PORTC
    #define PIN_GENERAL PINC
    #define SCL_MASK (1 << 5)
    #define SDA_MASK (1 << 4)
#else
    #define DDR_GENERAL DDRD
    #define PORT_GENERAL PORTD
    #define PIN_GENERAL PIND
    #define SCL_MASK (1 << 0)
    #define SDA_MASK (1 << 1)
#endif

#define SIGNAL_DELAY_US 50

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

typedef enum
{
    OP_REQUEST_WRITE = 0,
    OP_REQUEST_READ  = 1,
    OP_REQUEST_NOT_FOR_ME,
} OpRequest;

typedef enum
{
    I2C_ERROR_WRITE_SLAVE_ADDRESS = 0,
    I2C_ERROR_WRITE_SLAVE_REGISTER,
    I2C_ERROR_WRITE_SLAVE_PAYLOAD,
    I2C_ERROR_SLAVE_SEND_PAYLOAD,

    I2C_ERROR_SUCCESS,
} I2C_Error;

static const char* error_lookup_table[I2C_ERROR_SUCCESS] = {
    [I2C_ERROR_WRITE_SLAVE_ADDRESS] = "master failed to write slave address",
    [I2C_ERROR_WRITE_SLAVE_REGISTER] = "master failed to write slave register",
    [I2C_ERROR_WRITE_SLAVE_PAYLOAD] = "master failed to write payload to slave",
    [I2C_ERROR_SLAVE_SEND_PAYLOAD] = "slave failed to write payload to master",
};

const char* i2c_error_str(int error)
{
    return error_lookup_table[error];
}

void i2c_init(I2C* i2c, const uint8_t slave_address);
void i2c_scl_high(I2C* i2c);
void i2c_scl_low(I2C* i2c);
void i2c_sda_low(I2C* i2c);

void i2c_master_start_condition(I2C* i2c);
void i2c_master_write_byte(I2C* i2c, const uint8_t byte);
uint8_t i2c_master_read_byte(I2C* i2c);
bool i2c_master_check_ack(I2C* i2c);
void i2c_master_send_ack(I2C* i2c);
void i2c_master_send_nack(I2C* i2c);
void i2c_master_stop_condition(I2C* i2c);

void i2c_master_write_slave_address(I2C* i2c, bool read_or_write);
void i2c_master_write_register_address(I2C* i2c, const uint8_t register_address);
int i2c_master_write_to_slave_register(I2C* i2c, const uint8_t register_address, const uint8_t payload);
int i2c_master_read_from_slave_register(I2C* i2c, const uint8_t register_address, uint8_t* result);

void i2c_slave_write_byte(I2C* i2c, const uint8_t byte);
uint8_t i2c_slave_read_byte(I2C* i2c);
void i2c_slave_send_ack(I2C* i2c);
OpRequest i2c_slave_listen(I2C* i2c);
int i2c_slave_receive(I2C* i2c, uint8_t* register_map);
int i2c_slave_send(I2C* i2c, uint8_t* register_map);

#ifdef I2C_IMPLEMENTATION

void i2c_init(I2C* i2c, const uint8_t slave_address)
{
    i2c->scl_mask = SCL_MASK;
    i2c->sda_mask = SDA_MASK;
    i2c->slave_address = slave_address;

    // Set SCL & SDA to INPUT PULLUP & LOW
    // Basically the PORT is always going to stay low
    // but we are going to change 'state' by changing
    // the data direction register.
    DDR_GENERAL  &= ~(i2c->scl_mask | i2c->sda_mask);
    PORT_GENERAL |= (i2c->scl_mask | i2c->sda_mask);
}

void i2c_scl_high(I2C* i2c)
{
    DDR_GENERAL &= ~(i2c->scl_mask);
    _delay_us(SIGNAL_DELAY_US);

    PORT_GENERAL |= i2c->scl_mask;
    _delay_us(SIGNAL_DELAY_US);
}

void i2c_scl_low(I2C* i2c)
{
    DDR_GENERAL |= i2c->scl_mask;
    _delay_us(SIGNAL_DELAY_US);

    PORT_GENERAL &= ~(i2c->scl_mask);
    _delay_us(SIGNAL_DELAY_US);
}

void i2c_sda_high(I2C* i2c)
{
    DDR_GENERAL &= ~(i2c->sda_mask);
    _delay_us(SIGNAL_DELAY_US);

    PORT_GENERAL |= i2c->sda_mask;
    _delay_us(SIGNAL_DELAY_US);
}

void i2c_sda_low(I2C* i2c)
{
    DDR_GENERAL |= i2c->sda_mask;
    _delay_us(SIGNAL_DELAY_US);

    PORT_GENERAL &= ~(i2c->sda_mask);
    _delay_us(SIGNAL_DELAY_US);
}

// From: https://en.wikipedia.org/wiki/I%C2%B2C
// Assumes SDA is HIGH
// Data transfer is initiated with a start condition (S) signalled by SDA being pulled low while SCL stays high.
void i2c_master_start_condition(I2C* i2c)
{
    i2c_scl_high(i2c);
    i2c_sda_low(i2c);
    i2c_scl_low(i2c);
}

void i2c_master_write_byte(I2C* i2c, const uint8_t byte)
{
    // i2c_start_condition has set scl to low & sda to low
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

uint8_t i2c_master_read_byte(I2C* i2c)
{
    // Reading from MSB onwards
    // We just control the clock line
    uint8_t ret = 0;
    for (uint8_t bit = 0x80; bit != 0; bit >>= 1) {
        i2c_scl_high(i2c);
        if (PIN_GENERAL & i2c->sda_mask) {
            ret |= bit;
        }
        i2c_scl_low(i2c);
    }

    return ret;
}

bool i2c_master_check_ack(I2C* i2c)
{
    bool ack = false;

    // Release data line so the receiver can transmit ACK
    i2c_sda_high(i2c);

    i2c_scl_high(i2c);
    ack = (PIN_GENERAL & i2c->sda_mask) == 0;
    i2c_scl_low(i2c);

    return ack;
}

void i2c_master_send_ack(I2C* i2c)
{
    i2c_sda_low(i2c);
    i2c_scl_high(i2c);
    i2c_scl_low(i2c);

    // Release data line
    i2c_sda_high(i2c);
}

void i2c_master_send_nack(I2C* i2c)
{
    i2c_sda_high(i2c);
    i2c_scl_high(i2c);
    i2c_scl_low(i2c);
}

void i2c_master_stop_condition(I2C* i2c)
{
    i2c_sda_low(i2c);
    i2c_scl_high(i2c);
    i2c_sda_high(i2c);
}

// Write = 0, Read = 1
void i2c_master_write_slave_address(I2C* i2c, bool read_or_write)
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
        i2c_sda_low(i2c);
    }

    i2c_scl_high(i2c);
    i2c_scl_low(i2c);

    // Release SDA line for ACK
    i2c_sda_high(i2c);
}

void i2c_master_write_register_address(I2C* i2c, const uint8_t register_address)
{
    i2c_master_write_byte(i2c, register_address);
}

int i2c_master_write_to_slave_register(I2C* i2c, const uint8_t register_address, const uint8_t payload)
{
    i2c_master_start_condition(i2c);

    i2c_master_write_slave_address(i2c, 0);
    _delay_ms(SIGNAL_DELAY_US);
    if (!i2c_master_check_ack(i2c)) {
        i2c_master_stop_condition(i2c);
        return I2C_ERROR_WRITE_SLAVE_ADDRESS;
    }

    i2c_master_write_register_address(i2c, register_address);
    _delay_ms(SIGNAL_DELAY_US);
    if (!i2c_master_check_ack(i2c)) {
        i2c_master_stop_condition(i2c);
        return I2C_ERROR_WRITE_SLAVE_REGISTER;
    }

    i2c_master_write_byte(i2c, payload);
    if (!i2c_master_check_ack(i2c)) {
        i2c_master_stop_condition(i2c);
        return I2C_ERROR_WRITE_SLAVE_PAYLOAD;
    }

    i2c_master_stop_condition(i2c);
    return I2C_ERROR_SUCCESS;
}

int i2c_master_read_from_slave_register(I2C* i2c, const uint8_t register_address, uint8_t* result)
{
    i2c_master_start_condition(i2c);

    i2c_master_write_slave_address(i2c, 0);
    if (!i2c_master_check_ack(i2c)) {
        i2c_master_stop_condition(i2c);
        return I2C_ERROR_WRITE_SLAVE_ADDRESS;
    }

    i2c_master_write_register_address(i2c, register_address);
    if (!i2c_master_check_ack(i2c)) {
        i2c_master_stop_condition(i2c);
        return I2C_ERROR_WRITE_SLAVE_REGISTER;
    }
    printf("hello\n");

    // Repeated start
    i2c_master_start_condition(i2c);

    i2c_master_write_slave_address(i2c, 1);
    if (!i2c_master_check_ack(i2c)) {
        i2c_master_stop_condition(i2c);
        return I2C_ERROR_WRITE_SLAVE_ADDRESS;
    }

    *result = i2c_master_read_byte(i2c);

    // NACK used to tell the slave we are done reading
    i2c_master_send_nack(i2c);

    i2c_master_stop_condition(i2c);
    return I2C_ERROR_SUCCESS;
}

void i2c_slave_write_byte(I2C* i2c, const uint8_t byte)
{
    for (uint8_t bit = 0x80; bit != 0 ; bit >>= 1) {
        if (byte & bit) {
            i2c_sda_high(i2c);
        } else {
            i2c_sda_low(i2c);
        }

        while ((PIN_GENERAL & i2c->scl_mask) == 0);
        while ((PIN_GENERAL & i2c->scl_mask) != 0);
    }

    // Release data line
    i2c_sda_high(i2c);
}

uint8_t i2c_slave_read_byte(I2C* i2c)
{
    uint8_t ret = 0;
    for (uint8_t bit = 0x80; bit != 0; bit >>= 1) {
        while ((PIN_GENERAL & i2c->scl_mask) == 0);
        if (PIN_GENERAL & i2c->sda_mask) {
            ret |= bit;
        }
        while ((PIN_GENERAL & i2c->scl_mask) != 0);
    }

    return ret;
}

void i2c_slave_send_ack(I2C* i2c)
{
    i2c_sda_low(i2c);
    while ((PIN_GENERAL & i2c->scl_mask) == 0);
    while ((PIN_GENERAL & i2c->scl_mask) != 0);

    // Release data line
    i2c_sda_high(i2c);
}

bool i2c_slave_check_ack(I2C* i2c)
{
    bool ack = false;

    // Release data line so the receiver can transmit ACK
    i2c_sda_high(i2c);

    while ((PIN_GENERAL & i2c->scl_mask) == 0);
    ack = (PIN_GENERAL & i2c->sda_mask) != 0;
    while ((PIN_GENERAL & i2c->scl_mask) != 0);

    return ack;
}

OpRequest i2c_slave_listen(I2C* i2c)
{
    // Wait for start condition
    while ((PIN_GENERAL & i2c->sda_mask) == 0);
    while ((PIN_GENERAL & i2c->scl_mask) != 0);
    while ((PIN_GENERAL & i2c->sda_mask) != 0);

    const uint8_t received_address = i2c_slave_read_byte(i2c);

    printf("received address = 0x%02x == 0x%02x\n", received_address >> 1, i2c->slave_address);

    if (received_address >> 1 == i2c->slave_address) {
        printf("r/w: %d\n", received_address & 0x1);
        i2c_slave_send_ack(i2c);
        printf("sent ack\n");
        return (OpRequest) received_address & 0x1;
    }

    return OP_REQUEST_NOT_FOR_ME;
}

// This assumes slave address and ack have already been read
// by i2c_slave_listen()
int i2c_slave_receive(I2C* i2c, uint8_t* register_map)
{
    // 1. Read register address and send ACK
    const uint8_t register_address = i2c_slave_read_byte(i2c);

    printf("slave received: register address = 0x%02x\n", register_address);

    i2c_slave_send_ack(i2c);
    printf("sent ack in slave_receive\n");

    // 2. Read the byte, store it and send ACK
    uint8_t data = i2c_slave_read_byte(i2c);
    printf("slave received: payload = 0x%02x\n", data);

    register_map[register_address] = data;
    i2c_slave_send_ack(i2c);
    return I2C_ERROR_SUCCESS;
}

// This assumes slave address and ack have already been read
// by i2c_slave_listen()
int i2c_slave_send(I2C* i2c, uint8_t* register_map)
{
    // 1. Read register address and send ACK
    const uint8_t register_address = i2c_slave_read_byte(i2c);

    printf("slave sending: register address = 0x%02x\n", register_address);

    i2c_slave_send_ack(i2c);

    // 2. Read repeated start condition
    while ((PIN_GENERAL & i2c->scl_mask) == 0);
    while ((PIN_GENERAL & i2c->scl_mask) != 0);

    // 3. Read slave address once again
    // FIXME: Should also check that last bit is set one
    const uint8_t slave_address = i2c_slave_read_byte(i2c);
    if (slave_address >> 1 == i2c->slave_address) {
        i2c_slave_send_ack(i2c);

        printf("slave sending: payload = 0x%02x\n", register_map[register_address]);
        // 4. Write the requested register data
        i2c_slave_write_byte(i2c, register_map[register_address]);
        if (!i2c_slave_check_ack(i2c)) {
            return I2C_ERROR_SLAVE_SEND_PAYLOAD;
        }
    }

    return I2C_ERROR_SUCCESS;
}

#endif // I2C_IMPLEMENTATION

#endif // I2C_H
