#ifndef TWI_H
#define TWI_H

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdio.h>
#include <uart.h>
#include <util/twi.h>

#if defined(__AVR_ATmega328P__)
    #define ENABLE_TWI_PULLUPS()      \
        do {                          \
            DDRC &= ~((1 << PC4) | (1 << PC5));   /* SDA/SCL as input */   \
            PORTC |= (1 << PC4) | (1 << PC5);     /* Enable pull-ups */    \
        } while (0)

#elif defined(__AVR_ATmega2560__)
    #define ENABLE_TWI_PULLUPS()      \
        do {                          \
            DDRD &= ~((1 << PD1) | (1 << PD0));   /* SDA/SCL as input */   \
            PORTD |= (1 << PD1) | (1 << PD0);     /* Enable pull-ups */    \
        } while (0)

#else
    #warning "ENABLE_TWI_PULLUPS not defined for this MCU!"
    #define ENABLE_TWI_PULLUPS() do {} while (0)
#endif

#ifndef TWI_SCL_CLOCK
#define TWI_SCL_CLOCK 100000L
#endif // TWI_SCL_CLOCK

#define TWI_BUFFER_SIZE 32

typedef enum
{
    TwiReady = 0,
    TwiMasterTransmitter,
    TwiMasterReceiver,
    TwiSlaveReceiver,
    TwiSlaveTransmitter,
} TwiState;

// Slave address 7 bits + r/w bit
static volatile uint8_t  twi_slave_address_rw = 0;
static volatile TwiState twi_state            = TwiReady;

static volatile uint8_t twi_master_buffer[TWI_BUFFER_SIZE] = { 0 };
static volatile uint8_t twi_master_buffer_index            = 0;
static volatile uint8_t twi_master_buffer_size             = 0;

static volatile uint8_t twi_read_buffer[TWI_BUFFER_SIZE] = { 0 };
static volatile uint8_t twi_read_buffer_index            = 0;
static volatile uint8_t twi_read_buffer_size             = 0;

static volatile uint8_t twi_write_buffer[TWI_BUFFER_SIZE] = { 0 };
static volatile uint8_t twi_write_buffer_index            = 0;
static volatile uint8_t twi_write_buffer_size             = 0;

typedef void (*OnSlaveReceiveCallback)(volatile uint8_t *, volatile uint8_t);
static volatile OnSlaveReceiveCallback twi_on_slave_receive_callback = NULL;

typedef void (*OnSlaveTransmitCallback)(void);
static volatile OnSlaveTransmitCallback twi_on_slave_transmit_callback = NULL;

/// API

void twi_master_init(void);

void twi_slave_init(const uint8_t address);

void twi_set_slave_receive_callback(OnSlaveReceiveCallback callback);
void twi_set_slave_transmit_callback(OnSlaveTransmitCallback callback);

uint8_t twi_master_read_from(
  const uint8_t address,
  uint8_t      *data,
  const uint8_t size
);

int twi_available_bytes(void);

void twi_master_write_to(
  const uint8_t  address,
  const uint8_t *data,
  const uint8_t  size
);

void twi_slave_prepare_transmission(const uint8_t *data, const uint8_t size);

#ifdef TWI_IMPLEMENTATION

static void twi_send_start(void)
{
    TWCR =
      (1 << TWSTA) | (1 << TWINT) | (1 << TWEN) | (1 << TWIE) | (1 << TWEA);
}

static void twi_send_stop(void)
{
    TWCR =
      (1 << TWSTO) | (1 << TWINT) | (1 << TWEN) | (1 << TWIE) | (1 << TWEA);

    while (!(TWCR & (1 << TWSTO)));

    twi_state = TwiReady;
}

static void twi_send_ack(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE) | (1 << TWEA);
}

static void twi_send_nack(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE);
}

void twi_master_init(void)
{
    ENABLE_TWI_PULLUPS();

    TWSR &= ~(1 << TWPS0);
    TWSR &= ~(1 << TWPS1);
    TWBR = ((F_CPU / TWI_SCL_CLOCK) - 16) / 2;

    TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWEA) | (1 << TWINT);
}

void twi_slave_init(const uint8_t address)
{
    twi_master_init();
    TWAR = address << 1;
}

void twi_set_slave_receive_callback(OnSlaveReceiveCallback callback)
{
    twi_on_slave_receive_callback = callback;
}

void twi_set_slave_transmit_callback(OnSlaveTransmitCallback callback)
{
    twi_on_slave_transmit_callback = callback;
}

uint8_t
  twi_master_read_from(const uint8_t address, uint8_t *data, const uint8_t size)
{
    if (size > TWI_BUFFER_SIZE) { return 0; }

    while (twi_state != TwiReady);

    twi_master_buffer_index = 0;
    twi_master_buffer_size  = size - 1;

    twi_state            = TwiMasterReceiver;
    twi_slave_address_rw = TW_READ;
    twi_slave_address_rw |= address << 1;

    twi_send_start();

    while (twi_state == TwiMasterReceiver);

    const uint8_t read_bytes =
      twi_master_buffer_index < size ? twi_master_buffer_index : size;

    for (uint8_t i = 0; i < read_bytes; ++i) { data[i] = twi_master_buffer[i]; }

    return read_bytes;
}

int twi_available_bytes(void)
{
    return twi_read_buffer_size - twi_read_buffer_index;
}

void twi_master_write_to(
  const uint8_t  address,
  const uint8_t *data,
  const uint8_t  size
)
{
    if (size > TWI_BUFFER_SIZE) { return; }

    while (twi_state != TwiReady);

    twi_master_buffer_index = 0;
    twi_master_buffer_size  = size;

    twi_state            = TwiMasterTransmitter;
    twi_slave_address_rw = TW_WRITE;
    twi_slave_address_rw = address << 1;

    for (uint8_t i = 0; i < size; ++i) { twi_master_buffer[i] = data[i]; }

    twi_send_start();

    while (twi_state == TwiMasterTransmitter);
}

void twi_slave_prepare_transmission(const uint8_t *data, const uint8_t size)
{
    if (twi_master_buffer_size + size > TWI_BUFFER_SIZE) { return; }

    if (twi_state != TwiSlaveTransmitter) { return; }

    for (uint8_t i = 0; i < size; ++i) {
        twi_write_buffer[twi_master_buffer_size + i] = data[i];
    }

    twi_write_buffer_size += size;
}

ISR(TWI_vect)
{
    switch (TW_STATUS) {
        // General Master Interrupt Routines
        case TW_START:
        case TW_REP_START: {
            TWDR = twi_slave_address_rw;
            twi_send_ack();
            break;
        }

        // Master Transmitter Interrupt Routines
        case TW_MT_SLA_ACK:
        case TW_MT_DATA_ACK: {
            if (twi_master_buffer_index < twi_master_buffer_size) {
                TWDR = twi_master_buffer[twi_master_buffer_index++];
                twi_send_ack();
            } else {
                twi_send_stop();
            }

            break;
        }

        case TW_MT_SLA_NACK:
        case TW_MT_DATA_NACK:
        case TW_MT_ARB_LOST: {
            TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWEA) | (1 << TWINT);
            twi_send_stop();
            break;
        }

        // Master Receiver Interrupt Routines
        case TW_MR_SLA_ACK: {
            if (twi_master_buffer_index < twi_master_buffer_size) {
                twi_send_ack();
            } else {
                twi_send_nack();
            }

            break;
        }

        case TW_MR_DATA_ACK: {
            twi_master_buffer[twi_master_buffer_index++] = TWDR;
            if (twi_master_buffer_index < twi_master_buffer_size) {
                twi_send_ack();
            } else {
                twi_send_nack();
            }

            break;
        }

        case TW_MR_DATA_NACK: {
            twi_master_buffer[twi_master_buffer_index++] = TWDR;
            twi_send_stop();
            break;
        }

        case TW_MR_SLA_NACK: {
            twi_send_stop();
            break;
        }

        // Slave Receiver Interrupt Routines
        case TW_SR_SLA_ACK:
        case TW_SR_ARB_LOST_SLA_ACK:
        case TW_SR_GCALL_ACK:
        case TW_SR_ARB_LOST_GCALL_ACK: {
            twi_state             = TwiSlaveReceiver;
            twi_read_buffer_index = 0;
            twi_send_ack();
            break;
        }

        case TW_SR_DATA_ACK:
        case TW_SR_GCALL_DATA_ACK: {
            if (twi_read_buffer_index < TWI_BUFFER_SIZE) {
                twi_read_buffer[twi_read_buffer_index++] = TWDR;
                twi_send_ack();
            } else {
                twi_send_nack();
            }

            break;
        }

        case TW_SR_DATA_NACK:
        case TW_SR_GCALL_DATA_NACK: {
            twi_send_nack();
            break;
        }

        case TW_SR_STOP: {
            TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWEA) | (1 << TWINT);

            if (twi_read_buffer_index < TWI_BUFFER_SIZE) {
                twi_read_buffer[twi_read_buffer_index] = '\0';
            }

            twi_on_slave_receive_callback(
              twi_read_buffer, twi_read_buffer_index
            );
            twi_read_buffer_index = 0;

            break;
        }

        // Slave Transmitter Interrupt Routines
        case TW_ST_SLA_ACK:
        case TW_ST_ARB_LOST_SLA_ACK: {
            twi_state              = TwiSlaveTransmitter;
            twi_write_buffer_index = 0;

            // This will be set later by twi_slave_prepare_transmission
            twi_write_buffer_size = 0;

            twi_on_slave_transmit_callback();

            if (twi_write_buffer_size == 0) {
                twi_write_buffer_size = 1;
                twi_write_buffer[0]   = 0;
            }


            TWDR = twi_write_buffer[twi_write_buffer_index++];
            if (twi_write_buffer_index < twi_write_buffer_size) {
                twi_send_ack();
            } else {
                twi_send_nack();
            }

            break;
        }

        case TW_ST_DATA_ACK: {
            TWDR = twi_write_buffer[twi_write_buffer_index++];
            if (twi_write_buffer_index < twi_write_buffer_size) {
                twi_send_ack();
            } else {
                twi_send_nack();
            }
        }

        case TW_ST_DATA_NACK:
        case TW_ST_LAST_DATA: {
            twi_send_ack();
            twi_state = TwiReady;
            break;
        }

        default: {
            printf("[INFO] unhandled interrupt: 0x%02x\n", TW_STATUS);
            break;
        }
    }
}

#endif // TWI_IMPLEMENTATION

#endif // TWI_H
