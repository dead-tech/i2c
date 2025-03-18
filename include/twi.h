#ifndef TWI_H
#define TWI_H

#include <stdbool.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>

#define TWI_IMPLEMENTATION

#include <stdio.h>

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

/// Cheatsheet
/// TWSR    - Two Wire Status Register
/// TWPS<N> - Two Wire Prescaler N
/// TWCR    - Two Wire Control Register
/// TWBR    - Two Wire Bit Rate
/// TWEN    - Two Wire ENable
/// TWEA    - Two Wire Enable ACKs
/// TWIE    - Two Wire Interrupts Enable
/// TWAR    - Two Wire Address
/// TWDR    - Two Wire Data Register
/// TWWC    - Two Wire Write Collision

/// Async Interrupts
/// TW_START                  - Two Wire Master (All) Interrupt: Start Condition
/// TW_REP_START              - Two Wire Master (All) Interrupt: Repeated Start Condition
/// TW_MT_SLA_ACK             - Two Wire Master Transmitter Interrupt: ACK for Slave Address
/// TW_MT_DATA_ACK            - Two Wire Master Transmitter Interrupt: ACK for Data
/// TW_MT_SLA_NACK            - Two Wire Master Transmitter Interrupt: NACK for Slave Address
/// TW_MT_DATA_NACK           - Two Wire Master Transmitter Interrupt: NACK for Data
/// TW_MT_ARB_LOST            - Two Wire Master Transmitter Arbitration Lost
/// TW_MR_DATA_ACK            - Two Wire Master Receiver Interrupt: ACK for Data
/// TW_MR_SLA_ACK             - Two Wire Master Receiver Interrupt: ACK for Slave Address
/// TW_MR_DATA_NACK           - Two Wire Master Receiver Interrupt: NACK for Data
/// TW_MR_SLA_NACK            - Two Wire Master Receiver Interrupt: NACK for Slave Address
/// TW_ST_SLA_ACK             - Two Wire Slave Transmitter Interrupt: ACK for Slave Address
/// TW_ST_ARB_LOST_SLA_ACK    - Two Wire Slave Transmitter Arbitration Lost ACK for Slave Address
/// TW_ST_DATA_ACK            - Two Wire Slave Transmitter Interrupt: ACK for Data
/// TW_ST_DATA_NACK           - Two Wire Slave Transmitter Interrupt: NACK for Data
/// TW_ST_LAST_DATA           - Two Wire Slave Transmitter Interrupt: Last Data Indicator
/// TW_SR_SLA_ACK             - Two Wire Slave Receiver Interrupt: ACK for Slave address
/// TW_SR_GCALL_ACK           - Two Wire Slave Receiver Interrupt: Broadcast ACK
/// TW_SR_ARB_LOST_SLA_ACK    - Two Wire Slave Receiver Interrupt: Arbitration Lost ACK for Slave Address
/// TW_SR_ARB_LOST_GCALL_ACK  - Two Wire Slave Receiver Interrupt: Arbitration Lost ACK for Broadcast
/// TW_SR_DATA_ACK            - Two Wire Slave Receiver Interrupt: ACK for Data
/// TW_SR_GCALL_DATA_ACK      - Two Wire Slave Receiver Interrupt: ACK for Data in Broadcast
/// TW_SR_STOP                - Two Wire Slave Receiver Interrupt: Stop
/// TW_SR_DATA_NACK           - Two Wire Slave Receiver Interrupt: NACK for Data
/// TW_SR_GCALL_DATA_NACK     - Two Wire Slave Receiver Interrupt: NACK for Data in Broadcast
/// TW_NO_INFO                - Two Wire (General) Interrupt: No State Information Available
/// TW_BUS_ERROR              - Two Wire (General) Interrupt: Bus Error


/// Constants
#define TWI_FREQ 100000L

#define TWI_BUFFER_CAPACITY 32

#define USE_ACKS 1
#define NO_ACKS  0

typedef enum {
    TWI_STATE_READY = 0,
    TWI_STATE_MASTER_TRANSMITTER,
    TWI_STATE_MASTER_RECEIVER,
    TWI_STATE_SLAVE_TRANSMITTER,
    TWI_STATE_SLAVE_RECEIVER,
} TwiState;

/// General state
static volatile TwiState twi_state;
static volatile uint8_t  twi_sla_rw;
static volatile bool     twi_repeated_start = false;
static volatile bool     twi_stop = true;

static volatile unsigned long twi_timeout = 100000;

/// Master buffer stuff
static volatile uint8_t  twi_master_buffer[TWI_BUFFER_CAPACITY];
static volatile uint8_t  twi_master_buffer_count;
static volatile uint8_t  twi_master_buffer_size;

/// Transmitter buffer stuff
static volatile uint8_t twi_transmitter_buffer[TWI_BUFFER_CAPACITY];
static volatile uint8_t twi_transmitter_buffer_count;
static volatile uint8_t twi_transmitter_buffer_size;

/// Receiver buffer stuff
static volatile uint8_t twi_receive_buffer[TWI_BUFFER_CAPACITY];
static volatile uint8_t twi_receiver_buffer_count;

typedef void (*SlaveTransmitCallback)(void);
typedef void (*SlaveReceiveCallback)(volatile uint8_t*, const int);

/// Callbacks
SlaveTransmitCallback twi_slave_transmit_callback;
SlaveReceiveCallback twi_slave_receive_callback;

/// Low level API
void twi_enable(void);
void twi_disable(void);
void twi_set_address(const uint8_t address);
void twi_set_frequency(const uint32_t frequency);
uint8_t twi_read_from(const uint8_t slave_address, uint8_t* dest, const uint8_t size, const bool stop);
uint8_t twi_write_to(const uint8_t slave_address, const uint8_t* dest, const uint8_t size, const bool stop);
uint8_t twi_transmit(const uint8_t* data, const uint8_t size);
void twi_set_slave_transmit_callback(SlaveTransmitCallback callback);
void twi_set_slave_receive_callback(SlaveReceiveCallback callback);
void twi_reply(bool use_acks);
void twi_send_stop(void);
void twi_release_bus(void);

#ifdef TWI_IMPLEMENTATION

static void handle_timeout() {
    const uint8_t previous_TWBR = TWBR;
    const uint8_t previous_TWAR = TWAR;

    // reset the interface
    twi_disable();
    twi_enable();

    // reapply the previous register values
    TWAR = previous_TWAR;
    TWBR = previous_TWBR;
}

/// Low level API
void twi_enable(void)
{
    twi_state = TWI_STATE_READY;

    // Enable pull-up resistors
    DDR_GENERAL  &= ~(SCL_MASK | SDA_MASK);
    PORT_GENERAL |= (SCL_MASK | SDA_MASK);

    // Setup prescalers
    // The prescalers are used to reduce the frequency of the clock
    // to what the user desires.
    TWSR &= ~(1 << TWPS0);
    TWSR &= ~(1 << TWPS1);

    // Setup bitrate
    // From the atmega manual:
    // SCL Frequency = CPU Clock Frequency / (16 + (2 * TWBR))
    TWBR = ((F_CPU / TWI_FREQ) - 16) / 2;

    // Enable twi itself, twi acks, twi interuppts
    TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWIE);
}

void twi_disable(void)
{
    // Disable twi itself, twi acks, twi interrupts
    TWCR &= ~((1 << TWEN) | (1 << TWEA) | (1 << TWIE));

    // Disable pull-ups
    PORT_GENERAL |= 0;
}

void twi_set_address(const uint8_t address)
{
    // With shl by 1 we are ignoring the TWGCE bit
    // which is used to broadcast to all slaves
    TWAR = address << 1;
}

void twi_set_frequency(const uint32_t frequency)
{
    // From the atmega manual: SCL Frequency = CPU Clock Frequency / (16 + (2 * TWBR))
    TWBR = ((F_CPU / frequency) - 16) / 2;
}

uint8_t twi_read_from(const uint8_t slave_address, uint8_t* dest, const uint8_t size, const bool stop)
{
    // Check that we do have enough space
    if (size > TWI_BUFFER_CAPACITY) {
        return 0;
    }

    // Wait for the state of the bus to be ready
    twi_timeout = 100000;
    while (twi_state != TWI_STATE_READY) {
        if (twi_timeout == 0) handle_timeout();
        --twi_timeout;
    }

    twi_stop = stop;

    // Switch to master receiving state
    twi_state = TWI_STATE_MASTER_RECEIVER;

    // Reset master buffer count and capacity
    // size - 1 because of ACKs/NACKs
    twi_master_buffer_count = 0;
    twi_master_buffer_size  = size - 1;

    // Set slave address and read bit
    twi_sla_rw = TW_READ;
    twi_sla_rw |= slave_address << 1;

    // We have already sent the start condition and we are waiting for the address once again
    if (twi_repeated_start) {
        twi_repeated_start = false;
        do {
            TWDR = twi_sla_rw;
        } while (TWCR & (1 << TWWC));

        // "TWINT flag must be cleared by software before starting any operation on TWI"
        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA) | (1 << TWIE);
    } else {
        // otherwise send the start condition
        TWCR = (1 << TWSTA) | (1 << TWINT) | (1 << TWEN) | (1 << TWEA) | (1 << TWIE);
    }

    // Wait until the state change takes place
    twi_timeout = 100000;
    while (twi_state == TWI_STATE_MASTER_RECEIVER) {
        if (twi_timeout == 0) {
            handle_timeout();
        }
        --twi_timeout;
    }

    // Consider the case where fewer bytes than the ones requested
    // are actually read.
    uint8_t read_bytes = size;
    if (twi_master_buffer_count < size) {
        read_bytes = twi_master_buffer_count;
    }

    // Copy the data into the buffer
    for (uint8_t i = 0; i < size; ++i) {
        dest[i] = twi_master_buffer[i];
    }

    return read_bytes;
}

uint8_t twi_write_to(const uint8_t slave_address, const uint8_t* dest, const uint8_t size, const bool stop)
{
    // Check that we do have enough space
    if (size > TWI_BUFFER_CAPACITY) {
        return 0;
    }

    // Wait for the state of the bus to be ready
    twi_timeout = 100000;
    while (twi_state != TWI_STATE_READY) {
        if (twi_timeout == 0) handle_timeout();
        --twi_timeout;
    }

    twi_stop = stop;

    // Switch to master transmitter state
    twi_state = TWI_STATE_MASTER_TRANSMITTER;

    // Reset master buffer count and size
    twi_master_buffer_count = 0;
    twi_master_buffer_size = size;

    // Copy data into the buffer
    for (uint8_t i = 0; i < size; ++i) {
        twi_master_buffer[i] = dest[i];
    }

    // Set slave address and write bit
    twi_sla_rw = TW_WRITE;
    twi_sla_rw |= slave_address << 1;

    // We have already sent the start condition and we are waiting for the address once again
    if (twi_repeated_start) {
        twi_repeated_start = false;
        do {
            TWDR = twi_sla_rw;
        } while (TWCR & (1 << TWWC));

        // "TWINT flag must be cleared by software before starting any operation on TWI"
        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA) | (1 << TWIE);
    } else {
        // otherwise send the start condition
        TWCR = (1 << TWSTA) | (1 << TWINT) | (1 << TWEN) | (1 << TWEA) | (1 << TWIE);
    }

    // Wait until the state change takes place aka we are done writing
    twi_timeout = 100000;
    while (twi_state == TWI_STATE_MASTER_TRANSMITTER) {
        if (twi_timeout == 0) handle_timeout();
        --twi_timeout;
    }

    return 0;
}

uint8_t twi_transmit(const uint8_t* data, const uint8_t size)
{
    // Check that we do have enough space
    if (twi_transmitter_buffer_size + size > TWI_BUFFER_CAPACITY) {
        return 1;
    }

    // Check that we are in the correct state
    if (twi_state != TWI_STATE_SLAVE_TRANSMITTER) {
        return 2;
    }

    // Copy the data into the buffer
    for (uint8_t i = 0; i < size; ++i) {
        twi_transmitter_buffer[size + i]  = data[i];
    }

    // Update the size
    twi_transmitter_buffer_size += size;
    return 0;
}

void twi_set_slave_transmit_callback(SlaveTransmitCallback callback)
{
    twi_slave_transmit_callback = callback;
}

void twi_set_slave_receive_callback(SlaveReceiveCallback callback)
{
    twi_slave_receive_callback = callback;
}

void twi_reply(bool use_acks)
{
    // Enable or disable acks
    if (use_acks) {
        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA) | (1 << TWIE);
    } else {
        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE);
    }
}

void twi_send_stop(void)
{
    // Send the stop condition
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA) | (1 << TWIE) | (1 << TWSTO);

    twi_timeout = 100000;
    while (TWCR & (1 << TWSTO)) {
        if (twi_timeout == 0) handle_timeout();
        --twi_timeout;
    }

    twi_state = TWI_STATE_READY;
}

void twi_release_bus(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA) | (1 << TWIE);
    twi_state = TWI_STATE_READY;
}

ISR(TWI_vect)
{
    switch (TW_STATUS) {
        // Transmitter & Receiver Master
        // Start condition was sent | repeated start condition was sent
        case TW_START:
        case TW_REP_START: {
            // Send the slave address and ack
            TWDR = twi_sla_rw;
            twi_reply(USE_ACKS);
            break;
        }

        // Transmitter Master
        case TW_MT_SLA_ACK:
        case TW_MT_DATA_ACK: {
            // There is something to transmit
            if (twi_master_buffer_count < twi_master_buffer_size) {
                TWDR = twi_master_buffer[twi_master_buffer_count++];
                twi_reply(USE_ACKS);
            } else {
                if (twi_stop) {
                    twi_send_stop();
                } else {
                    twi_repeated_start = true;
                    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
                    twi_state = TWI_STATE_READY;
                }
            }
            break;
        }

        // Transmitter Master Errors
        case TW_MT_SLA_NACK:
        case TW_MT_DATA_NACK: {
            twi_send_stop();
            break;
        }
        case TW_MT_ARB_LOST: {
            twi_release_bus();
            break;
        }

        // Master Receiver
        case TW_MR_DATA_ACK: {
            twi_master_buffer[twi_master_buffer_count++] = TWDR;
            // Fallthrough!
        }
        case TW_MR_SLA_ACK: {
            if (twi_master_buffer_count < twi_master_buffer_size) {
                twi_reply(USE_ACKS);
            } else {
                twi_reply(NO_ACKS);
            }
            break;
        }
        case TW_MR_DATA_NACK: {
            twi_master_buffer[twi_master_buffer_count++] = TWDR;
            if (twi_stop) {
                twi_send_stop();
            } else {
                twi_repeated_start = true;
                TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
                twi_state = TWI_STATE_READY;
            }
            break;
        }
        case TW_MR_SLA_NACK: {
            twi_send_stop();
            break;
        }

        // Slave transmitter
        case TW_ST_SLA_ACK:
        case TW_ST_ARB_LOST_SLA_ACK: {
            twi_state = TWI_STATE_SLAVE_TRANSMITTER;
            twi_transmitter_buffer_count = 0;
            twi_transmitter_buffer_size  = 0;
            twi_slave_transmit_callback();
            if (twi_transmitter_buffer_size == 0) {
                twi_transmitter_buffer_size = 1;
                twi_transmitter_buffer[0] = 0;
            }
            // Fallthrough! Send the first byte in the buffer
            break;
        }
        case TW_ST_DATA_ACK: {
            TWDR = twi_transmitter_buffer[twi_transmitter_buffer_count++];
            if (twi_transmitter_buffer_count < twi_transmitter_buffer_size) {
                twi_reply(USE_ACKS);
            } else {
                twi_reply(NO_ACKS);
            }
            break;
        }
        case TW_ST_DATA_NACK:
        case TW_ST_LAST_DATA: {
            twi_reply(USE_ACKS);
            twi_state = TWI_STATE_READY;
            break;
        }

        // Slave Receiver
        case TW_SR_SLA_ACK:
        case TW_SR_GCALL_ACK:
        case TW_SR_ARB_LOST_SLA_ACK:
        case TW_SR_ARB_LOST_GCALL_ACK: {
            twi_state = TWI_STATE_SLAVE_RECEIVER;
            twi_receiver_buffer_count = 0;
            twi_reply(USE_ACKS);
            break;
        }

        case TW_SR_DATA_ACK:
        case TW_SR_GCALL_DATA_ACK: {
            if (twi_receiver_buffer_count < TWI_BUFFER_CAPACITY) {
                twi_receive_buffer[twi_receiver_buffer_count++] = TWDR;
                twi_reply(USE_ACKS);
            } else {
                twi_reply(NO_ACKS);
            }
            break;
        }

        case TW_SR_STOP: {
            twi_release_bus();

            if (twi_receiver_buffer_count < TWI_BUFFER_CAPACITY) {
                twi_receive_buffer[twi_receiver_buffer_count] = '\0';
            }

            twi_slave_receive_callback(twi_receive_buffer, twi_receiver_buffer_count);
            twi_receiver_buffer_count = 0;
            break;
        }

        case TW_SR_DATA_NACK:
        case TW_SR_GCALL_DATA_NACK: {
            twi_reply(NO_ACKS);
            break;
        }

        // All
        case TW_NO_INFO: {
            break;
        }
        case TW_BUS_ERROR: {
            twi_send_stop();
            break;
        }

    }
}

#endif // TWI_IMPLEMENTATION

#endif // TWI_H
