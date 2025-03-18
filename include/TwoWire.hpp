#ifndef TWO_WIRE_HPP
#define TWO_WIRE_HPP

#include <inttypes.h>

#define TWI_IMPLEMENTATION
extern "C" {
#include <twi.h>
}

class TwoWire final
{
public:
    using RequestCallback = void (*)(void);
    using ReceiveCallback = void (*)(const int);

public:
    TwoWire() = default;

    auto enable() const -> void;
    auto enable_as_slave(const uint8_t address) const -> void;

    auto on_request(RequestCallback callback) const -> void;
    auto on_receive(ReceiveCallback callback) const -> void;

    auto begin_transmission(const uint8_t address) const -> void;
    auto end_transmission() const -> uint8_t;

    auto request_from(const uint8_t address, const uint8_t size) const -> uint8_t;
    auto write(const uint8_t byte) const -> size_t;
    auto write(const uint8_t* data, const size_t size) const -> size_t;

    auto eos() const -> bool;
    auto read() const -> uint8_t;

    auto disable() const -> void;

private:
    static auto transmit_dispatcher() -> void;
    static auto receive_dispatcher(volatile uint8_t* data, const int size) -> void;

private:
    static uint8_t receive_buffer[TWI_BUFFER_CAPACITY];
    static uint8_t receive_iterator;
    static uint8_t receive_size;

    static uint8_t transmit_buffer[TWI_BUFFER_CAPACITY];
    static uint8_t transmit_iterator;
    static uint8_t transmit_size;

    static uint8_t transmit_address;
    static bool transmitting;

    static RequestCallback request_callback;
    static ReceiveCallback receive_callback;
};

uint8_t TwoWire::receive_buffer[TWI_BUFFER_CAPACITY];
uint8_t TwoWire::receive_iterator;
uint8_t TwoWire::receive_size;

uint8_t TwoWire::transmit_buffer[TWI_BUFFER_CAPACITY];
uint8_t TwoWire::transmit_iterator;
uint8_t TwoWire::transmit_size;

uint8_t TwoWire::transmit_address;
bool TwoWire::transmitting;

TwoWire::RequestCallback TwoWire::request_callback;
TwoWire::ReceiveCallback TwoWire::receive_callback;

extern TwoWire two_wire;

inline auto TwoWire::enable() const -> void
{
    receive_iterator = 0;
    receive_size = 0;

    transmit_iterator = 0;
    transmit_size = 0;

    twi_enable();
    twi_set_slave_transmit_callback(transmit_dispatcher);
    twi_set_slave_receive_callback(receive_dispatcher);
}

inline auto TwoWire::enable_as_slave(const uint8_t address) const -> void
{
    enable();
    twi_set_address(address);
}

inline auto TwoWire::on_request(RequestCallback callback) const -> void
{
    request_callback = callback;
}

inline auto TwoWire::on_receive(ReceiveCallback callback) const -> void
{
    receive_callback = callback;
}

inline auto TwoWire::begin_transmission(const uint8_t address) const -> void
{
    transmitting = true;
    transmit_address = address;
    transmit_iterator = 0;
    transmit_size = 0;
}

inline auto TwoWire::end_transmission() const -> uint8_t
{
    const auto ret = twi_write_to(transmit_address, transmit_buffer, transmit_size, true);

    transmit_iterator = 0;
    transmit_size = 0;
    transmitting = false;
    return ret;
}

inline auto TwoWire::request_from(const uint8_t address, const uint8_t size) const -> uint8_t
{
    // Clamp size to buffer capacity
    size_t clamped_size = size;
    if (size > TWI_BUFFER_CAPACITY) {
        clamped_size = TWI_BUFFER_CAPACITY;
    }

    const uint8_t read_bytes = twi_read_from(address, receive_buffer, clamped_size, true);

    receive_iterator = 0;
    receive_size = read_bytes;
    return read_bytes;
}

inline auto TwoWire::write(const uint8_t byte) const -> size_t
{
    if (transmitting) {
        // We are in master transmitter mode
        // Check for enough space
        if (transmit_size >= TWI_BUFFER_CAPACITY) {
            return 0;
        }

        transmit_buffer[transmit_iterator++] = byte;
        transmit_size = transmit_iterator;
    } else {
        // We are slave transmitter
        twi_transmit(&byte, 1);
    }

    return 1;
}

inline auto TwoWire::write(const uint8_t* data, const size_t size) const -> size_t
{
    if (transmitting) {
        // Master transmitter
        for (size_t i = 0; i < size; ++i) {
            write(data[i]);
        }
    } else {
        // Slave transmitter
        twi_transmit(data, size);
    }

    return size;
}

inline auto TwoWire::eos() const -> bool
{
    return !((receive_size - receive_iterator) > 0);
}

inline auto TwoWire::read() const -> uint8_t
{
    return receive_buffer[receive_iterator++];
}

inline auto TwoWire::disable() const -> void
{
    twi_disable();
}

inline auto TwoWire::transmit_dispatcher() -> void
{
    if (!request_callback) {
        return;
    }

    transmit_iterator = 0;
    transmit_size = 0;
    request_callback();
}

inline auto TwoWire::receive_dispatcher(volatile uint8_t* data, const int size) -> void
{
    if (!receive_callback) {
        return;
    }

    // Not finished reading
    if (receive_iterator < receive_size) {
        return;
    }

    for (uint8_t i = 0; i < size; ++i) {
        receive_buffer[i] = data[i];
    }

    receive_iterator = 0;
    receive_size = size;
    receive_callback(size);
}

// Preinstantiate
TwoWire two_wire = TwoWire();

#endif // TWO_WIRE_HPP
