#include <sensors/i2c.hpp>
#include "pico/binary_info.h"
#include "pico/stdlib.h"

i2c::device::device(i2c_inst_t * p_bus) {
    bus = p_bus;
}

i2c::device::device(i2c_inst_t * p_bus, int _addr) {
    bus = p_bus;
    addr = _addr;
}

void i2c::device::close() {
}

uint8_t i2c::device::read_byte(uint8_t reg) {
    uint8_t buf;
    i2c_write_blocking(bus, addr, &reg, 1, true);
    i2c_read_blocking(bus, addr, &buf, 1, false);
    return buf;
}

void i2c::device::read_burst(uint8_t reg, uint8_t * buf, int len) {
    i2c_write_blocking(bus, addr, &reg, 1, true);
    i2c_read_blocking(bus, addr, buf, len, false);
}

void i2c::device::write_byte(uint8_t reg, uint8_t val) {
    uint8_t write_buffer[2] = { reg, val };
    i2c_write_blocking(bus, addr, write_buffer, 2, false);
}