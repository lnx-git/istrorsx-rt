#pragma once

#include <cstdint>

// INA219 power/battery monitor over I2C (Waveshare UPS Power Module (C)) --
// no legacy (istro_rt2025.cpp) equivalent, this hardware isn't part of the
// original robot. Register map/calibration constants/getters ported
// directly from Waveshare's own /home/istrobotics/UPS_Power_Module_C/
// ina219.py (set_calibration_16V_5A()) -- same 16V/5A calibration, same
// 0.01ohm shunt assumption, no behavior change from the reference. Uses
// plain I2C_SMBUS block-data ioctl calls (<linux/i2c-dev.h>, kernel UAPI --
// no libi2c-dev needed) to reproduce Python smbus's read_i2c_block_data()/
// write_i2c_block_data() exactly: 2 raw big-endian bytes per register, not
// the byte-order-swapping SMBus "word" ops.
class Ina219 {
public:
    // Opens /dev/i2c-<i2c_bus>, selects i2c_addr via ioctl(I2C_SLAVE), and
    // writes the same calibration/config registers set_calibration_16V_5A()
    // does. Returns 0 on success, <0 on error (open/ioctl/write failure).
    int init(int i2c_bus, int i2c_addr);
    void close();

    double getBusVoltage_V();
    double getShuntVoltage_mV();
    double getCurrent_mA();
    double getPower_W();

private:
    int fd_ = -1;
    double current_lsb_ = 0;
    double power_lsb_ = 0;

    // I2C_SMBUS_I2C_BLOCK_DATA transactions, 2 bytes, big-endian -- matches
    // ina219.py's read()/write() exactly (read_i2c_block_data()/
    // write_i2c_block_data(), not the plain SMBus word ops, which are
    // little-endian and would silently swap the byte order INA219 needs).
    int readWord(uint8_t reg, uint16_t &value);
    int writeWord(uint8_t reg, uint16_t value);
};
