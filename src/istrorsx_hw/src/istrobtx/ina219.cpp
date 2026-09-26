#include "istrorsx_hw/istrobtx/ina219.h"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <cstdio>

#include "logger.h"

LOG_DEFINE(loggerIna219, "Ina219");

// Config Register (R/W)
static const uint8_t REG_CONFIG        = 0x00;
// SHUNT VOLTAGE REGISTER (R)
static const uint8_t REG_SHUNTVOLTAGE  = 0x01;
// BUS VOLTAGE REGISTER (R)
static const uint8_t REG_BUSVOLTAGE    = 0x02;
// POWER REGISTER (R)
static const uint8_t REG_POWER         = 0x03;
// CURRENT REGISTER (R)
static const uint8_t REG_CURRENT       = 0x04;
// CALIBRATION REGISTER (R/W)
static const uint8_t REG_CALIBRATION   = 0x05;

// Config register bit fields -- see ina219.py's BusVoltageRange/Gain/
// ADCResolution/Mode classes.
static const int BUS_VOLTAGE_RANGE_16V   = 0x00;
static const int GAIN_DIV_2_80MV         = 0x01;
static const int ADCRES_12BIT_32S        = 0x0D;
static const int MODE_SANDBVOLT_CONTINUOUS = 0x07;

int Ina219::readWord(uint8_t reg, uint16_t &value)
{
    i2c_smbus_data data;
    data.block[0] = 2;  // I2C_SMBUS_I2C_BLOCK_DATA: caller specifies the length to read

    i2c_smbus_ioctl_data args;
    args.read_write = I2C_SMBUS_READ;
    args.command = reg;
    args.size = I2C_SMBUS_I2C_BLOCK_DATA;
    args.data = &data;

    if (ioctl(fd_, I2C_SMBUS, &args) < 0) {
        LOGM_ERROR(loggerIna219, "readWord", "msg=\"ioctl(I2C_SMBUS, READ) failed!\", reg=" << (int)reg);
        return -1;
    }

    // Big-endian on the wire, matches ina219.py's read(): (data[0]<<8)+data[1].
    value = ((uint16_t)data.block[1] << 8) | (uint16_t)data.block[2];
    return 0;
}

int Ina219::writeWord(uint8_t reg, uint16_t value)
{
    i2c_smbus_data data;
    data.block[0] = 2;
    data.block[1] = (value >> 8) & 0xFF;
    data.block[2] = value & 0xFF;

    i2c_smbus_ioctl_data args;
    args.read_write = I2C_SMBUS_WRITE;
    args.command = reg;
    args.size = I2C_SMBUS_I2C_BLOCK_DATA;
    args.data = &data;

    if (ioctl(fd_, I2C_SMBUS, &args) < 0) {
        LOGM_ERROR(loggerIna219, "writeWord", "msg=\"ioctl(I2C_SMBUS, WRITE) failed!\", reg=" << (int)reg);
        return -1;
    }

    return 0;
}

int Ina219::init(int i2c_bus, int i2c_addr)
{
    char devpath[32];
    snprintf(devpath, sizeof(devpath), "/dev/i2c-%d", i2c_bus);

    fd_ = open(devpath, O_RDWR);
    if (fd_ < 0) {
        LOGM_ERROR(loggerIna219, "init", "msg=\"open() failed!\", devpath=\"" << devpath << "\"");
        return -1;
    }

    if (ioctl(fd_, I2C_SLAVE, i2c_addr) < 0) {
        LOGM_ERROR(loggerIna219, "init", "msg=\"ioctl(I2C_SLAVE) failed!\", i2c_addr=" << i2c_addr);
        close();
        return -2;
    }

    // set_calibration_16V_5A() (ina219.py) -- assumes a 0.01ohm shunt, 16V/5A
    // range (16A before counter overflow).
    current_lsb_ = 0.1524;    // 100uA per bit
    power_lsb_ = 0.003048;    // 2mW per bit

    if (writeWord(REG_CALIBRATION, 26868) < 0) {
        close();
        return -3;
    }

    uint16_t config = (BUS_VOLTAGE_RANGE_16V << 13) | (GAIN_DIV_2_80MV << 11)
        | (ADCRES_12BIT_32S << 7) | (ADCRES_12BIT_32S << 3) | MODE_SANDBVOLT_CONTINUOUS;
    if (writeWord(REG_CONFIG, config) < 0) {
        close();
        return -4;
    }

    return 0;
}

void Ina219::close()
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

double Ina219::getShuntVoltage_mV()
{
    uint16_t raw;
    if (readWord(REG_SHUNTVOLTAGE, raw) < 0) {
        return 0;
    }
    int16_t value = (int16_t)raw;  // sign-extend, matches ina219.py's "if value > 32767: value -= 65536"
    return value * 0.01;
}

double Ina219::getBusVoltage_V()
{
    uint16_t raw;
    if (readWord(REG_BUSVOLTAGE, raw) < 0) {
        return 0;
    }
    return (raw >> 3) * 0.004;
}

double Ina219::getCurrent_mA()
{
    uint16_t raw;
    if (readWord(REG_CURRENT, raw) < 0) {
        return 0;
    }
    int16_t value = (int16_t)raw;
    return value * current_lsb_;
}

double Ina219::getPower_W()
{
    uint16_t raw;
    if (readWord(REG_POWER, raw) < 0) {
        return 0;
    }
    int16_t value = (int16_t)raw;
    return value * power_lsb_;
}
