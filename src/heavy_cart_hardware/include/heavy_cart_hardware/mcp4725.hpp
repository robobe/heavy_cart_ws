#pragma once
#include <cstdint>
#include <iostream>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <linux/types.h>
#include <sys/ioctl.h>
#include <thread>
#include <iostream>
#include <unistd.h>
#include <string>

#define DAC_BITS 12

class MCP4725
{
public:
    enum class WriteCMD : uint8_t
    {
        CMD_RAM_WRITE = 0x40,
        CMD_EEPROM_WRITE = 0x60
    };
    const int MAX_VALUE = (1 << DAC_BITS) - 1;

public:
    MCP4725() = delete;
    MCP4725(const MCP4725 &) = delete;
    MCP4725(uint8_t bus_id, uint8_t address);
    MCP4725(std::string device, uint8_t address);
    ~MCP4725();
    friend std::ostream &operator<<(std::ostream &, const MCP4725 &);
    int write_data(uint16_t data);
    int write_persist_data(uint16_t data);

private:
    int fd_;
    std::string device_;
    uint8_t address_;

private:
    void init_(std::string device, uint8_t address)
    {
        device_ = device;
        address_ = address;
        fd_ = open(device.c_str(), O_RDWR);
        if (fd_ < 0)
        {
            std::__throw_runtime_error("Failed to open I2C interface!");
        }

        // Specify the address of the I2C slave device
        address_ = address;
        if (ioctl(fd_, I2C_SLAVE, address_) < 0)
        {
            close(fd_);
            throw std::runtime_error("Failed to acquire bus access and/or talk to slave");
        }
    }

private:
    int write_(u_int16_t value, MCP4725::WriteCMD write_cmd)
    {
        if (value > MAX_VALUE)
        {
            throw std::invalid_argument("DAC value must be between 0 and 4095");
        }

        // Prepare data for fast mode: 2 bytes (12-bit value)
        uint8_t buffer[3];
        buffer[0] = static_cast<u_int8_t>(write_cmd);          // Command byte for normal mode
        buffer[1] = (value >> 4) & 0xFF; // Upper 8 bits (D11-D4)
        buffer[2] = (value & 0x0F) << 4; // Lower 4 bits (D3-D0) in upper nibble

        // Write the data to the I2C bus
        int c = write(fd_, buffer, 3);
        if (c != 3)
        {
            throw std::runtime_error("Failed to write to the I2C bus");
        }

        return c;
    }
};