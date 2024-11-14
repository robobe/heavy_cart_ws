#include "heavy_cart_hardware/mcp4725.hpp"

MCP4725::MCP4725(uint8_t bus_id, uint8_t address)
{
    auto device = "/dev/i2c-" + std::to_string(bus_id);
    init_(device, address);
}

MCP4725::MCP4725(std::string device, uint8_t address)
{
    init_(device, address);
}

MCP4725::~MCP4725()
{
    if (fd_)
    {
        close(fd_);
    }
}

std::ostream &operator<<(std::ostream &os, const MCP4725 &obj)
{
    os << "MCP4725 connect" 
        << " to address: 0x" << std::hex << static_cast<int>(obj.address_) 
        << " on bus: " << obj.device_ << "\n";
    return os;
}

int MCP4725::write_data(u_int16_t value)
{
    return write_(value, MCP4725::WriteCMD::CMD_RAM_WRITE);
}

int MCP4725::write_persist_data(u_int16_t value)
{
    return write_(value, MCP4725::WriteCMD::CMD_EEPROM_WRITE);
}