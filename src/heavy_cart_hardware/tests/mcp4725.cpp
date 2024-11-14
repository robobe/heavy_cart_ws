#include <iostream>
#include <heavy_cart_hardware/mcp4725.hpp>

int main(int argc, char *argv[])
{
    u_int16_t data = 2048;
    if (argc>1){
        data = (uint16_t)atoi(argv[1]);
        std::cout << "send data: " << data << std::endl;
    }
    try
    {
        MCP4725 mcp(16, 0x62);
        std::cout << "test mcp4735 " << mcp << std::endl;
        auto c = mcp.write_data(data);
        std::cout << "test mcp4735 data: " << c << std::endl;
    }
    catch( std::runtime_error &e)
    {
        std::cout << "error ---------" << e.what() << std::endl;
    }
    return 0;
}