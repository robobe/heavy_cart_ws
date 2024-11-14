#pragma once

#include <vector>
#include <memory>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "heavy_cart_hardware/mcp4725.hpp"


namespace heavy_cart
{
    struct Config
    {
        std::string i2c_device_name;
        uint8_t left_dac_i2c_address;
        uint8_t right_dac_i2c_address;

    };

    struct Motor{
        std::string name = "";
        std::unique_ptr<MCP4725> driver;
        double vel = 0;
        double cmd = 0;
    };

    class HeavyCartI2cDiffHardware : public hardware_interface::SystemInterface
    {
        public:
        RCLCPP_SHARED_PTR_DEFINITIONS(HeavyCartI2cDiffHardware)

        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;
    private:
        Config cfg_;
        std::vector<Motor> motors_;
        std::vector<double> hw_commands_;
        std::vector<double> hw_velocities_;
    };
}