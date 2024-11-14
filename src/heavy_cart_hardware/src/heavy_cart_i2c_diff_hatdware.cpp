#include "heavy_cart_hardware/heavy_cart_i2c_diff_hatdware.hpp"
#include <rclcpp/rclcpp.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <string>

namespace heavy_cart
{

    hardware_interface::CallbackReturn HeavyCartI2cDiffHardware::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (
            hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        hw_velocities_.resize(info_.joints.size(),
                              std::numeric_limits<double>::quiet_NaN());
        hw_commands_.resize(info_.joints.size(),
                            std::numeric_limits<double>::quiet_NaN());
        // motors_.resize(info_.joints.size());

        // init i2c settings
        cfg_.i2c_device_name = info_.hardware_parameters["i2c_device_name"];

        // init joints
        for (const hardware_interface::ComponentInfo &joint : info_.joints)
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger(""), "-----" << joint.name);
            RCLCPP_INFO_STREAM(rclcpp::get_logger(""), "-----" << joint.parameters.at("dac_i2c_address"));
            auto i2c_address = static_cast<uint8_t>(std::stoi(joint.parameters.at("dac_i2c_address"), nullptr, 16));
            
            motors_.push_back({
                joint.name,
                std::make_unique<MCP4725>(cfg_.i2c_device_name, i2c_address)
            });
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn HeavyCartI2cDiffHardware::on_cleanup(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("HeavyCartI2cDiffHardware"), "Cleaning up ...please wait...");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn HeavyCartI2cDiffHardware::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("HeavyCartI2cDiffHardware"), "Configuring ...please wait...");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn HeavyCartI2cDiffHardware::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("HeavyCartI2cDiffHardware"), "Activating ...please wait...");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn HeavyCartI2cDiffHardware::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("HeavyCartI2cDiffHardware"), "Deactivating ...please wait...");
        RCLCPP_INFO(rclcpp::get_logger("HeavyCartI2cDiffHardware"), "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> HeavyCartI2cDiffHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (auto i = 0u; i < motors_.size(); i++) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                motors_[i].name,
                hardware_interface::HW_IF_VELOCITY,
                &motors_[i].vel));
        }
 
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> HeavyCartI2cDiffHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (auto i = 0u; i < motors_.size(); i++) {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                motors_[i].name,
                hardware_interface::HW_IF_VELOCITY,
                &motors_[i].cmd));
        }

        

        return command_interfaces;
    }

    hardware_interface::return_type HeavyCartI2cDiffHardware::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
    {
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type HeavyCartI2cDiffHardware::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        for (auto i = 0u; i < motors_.size(); i++) {
            motors_[i].vel = motors_[i].cmd;
            RCLCPP_INFO(rclcpp::get_logger(""), std::to_string(motors_[i].cmd).c_str());
        }
        return hardware_interface::return_type::OK;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    heavy_cart::HeavyCartI2cDiffHardware, hardware_interface::SystemInterface)