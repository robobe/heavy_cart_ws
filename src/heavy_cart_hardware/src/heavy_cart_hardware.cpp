#include <rclcpp/rclcpp.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "heavy_cart_hardware/heavy_cart_hardware.hpp"
#include <vector>

namespace heavy_cart_arduino
{
    HeavyCartArduinoHardware::HeavyCartArduinoHardware(){

    }

    hardware_interface::CallbackReturn HeavyCartArduinoHardware::on_init(
        const hardware_interface::HardwareInfo &info)
    {

        // init serial and other inits
        cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
        cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
        RCLCPP_WARN(rclcpp::get_logger("HeavyCartArduinoHardware"), cfg_.left_wheel_name.c_str());
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn HeavyCartArduinoHardware::on_cleanup(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("HeavyCartArduinoHardware"), "Cleaning up ...please wait...");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn HeavyCartArduinoHardware::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("HeavyCartArduinoHardware"), "Configuring ...please wait...");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn HeavyCartArduinoHardware::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("HeavyCartArduinoHardware"), "Activating ...please wait...");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn HeavyCartArduinoHardware::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("HeavyCartArduinoHardware"), "Deactivating ...please wait...");
        RCLCPP_INFO(rclcpp::get_logger("HeavyCartArduinoHardware"), "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> HeavyCartArduinoHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "wheel_left_joint",
            hardware_interface::HW_IF_POSITION,
            &_left_joint.pos
        ));
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> HeavyCartArduinoHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            "wheel_left_joint",
            hardware_interface::HW_IF_POSITION,
            &_left_joint.cmd));
        return command_interfaces;
    }

    hardware_interface::return_type HeavyCartArduinoHardware::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
    {
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type HeavyCartArduinoHardware::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        return hardware_interface::return_type::OK;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  heavy_cart_arduino::HeavyCartArduinoHardware, hardware_interface::SystemInterface)