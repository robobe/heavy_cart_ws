#ifndef __HEAVY_CART_HARDWARE__
#define __HEAVY_CART_HARDWARE__

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"

namespace heavy_cart_arduino
{
    struct Config
    {
        std::string left_wheel_name = "";
        std::string right_wheel_name = "";
    };

    struct ServoJoint
    {
        std::string name = "";
        // std::unique_ptr<AngularServo> servo;
        double pos = 0;
        double cmd = 0;
    };

    class HeavyCartArduinoHardware : public hardware_interface::SystemInterface
    {
        // on_configure;
        // on_cleanup;
        // on_activate;
        // on_deactivate;
        // on_shutdown;
        // on_error;
    public:
        HeavyCartArduinoHardware();

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
        ServoJoint _left_joint;
        ServoJoint _right_joint;
        Config cfg_;
    };
}
#endif