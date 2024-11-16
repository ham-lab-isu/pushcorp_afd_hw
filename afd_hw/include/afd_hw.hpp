#ifndef AFD_HARDWARE_INTERFACE_HPP
#define AFD_HARDWARE_INTERFACE_HPP

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <yaml-cpp/yaml.h>
#include <string>
#include <memory>
#include <chrono>
#include <asio.hpp>

namespace afd_hardware
{
    class AFDHW : public hardware_interface::SystemInterface
    {
    public:
        // Lifecycle functions
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

        // Functions to define state and command interfaces
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        // Read and write functions for the hardware interface
        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
        hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    private:
        // Simulation mode
        bool simulation_{true};

        // Variables to store sensor data and command targets
        double force_{0.0};
        double position_{0.0};
        double command_force_{0.0};
        double command_position_{0.0};

        // TCP socket for real-time communication with AFD
        std::unique_ptr<asio::ip::tcp::socket> socket_;

        // Private helper functions for parsing and network handling
        double parseForce(const std::string & data);
        double parsePosition(const std::string & data);
        hardware_interface::CallbackReturn connect_to_afd();
    };

}  // namespace afd_hardware

#endif  // AFD_HARDWARE_INTERFACE_HPP
