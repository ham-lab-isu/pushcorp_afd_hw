#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <yaml-cpp/yaml.h>
#include <string>
#include <memory>
#include <chrono>
#include <boost/asio.hpp>

namespace afd_hardware
{
    // Initialize force and posisiton values as well as command force and positon targets
    double force = 0.0;
    double position = 0.0;
    double command_force = 0.0;
    double command_position = 0.0;

    // Simulation Flag
    bool simulation = true;

    class AFDHW : public hardware_interface::SystemInterface
        {
        public:
            // Method to Initialize hardware interface and afd connection
            hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
            {
                // Initialize simulation parameter
                simulation = info.hardware_parameters["simulation"] == "true";

                // If not in simulation, connect to afd
                if (!simulation)
                {
                    try
                    {
                    boost::asio::io_service io_service;
                    boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address::from_string(info.hardware_parameters["ip"]), std::stoi(info.hardware_parameters["port"]));
                    socket_ = std::make_unique<boost::asio::ip::tcp::socket>(io_service);
                    socket_->connect(endpoint);
                    }
                    catch(const std::exception& e)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("AFDHardwareInterface"), "Failed to connect to AFD device: %s", e.what()); 
                        return hardware_interface::CallbackReturn::ERROR;
                    }
                }

                return hardware_interface::CallbackReturn::SUCCESS;
            }

            // Method to make state interfaces for ros2control
            std::vector<hardware_interface::StateInterface> export_state_interfaces() override
            {
                std::vector<hardware_interface::StateInterface> state_interfaces;

                // Create state interfaces for force and position
                state_interfaces.emplace_back(hardware_interface::StateInterface("afd", "force", &force));
                state_interfaces.emplace_back(hardware_interface::StateInterface("afd", "position", &position));

                return state_interfaces;
            }

            // Method for exporting commands through ros2control
            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
            {
                std::vector<hardware_interface::CommandInterface> command_interfaces;

                // Create command interfaces for force and position control
                command_interfaces.emplace_back(hardware_interface::CommandInterface("afd", "command_force", &command_force));
                command_interfaces.emplace_back(hardware_interface::CommandInterface("afd", "command_position", &command_position));

                return command_interfaces;
            }

            hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
            {
                if (simulation)
                {
                    // Simulate force and position data for testing
                    force = 10.0;  // Example static value, could be a sine wave or other simulated pattern
                    position = 5.0;
                }
                else
                {
                    // Read real data from the afd device
                    try
                    {
                        std::vector<char> buffer(128);
                        size_t len = socket_->read_some(boost::asio::buffer(buffer));
                        std::string data(buffer.begin(), buffer.begin() + len);

                        // Parse data to update force and position values (implement parseForce and parsePosition functions)
                        force = parseForce(data);
                        position = parsePosition(data);

                        // Uncomment for debugging
                        //RCLCPP_DEBUG(rclcpp::get_logger("AFDHardwareInterface"), "Force: %f, Position: %f", force, position);
                    }
                    catch (const std::exception & e)
                    {
                        // Uncomment for error report
                        // RCLCPP_ERROR(rclcpp::get_logger("AFDHardwareInterface"), "Error reading from AFD device: %s", e.what());
                        return hardware_interface::return_type::ERROR;
                    }
                }

                return hardware_interface::return_type::OK;
            }

            hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
            {
                if (!simulation)
                {
                    // Send force and position commands to the AFD device
                    try
                    {
                        // Assume commands are formatted as "force:<value>,position:<value>"
                        std::string command = "force:" + std::to_string(command_force) + ",position:" + std::to_string(command_position);
                        boost::asio::write(*socket_, boost::asio::buffer(command));

                        // Uncomment for reponse
                        // RCLCPP_DEBUG(rclcpp::get_logger("AFDHardwareInterface"), "Sent command - %s", command.c_str());
                    }
                    catch (const std::exception & e)
                    {
                        // Uncomment for error report
                        //RCLCPP_ERROR(rclcpp::get_logger("AFDHardwareInterface"), "Error writing to AFD device: %s", e.what());
                        return hardware_interface::return_type::ERROR;
                    }
                }

                return hardware_interface::return_type::OK;
            }

        private:
            std::unique_ptr<boost::asio::ip::tcp::socket> socket_;

            double parseForce(const std::string & data)
            {
                // Implement data parsing logic to extract force from the string
                return 0.0;  // Placeholder
            }     

            double parsePosition(const std::string & data)
            {
                // Implement data parsing logic to extract position from the string
                return 0.0;  // Placeholder
            }
        };

}  // namespace afd_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(afd_hardware::AFDHW, hardware_interface::SystemInterface)


