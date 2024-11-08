cmake_minimum_required(VERSION 3.8)
project(afd_hw)

set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)  # Use Boost.Asio if asio package is unavailable

# Add the library
add_library(afd_hardware_interface SHARED src/afd_hardware_interface.cpp)
target_compile_features(afd_hardware_interface PUBLIC cxx_std_17)
target_link_libraries(afd_hardware_interface yaml-cpp asio)

# Register the library as a plugin
pluginlib_export_plugin_description_file(hardware_interface afd_hw_interface.xml)

# Install the library
install(TARGETS afd_hardware_interface
        DESTINATION lib/${PROJECT_NAME})

# Install the plugin description file
install(FILES afd_hardware_interface.xml
        DESTINATION share/${PROJECT_NAME})

ament_package()
