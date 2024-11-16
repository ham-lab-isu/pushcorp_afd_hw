cmake_minimum_required(VERSION 3.8)
project(afd_hw)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add compiler options for better warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui Charts)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(std_msgs REQUIRED)

# Add the hardware interface library
add_library(afd_hardware_interface SHARED src/afd_hw.cpp)
target_compile_features(afd_hardware_interface PUBLIC cxx_std_17)
target_link_libraries(afd_hardware_interface Boost::system yaml-cpp)
ament_target_dependencies(afd_hardware_interface rclcpp hardware_interface pluginlib yaml-cpp)

# Register the hardware interface plugin with pluginlib
pluginlib_export_plugin_description_file(hardware_interface afd_hw_interface.xml)

# Check and add RViz plugin source
set(RVIZ_PLUGIN_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/rviz/afd_hw_panel.cpp")
if(EXISTS "${RVIZ_PLUGIN_SRC}")
  message(STATUS "Found RViz plugin source: ${RVIZ_PLUGIN_SRC}")

  # Add the RViz plugin library
  add_library(afd_hw_rviz_plugin SHARED ${RVIZ_PLUGIN_SRC})

  # Link against necessary Qt and RViz libraries
  target_link_libraries(afd_hw_rviz_plugin
    ${rviz_common_LIBRARIES}
    ${rviz_rendering_LIBRARIES}
    Qt5::Widgets
    Qt5::Core
    Qt5::Gui
    Qt5::Charts
  )

  # Specify include directories for RViz plugin
  target_include_directories(afd_hw_rviz_plugin PRIVATE
    ${rviz_common_INCLUDE_DIRS}
    ${rviz_rendering_INCLUDE_DIRS}
  )

  # Ensure dependencies for RViz plugin are correctly specified
  ament_target_dependencies(afd_hw_rviz_plugin rclcpp rviz_common rviz_rendering std_msgs)

  # Register the RViz plugin in pluginlib
  pluginlib_export_plugin_description_file(rviz_common src/rviz/plugins.xml)

  # Install the RViz plugin library
  install(TARGETS afd_hw_rviz_plugin
          DESTINATION lib/${PROJECT_NAME})

  # Install plugin description files
  install(FILES src/rviz/plugins.xml
          DESTINATION share/${PROJECT_NAME})
else()
  message(FATAL_ERROR "RViz plugin source file NOT found: ${RVIZ_PLUGIN_SRC}")
endif()

# Install the hardware interface library
install(TARGETS afd_hardware_interface
        DESTINATION lib/${PROJECT_NAME})

# Install plugin description files
install(FILES afd_hw_interface.xml
        DESTINATION share/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

# Install configuration files
install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})

# Install include directory (if needed)
install(DIRECTORY include/
        DESTINATION include/)

# Define the package
ament_package()
