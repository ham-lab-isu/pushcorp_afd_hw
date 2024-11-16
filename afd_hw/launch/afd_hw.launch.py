from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare the simulation argument
    declare_simulation_arg = DeclareLaunchArgument(
        'simulation',
        default_value='true',
        description='Set to "true" to run in simulation mode, "false" for real device mode'
    )

    # Path to the configuration file
    config_path = os.path.join(
        get_package_share_directory('afd_hw'),
        'config',
        'afd_hw_config.yaml'
    )

    # ros2_control_node with the configuration
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='ros2_control_node',
        output='screen',
        parameters=[config_path, {'simulation': LaunchConfiguration('simulation')}]
    )

    return LaunchDescription([
        declare_simulation_arg,
        ros2_control_node
    ])
