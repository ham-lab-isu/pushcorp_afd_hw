from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    declare_simulation_arg = DeclareLaunchArgument(
        'simulation',
        default_value='true',
        description='Set to "true" to run in simulation mode, "false" for real device mode'
    )

    afd_hardware_node = Node(
        package='afd_hw',
        executable='afd_hardware_interface',  # Make sure this matches the library name
        name='afd_hardware_interface',
        output='screen',
        parameters=[{
            'simulation': LaunchConfiguration('simulation'),
            'ip': '192.168.0.12',  # Replace with your device's IP
            'port': '1993'         # Replace with your device's port
        }]
    )

    return LaunchDescription([
        declare_simulation_arg,
        afd_hardware_node
    ])
