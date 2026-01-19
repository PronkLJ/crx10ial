from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip", default_value=TextSubstitution(text="192.168.100.39")
    )
    robot_type_arg = DeclareLaunchArgument(
        "robot_type", default_value=TextSubstitution(text="crx10ia_l")
    )
    robot_port_arg = DeclareLaunchArgument(
        "robot_port", default_value=TextSubstitution(text="502")
    )

    interface_node = Node(
        package='fanuc_ros2_driver',
        executable='fanuc_interface_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            "robot_ip": LaunchConfiguration('robot_ip'),
            "robot_type": LaunchConfiguration('robot_type'),
            "robot_port": LaunchConfiguration('robot_port'),
        }]
    )

    return LaunchDescription([
        robot_ip_arg,
        robot_type_arg,
        robot_port_arg,
        interface_node,
    ])
