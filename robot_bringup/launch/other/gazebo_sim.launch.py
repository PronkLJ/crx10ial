import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import xacro

# WORK IN PROGRESS

def generate_launch_description():
    
    # Robot model
    xacro_file = os.path.join(get_package_share_directory('crx_description'), 'urdf', 'crx10ia_l.urdf.xacro')

    # Process xacro
    doc = xacro.process_file(xacro_file)
    robot_description = {"robot_description": doc.toxml()}

    # Start Gazebo 
    gazebo_launch_file = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch_file]),
        launch_arguments=[('gz_args', [' -r -v4 0 empty.sdf '])],
    )

    # Robot State Publisher
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                os.path.join(get_package_share_directory("crx10ia_l_moveit_config")),
                'launch/rsp.launch.py'))
    )
    
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Spawn Gazebo model
    spawn_sim_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'crx10ia_l', '-topic', 'robot_description', '-z', '1.0'],
        output='screen',
    )

    # Joint State Controller
    joint_state_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    # Manipulator Controller
    manipulator_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['manipulator_controller'],
        output='screen',
    )

    # Bridge topics from Gazebo to ROS2
    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model'],
        output='screen',
    )
    ## Launch description
    return LaunchDescription([
        # Launch
        gazebo,

        # Nodes
        #robot_state_publisher,
        #static_tf,
        spawn_sim_robot,
        #joint_state_controller,
        #manipulator_controller,
        gz_bridge_node,
    ])