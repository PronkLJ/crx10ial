import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

world = 'empty_world'

# WORK IN PROGRESS

def generate_launch_description():

    # Value to ensure loading of correct controllers
    gazebo_classic = 'false'
    
    # Robot model
    xacro_file = os.path.join(get_package_share_directory('robot_description'), 'urdf', 'robot.xacro')
    # Process xacro
    doc = xacro.process_file(xacro_file, mappings={'gazebo_classic': gazebo_classic})

    ## Gazebo world 
    world_file = os.path.join(get_package_share_directory('robot_description'), 'world', world+'.sdf')

    # Start Gazebo 
    gazebo_launch_file = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch_file]),
        launch_arguments=[('gz_args', [' -r -v 0 ' + world_file])],
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name="robot_state_publisher",
        parameters=[{'robot_description': doc.toxml()}],
        #emulate_tty=True
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
        arguments=['-name', 'cobot', '-topic', 'robot_description', '-z', '1.0'],
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
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU'
            # Joint states (IGN -> ROS2)
            '/world/empty/model/cobot/joint_state@sensor_msgs/msg/JointState]gz.msgs.Model',
        ],
        remappings=[
            ('/world/empty/model/cobot/joint_state', 'joint_states'),
        ],
        output='screen',
    )

    ## Launch description
    return LaunchDescription([
        # Launch
        gazebo,

        # Nodes
        robot_state_publisher,
        static_tf,
        spawn_sim_robot,
        joint_state_controller,
        manipulator_controller,
        gz_bridge_node,
    ])