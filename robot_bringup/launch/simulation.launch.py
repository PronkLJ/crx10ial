import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

robot = 'robot'
world = 'empty_world'

def generate_launch_description():

    # Robot model
    xacro_file = os.path.join(get_package_share_directory(robot+'_description'), 'urdf', 'robot.xacro')

    # Gazebo world
    world_file = os.path.join(get_package_share_directory(robot+'_description'), 'world', world+'.sdf')

    # Start Gazebo
    gazebo_launch_file = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch_file]),
        launch_arguments=[('gz_args', ['-v 0 ' + world_file])],
    )

    # Robot State Publisher Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name="robot_state_publisher",
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', xacro_file, ' sim:=True']), value_type=str)
        }],
        emulate_tty=True
    )

    # Spawn Gazebo Model Node
    spawn_sim_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'cobot', '-topic', 'robot_description', '-x', '0.0', '-y', '0.0', '-z', '0.0'],
        output='screen',
    )

    # Bridge topics from Gazebo to ROS2 Node
    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                   '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU'],
        output='screen',
    )

    return LaunchDescription([

        # Launch
        gazebo,

        # Nodes
        robot_state_publisher,
        spawn_sim_robot,
        gz_bridge_node,
    ])
    