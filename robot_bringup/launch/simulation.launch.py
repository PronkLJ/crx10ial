import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():

    # Robot model
    xacro_file = os.path.join(get_package_share_directory('robot_description'), 'urdf', 'robot.xacro')
    # Process xacro 
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)

    # Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])]
        ),
        launch_arguments={"verbose": "false"}.items(),
    )

    # Robot State Publisher Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name="robot_state_publisher",
        output="screen",
        parameters=[{'robot_description': doc.toxml(),
                     #'use_sim_time': True,
                     }],
    )

    # Spawn Gazebo Model Node
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'cobot'],
        output='screen',
    )

    # Load the joint state controller
    load_joint_state_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    # Load the manipulator controller
    load_manipulator_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['manipulator_controller'],
        output='screen',
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        load_joint_state_controller,
        load_manipulator_controller,
    ])