import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():

    # Robot model
    xacro_file = os.path.join(get_package_share_directory('robot_description'), 'urdf', 'robot.xacro')

    # Start Gazebo
    gazebo_launch_file = os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch_file]),
    )

    # Process xacro 
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    # Robot State Publisher Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name="robot_state_publisher",
        output="screen",
        parameters=[params],
    )

    # Node to start controller_manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory("robot_moveit_config"), "config", "ros2_controllers.yaml")
        ],
    )

    # Load the Joint State Controller after a delay
    load_joint_state_controller = TimerAction(
        period=5.0,  # 5-second delay
        actions=[ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', 'joint_state_broadcaster'],
            output="screen",
        )]
    )

    # Load the Manipulator Controller after a delay
    load_manipulator_controller = TimerAction(
        period=7.0,  # Additional delay to ensure joint state controller loads first
        actions=[ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', 'manipulator_controller'],
            output="screen",
        )]
    )

    # Activate the Joint State Controller
    activate_joint_state_controller = TimerAction(
        period=9.0,  # Additional delay to ensure the controller is loaded
        actions=[ExecuteProcess(
            cmd=['ros2', 'control', 'switch_controllers', '--activate', 'joint_state_broadcaster'],
            output="screen",
        )]
    )

    # Activate the Manipulator Controller
    activate_manipulator_controller = TimerAction(
        period=11.0,  # Additional delay to ensure the previous step completes
        actions=[ExecuteProcess(
            cmd=['ros2', 'control', 'switch_controllers', '--activate', 'manipulator_controller'],
            output="screen",
        )]
    )

    # Spawn Gazebo Model Node
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'cobot'],
        output='screen',
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        controller_manager,
        spawn_entity,
        load_joint_state_controller,
        load_manipulator_controller,
        activate_joint_state_controller,
        activate_manipulator_controller,
    ])