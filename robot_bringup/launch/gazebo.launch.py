import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    
    # Robot model
    xacro_file = os.path.join(get_package_share_directory('robot_description'), 'urdf', 'robot.xacro')
    # Process xacro
    doc = xacro.process_file(xacro_file)

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name="robot_state_publisher",
        parameters=[{'robot_description': doc.toxml()}],
    )

    # Joint State Controller
    joint_state_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    # Manipulator Controller
    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen',
    )

    # Bridge topics from Gazebo to ROS2
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            # Joint states (IGN -> ROS2)
            '/world/empty/model/cobot/joint_state@sensor_msgs/msg/JointState]gz.msgs.Model',
        ],
        remappings=[
            ('/world/empty/model/cobot/joint_state', 'joint_states'),
        ],
        output='screen',
    )

    # Gazebo nodes
    world = os.path.join(get_package_share_directory("robot_description"),
                         "world", "empty_world.sdf")

    # Start Gazebo 
    gazebo_launch_file = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch_file]),
        launch_arguments=[('gz_args', [' -r -v 0 ' + world])],
    )
    # Spawn Gazebo model
    spawn_sim_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'cobot', '-topic', 'robot_description', '-z', '0.0'],
        output='screen',
    )    

    return LaunchDescription([
        gazebo,
        spawn_sim_robot,
        gz_bridge,
        robot_state_publisher,
        joint_state_controller,
        arm_controller,
    ])