import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    # MoveIt config location
    moveit_dir = os.path.join(get_package_share_directory("robot_moveit_config"))

    # Planning Context
    moveit_config=(
        MoveItConfigsBuilder("robot")
        .robot_description(os.path.join(get_package_share_directory('robot_description'), 'urdf', 'robot.moveit.xacro'))
        .robot_description_semantic(os.path.join(get_package_share_directory("robot_description"), "srdf", "crx10ia_l.srdf"))
        .trajectory_execution(os.path.join(moveit_dir, 'config', 'moveit_controllers.yaml'))
        .robot_description_kinematics(os.path.join(moveit_dir, 'config', 'kinematics.yaml'))
        .joint_limits(os.path.join(moveit_dir, 'config', 'joint_limits.yaml'))
        .planning_scene_monitor(
            publish_robot_description=True, 
            publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )
    
    # Move Group Node
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()]
    )

    # Robot State Publisher Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # Static Transform Node
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"]
    )

    # RViz Node
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", os.path.join(moveit_dir, 'config', 'moveit.rviz')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([
        move_group,
        robot_state_publisher,
        static_tf,
        rviz,
    ])
