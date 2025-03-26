import os, yaml
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def load_yaml(package_name, file_name):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_name)
    with open(absolute_file_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)

def generate_launch_description():

    # Planning Context
    moveit_config=(
        MoveItConfigsBuilder("robot")
        .trajectory_execution(os.path.join(get_package_share_directory("robot_moveit_config"), 'config', 'moveit_controllers.yaml'))
        .to_moveit_configs()
    )

    # Robot Description Content - MoveIt
    robot_description_content_moveit = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([
            os.path.join(get_package_share_directory("robot_description"), "urdf", "robot.moveit.xacro"),
        ]),
    ])
    robot_description_moveit = {
        "robot_description": robot_description_content_moveit}

    # Robot Description Semantic
    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([
            os.path.join(get_package_share_directory("robot_moveit_config"), "config", "crx10ia_l.srdf")
        ]),
    ])
    robot_description_semantic = {
        "robot_description_semantic": 
        ParameterValue(robot_description_semantic_content, value_type=str)    
    }
    
    # Robot Description Kinematics
    robot_description_kinematics = {
        "robot_description_kinematics":
        load_yaml("robot_moveit_config", os.path.join("config", "kinematics.yaml"))
    }

    # Joint Limits
    joint_limits = {
        "joint_limits":
        load_yaml("robot_moveit_config", os.path.join("config", "joint_limits.yaml"))
    }

    # Planning Scene Monitor
    planning_scene_monitor = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
    }

    # Move Group Node
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description_moveit,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits,
            planning_scene_monitor,
            moveit_config.planning_pipelines,
            moveit_config.trajectory_execution,
            moveit_config.moveit_cpp,
            moveit_config.pilz_cartesian_limits,
        ]
    )

    # Robot State Publisher Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description_moveit],
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
        arguments=["-d", os.path.join(get_package_share_directory("robot_moveit_config"), 'config', 'moveit.rviz')],
        parameters=[
            robot_description_moveit,
            robot_description_semantic,
            robot_description_kinematics,
        ],
    )


    return LaunchDescription([
        move_group,
        robot_state_publisher,
        static_tf,
        rviz,
    ])