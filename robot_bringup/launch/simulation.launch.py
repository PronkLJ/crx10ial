import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    # Launch Gazebo Classic
    gazebo_classic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('robot_bringup'), 'launch', 'gazebo_classic.launch.py')]),
    )

    # Launch MoveIt Components for Path Planning
    
    # Planning Context
    moveit_config=(
        MoveItConfigsBuilder("robot")
        .robot_description(os.path.join(get_package_share_directory('robot_description'), 'urdf', 'robot.xacro'))
        .trajectory_execution(os.path.join(get_package_share_directory('robot_moveit_config'), 'config', 'moveit_controllers.yaml'))
        .robot_description_kinematics(os.path.join(get_package_share_directory('robot_moveit_config'), 'config', 'kinematics.yaml'))
        .joint_limits(os.path.join(get_package_share_directory('robot_moveit_config'), 'config', 'joint_limits.yaml'))
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )
    
    # Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"moveit_simple_controller_manager": 
             os.path.join(get_package_share_directory('robot_moveit_config'),'config','controllers.yaml')},
            {'use_sim_time': True},        
        ],
    )

    # RViz Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", os.path.join(get_package_share_directory('robot_moveit_config'), 'config', 'moveit.rviz')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True} 
        ],
    )

    # Static Transformer Node
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"]
    )

    return LaunchDescription([
        gazebo_classic_launch,
        rviz_node,
        static_tf,
        move_group_node,
    ])
