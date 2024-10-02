import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    # Launch Gazebo Sim
    gazebo_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('robot_bringup'), 'launch', 'gazebo_sim.launch.py')]),
    )

    # Launch MoveIt Components for Path Planning
    
    ## Planning Context
    moveit_config=(
        MoveItConfigsBuilder("robot")
        .robot_description(os.path.join(get_package_share_directory('robot_description'), 'urdf', 'robot2.xacro'))
        .trajectory_execution(os.path.join(get_package_share_directory('robot_moveit_config'), 'config', 'moveit_controllers.yaml'))
        .robot_description_kinematics(os.path.join(get_package_share_directory('robot_moveit_config'), 'config', 'kinematics.yaml'))
        .joint_limits(os.path.join(get_package_share_directory('robot_moveit_config'), 'config', 'joint_limits.yaml'))
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )
    
    ## Move Group Node
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

    ## RViz Node
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

    return LaunchDescription([
        gazebo_sim_launch,
        rviz_node,
        move_group_node,
    ])
