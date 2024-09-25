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
                     'use_sim_time': True,
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
    
    # planning context
    moveit_config=(
        MoveItConfigsBuilder("robot")
        .robot_description(xacro_file)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )
    
    # Start the actual move group node
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

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        load_joint_state_controller,
        load_manipulator_controller,
        rviz_node,
        static_tf,
        move_group_node,
    ])
