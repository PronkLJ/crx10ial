import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg
import xacro

# NOT FUNCTIONING

def generate_launch_description():
    
    # Robot model
    xacro_file = os.path.join(get_package_share_directory("robot_description"), "urdf", "robot.xacro")
    # Process xacro 
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)

    # Load the SRDF file
    srdf_file = os.path.join(get_package_share_directory('robot_moveit_config'), 'config', 'crx10ial.srdf')
    with open(srdf_file, 'r') as f:
        srdf_content = f.read()

    
    # Configurations
    ## Initialize MoveItConfigsBuilder to load robot description
    moveit_config = MoveItConfigsBuilder(robot_name="crx10ial", package_name="robot_moveit_config").to_moveit_configs()
    ## Rviz configuration file for MoveIt
    rviz_config_file = os.path.join(get_package_share_directory("robot_moveit_config"), "config", "moveit.rviz")

    # Nodes
    ## Control Node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            os.path.join(get_package_share_directory("robot_moveit_config"), "config", "ros2_controllers.yaml")
        ],
    )
    # Robot State Publisher Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name="robot_state_publisher",
        output="screen",
        parameters=[{'robot_description': doc.toxml(),
                     'robot_description_semantic': srdf_content,  # Ensure SRDF is passed here as well
                     'use_sim_time': True,
                     }],
    )

    ## RViz Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        respawn=False,
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.planning_pipelines, 
            moveit_config.robot_description_kinematics,
        ]
    )

    # Launch
    ## Virtual joints
    virtual_joints_launch = os.path.join(get_package_share_directory("robot_moveit_config"), "launch", "static_virtual_joint_tfs.launch.py")
    virtual_joints = None
    if os.path.exists(virtual_joints_launch):
        virtual_joints = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(virtual_joints_launch))
    )

    ## Move group
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("robot_moveit_config"), "launch", "move_group.launch.py"))
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

        # Nodes
        control_node,
        robot_state_publisher,
        rviz_node,

        # Launch
        virtual_joints,
        move_group,
        load_joint_state_controller,
        load_manipulator_controller,
    ])