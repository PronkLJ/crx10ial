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
from srdfdom.srdf import SRDF

# NOT FUNCTIONING

def generate_launch_description():
    
    # Robot model
    xacro_file = os.path.join(get_package_share_directory("robot_description"), "urdf", "robot.xacro")
    
    # Configurations
    ## Initialize MoveItConfigsBuilder to load robot description
    moveit_config = MoveItConfigsBuilder(robot_name="crx10ial", package_name="robot_moveit_config").to_moveit_configs()
    ## Rviz configuration file for MoveIt
    rviz_config_file = os.path.join(get_package_share_directory("robot_moveit_config"), "config", "moveit.rviz")

    # Launch arguments
    db = DeclareBooleanLaunchArg("db", default_value=False, description="By default, we do not start a database (it can be large)")
    debug = DeclareBooleanLaunchArg("debug", default_value=False, description="By default, we are not in debug mode")

    # Nodes
    ## Control Node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            os.path.join(get_package_share_directory("robot_moveit_config"), "config", "controller_params.yaml")
        ],
    )
    ## Robot State Publisher Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name="robot_state_publisher",
        respawn=True,
        output="screen",
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', xacro_file, ' sim:=True']), value_type=str)
            
        }],
        emulate_tty=True
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

    ## If database loading is enabled, start the database
    warehouse_db = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("robot_moveit_config"), "launch", "warehouse_db.launch.py")),
            condition=IfCondition(LaunchConfiguration("db")),
    )

    ## Include the controller spawner launch file
    spawn_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("robot_moveit_config"), "launch", "spawn_controllers.launch.py"))
    )


            
    return LaunchDescription([

        # Add launch arguments
        db,
        debug,
        
        # Nodes
        control_node,
        robot_state_publisher,
        rviz_node,

        # Launch
        virtual_joints,
        move_group,
        warehouse_db,
        spawn_controllers,
    ])