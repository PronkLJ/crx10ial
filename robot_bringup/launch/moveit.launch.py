import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    # Argument to load the MoveIt controllers in robot.xacro
    moveit_only_arg = DeclareLaunchArgument(
        'moveit_only',
        default_value='true',
        description='If true, only MoveIt is launched'
    )
    moveit_only = LaunchConfiguration('moveit_only')

    # Planning Context
    moveit_config=(
        MoveItConfigsBuilder("robot")
        .robot_description(
            os.path.join(get_package_share_directory('robot_description'), 'urdf', 'robot.xacro'),
            {"moveit_only": moveit_only}               
        )
        .robot_description_semantic(os.path.join(get_package_share_directory("robot_moveit_config"), "config", "crx10ia_l.srdf"))
        .trajectory_execution(os.path.join(get_package_share_directory('robot_moveit_config'), 'config', 'moveit_controllers.yaml'))
        .robot_description_kinematics(os.path.join(get_package_share_directory('robot_moveit_config'), 'config', 'kinematics.yaml'))
        .joint_limits(os.path.join(get_package_share_directory('robot_moveit_config'), 'config', 'joint_limits.yaml'))
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

    # RViz Node
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", os.path.join(get_package_share_directory('robot_moveit_config'), 'config', 'moveit.rviz')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
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

    # ROS2 Controller Node
    ros2_controllers_path = os.path.join(get_package_share_directory("robot_moveit_config"), "config", "ros2_controllers.yaml")
    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
    )

    # Joint State Controllers
    joint_state_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # Manipulator Controller
    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )

    return LaunchDescription([
        moveit_only_arg,
        move_group,
        robot_state_publisher,
        static_tf,
        rviz,
        ros2_control,
        joint_state_controller,
        arm_controller,
    ])