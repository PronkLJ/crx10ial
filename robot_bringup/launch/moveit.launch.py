import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    # Planning context
    moveit_config=(
        MoveItConfigsBuilder("robot")
        .robot_description(os.path.join(get_package_share_directory('robot_description'), 'urdf', 'robot.xacro'))
        .trajectory_execution(os.path.join(get_package_share_directory('robot_control'), 'config', 'moveit_controllers.yaml'))
        .robot_description_kinematics(os.path.join(get_package_share_directory('robot_control'), 'config', 'kinematics.yaml'))
        .joint_limits(os.path.join(get_package_share_directory('robot_control'), 'config', 'joint_limits.yaml'))
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
        parameters=[moveit_config.to_dict()]
    )

    ## RViz Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", os.path.join(get_package_share_directory('robot_control'), 'config', 'moveit.rviz')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"]
    )

    # Robot State Publisher Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    ros2_controllers_path = os.path.join(get_package_share_directory("robot_control"), "config", "ros2_controllers.yaml")
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
    )

    joint_state_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    manipulator_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["manipulator_controller"],
        output="screen",
    )

    return LaunchDescription(
        [
            rviz_node,
            static_tf,
            robot_state_publisher,
            ros2_control_node,
            joint_state_controller,
            manipulator_controller,

            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_controller,
                    on_exit=[move_group_node]
                )
            )
        ]
    )