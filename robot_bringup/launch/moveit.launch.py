import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg

package_name = "robot_moveit_config"


def generate_launch_description():
    """
    
    Includes:
     * static_virtual_joint_tfs
     * robot_state_publisher
     * move_group
     * moveit_rviz
     * warehouse_db (optional)
     * ros2_control_node + controller spawners
    """

    # Initialize MoveItConfigsBuilder to load robot description
    moveit_config = MoveItConfigsBuilder(robot_name="crx10ial", package_name=package_name).to_moveit_configs()

    # Define paths using pathlib.Path for better path handling
    package_share_directory = get_package_share_directory(package_name)

    # Create LaunchDescription object
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(DeclareBooleanLaunchArg("db", default_value=False, description="By default, we do not start a database (it can be large)",))
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False, description="By default, we are not in debug mode",))
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))

    virtual_joints_launch = Path(package_share_directory) / 'launch' / 'static_virtual_joint_tfs.launch.py'
    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(virtual_joints_launch))))

    # Include the other necessary launch files
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(Path(package_share_directory) / 'launch' / 'rsp.launch.py'))))
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(Path(package_share_directory) / 'launch' / 'move_group.launch.py'))))

    # Run Rviz and load the default config if use_rviz is True
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(Path(package_share_directory) / 'launch' / 'moveit_rviz.launch.py')),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    # If database loading is enabled, start the database
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(Path(package_share_directory) / 'launch' / 'warehouse_db.launch.py')),
            condition=IfCondition(LaunchConfiguration("db")),
        )
    )

    # Fake joint driver
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                str(moveit_config.package_path / "config/ros2_controllers.yaml"),
            ],
        )
    )

    # Include the controller spawner launch file
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(Path(package_share_directory) / 'launch' / 'spawn_controllers.launch.py'))))

    return ld