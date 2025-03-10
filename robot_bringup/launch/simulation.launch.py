import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    moveit_dir = os.path.join(get_package_share_directory("crx10ia_l_moveit_config"));

    # Move Group
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                moveit_dir,
                'launch/move_group.launch.py'))
    )
    # Robot State Publisher
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                moveit_dir,
                'launch/rsp.launch.py'))
    )
    # Static Virtual Joints
    static_virtual_joint_tfs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                moveit_dir,
                'launch/static_virtual_joint_tfs.launch.py'))
    )
    # RViz Node
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                moveit_dir,
                'launch/moveit_rviz.launch.py'))
    )    
    # Add ros2_control_node for simulation
    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[os.path.join(moveit_dir,"config","ros2_controllers.yaml",)],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
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
        rviz,
        move_group,
        robot_state_publisher,
        static_virtual_joint_tfs,
        ros2_control,
        joint_state_controller,
        arm_controller,
    ])