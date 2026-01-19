import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory

def generate_launch_description():

    moveit_share_dir = get_package_share_directory("robot_moveit_config")

    move_group = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    moveit_share_dir,
                    'launch/move_group.launch.py'))
    )
    robot_state_publisher = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    moveit_share_dir,
                    'launch/rsp.launch.py'))
    )

    static_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                moveit_share_dir,
                'launch/static_virtual_joint_tfs.launch.py'))
    )
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                moveit_share_dir,
                'launch/moveit_rviz.launch.py'))
    )

    return LaunchDescription([
        move_group,
        robot_state_publisher,
        static_tf,
        rviz_launch
    ])