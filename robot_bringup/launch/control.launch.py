import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    moveit_dir = os.path.join(get_package_share_directory("crx10ia_l_moveit_config"));

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                moveit_dir,
                'launch/move_group.launch.py'))
    )
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                moveit_dir,
                'launch/rsp.launch.py'))
    )
    static_virtual_joint_tfs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                moveit_dir,
                'launch/static_virtual_joint_tfs.launch.py'))
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                moveit_dir,
                'launch/moveit_rviz.launch.py'))
    )    

    return LaunchDescription([
        move_group,
        robot_state_publisher,
        static_virtual_joint_tfs,
        rviz,
    ])
