import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("crx10ia_l_moveit_config"), 'launch', file))
        ) for file in [
            'move_group.launch.py',
            'rsp.launch.py',
            'static_virtual_joint_tfs.launch.py',
            'moveit_rviz.launch.py'
        ]
    ])
