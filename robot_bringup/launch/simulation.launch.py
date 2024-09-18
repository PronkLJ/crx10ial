import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # NOT YET WORKING

    # Launch MoveIt
    launch_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('robot_bringup'), 'launch', 'moveit.launch.py')]),
    )

    # Launch Gazebo
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('robot_bringup'), 'launch', 'gazebo.launch.py')]),
    )
 
    return LaunchDescription([

        # Launch
        launch_moveit,
        launch_gazebo,
    ])