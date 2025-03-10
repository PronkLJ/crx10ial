import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    ## Arguments
    sim_arg = DeclareLaunchArgument(name='sim', default_value='true', choices=['true', 'false'],
                                    description='Set to true to switch from hardware to simulation in the loop')

    launch_group = GroupAction([
        # Launch physical robot control
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('robot_bringup'), 'launch', 'control.launch.py')]),
            condition=UnlessCondition(LaunchConfiguration('sim')),
        ),
        # Or simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('robot_bringup'), 'launch', 'simulation.launch.py')]),
            condition=IfCondition(LaunchConfiguration('sim')),
        ),
    ])
    
    return LaunchDescription([
        sim_arg,
        launch_group,
    ])