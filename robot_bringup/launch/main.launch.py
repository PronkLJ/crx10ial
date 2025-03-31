import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import PythonExpression

def generate_launch_description():
    ## Arguments
    sim_arg = DeclareLaunchArgument(
        name='sim', 
        default_value='false', 
        choices=['true', 'false'],
        description='Set to true or false to switch between hardware and simulation'
    )
    
    gazebo_arg = DeclareLaunchArgument(
        name='gazebo', 
        default_value='true', 
        choices=['true', 'false'],
        description='Loading Gazebo & MoveIt or just MoveIt in simulation'
    )

    launch_group = GroupAction([
        # Launch MoveIt + Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('robot_bringup'), 'launch', 'moveit_gazebo.launch.py')]),
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('sim'), "' == 'true' and '", 
                LaunchConfiguration('gazebo'), "' == 'true'"  
            ]))    
        ),

        # Only launch MoveIt for simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('robot_bringup'), 'launch', 'moveit.launch.py')]),
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('sim'), "' == 'true' and '", 
                LaunchConfiguration('gazebo'), "' == 'false'"  
            ]))    
        ),

        # Physical robot control
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('robot_bringup'), 'launch', 'control.launch.py')]),
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('sim'), "' == 'false'"
            ]))
        ),
    ])

    # Add ground plane constraint
    ground_plane_node = Node(
        package="robot_motion_planning",
        executable="add_ground_plane",
        output="screen",
    )

    # Add ceiling plane constraint
    ceiling_plane_node = Node(
        package="robot_motion_planning",
        executable="add_ceiling_plane",
        output="screen",
    )

    return LaunchDescription([
        sim_arg,
        gazebo_arg,
        launch_group,
        ground_plane_node,
        #ceiling_plane_node,
    ])
