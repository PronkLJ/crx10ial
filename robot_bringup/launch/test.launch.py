import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    moveit_dir = os.path.join(get_package_share_directory("robot_moveit_config"))


    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("robot_description"),"urdf","robot.gazebo.xacro",]),
    ])
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )
    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )

    # Gazebo nodes
    world = os.path.join(get_package_share_directory("robot_description"),
                         "world", "empty_world.sdf")

    # Launch Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch", "/gz_sim.launch.py"]),
        launch_arguments={
            "gz_args": f"-r {world}",
            "on_exit_shutdown": "True",
        }.items(),
    )

    # Spawn Gazebo model
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_model',
        arguments=[
            '-name', 'crx10ia_l', 
            '-topic', 'robot_description', 
            '-z', '0.0'],
        output='both',
    )

    # Bridge topics from Gazebo to ROS2
    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output='screen',
    )  


    # Move Group
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                moveit_dir,
                'launch/move_group.launch.py'))
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
        parameters=[os.path.join(moveit_dir,"config","controllers.yaml",)],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )

    return LaunchDescription([ 
        gazebo,
        spawn_robot,
        gz_bridge_node,
        robot_state_publisher_node,
        joint_state_controller,
        arm_controller,
        rviz,
        move_group,
        static_virtual_joint_tfs,
        ros2_control,
    ])