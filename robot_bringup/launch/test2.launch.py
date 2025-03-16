import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    moveit_dir = os.path.join(get_package_share_directory("robot_moveit_config"))
    # Planning Context
    moveit_config=(
        MoveItConfigsBuilder("robot")
        .robot_description((os.path.join(get_package_share_directory('robot_description'), 'urdf', 'robot.gazebo.xacro')))
        .trajectory_execution(os.path.join(moveit_dir, 'config', 'controllers.yaml'))
        .robot_description_kinematics(os.path.join(moveit_dir, 'config', 'kinematics.yaml'))
        .joint_limits(os.path.join(moveit_dir, 'config', 'joint_limits.yaml'))
        .planning_scene_monitor(
            publish_robot_description=True, 
            publish_robot_description_semantic=True,
            publish_planning_scene=True,  # Ensure this is added
        )
        .to_moveit_configs()
    )

    # Move Group Node
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()]
    )
    # Robot State Publisher Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output="both",
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
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
        parameters=[{"use_sim_time": True}],
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
    # Add ros2_control_node for simulation
    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            os.path.join(moveit_dir,"config","controllers.yaml"), 
            {"use_sim_time": True}
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
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
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"
            '/world/empty/model/crx10ia_l/joint_state@sensor_msgs/msg/JointState]gz.msgs.Model',
        ],
        remappings=[
            ('/world/empty/model/crx10ia_l/joint_state', 'joint_states'),
        ],
        output='screen',
    )  
    # Static Transform Node
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"]
    )
    # RViz Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", os.path.join(get_package_share_directory('robot_moveit_config'), 'config', 'moveit.rviz')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )
    return LaunchDescription([ 
        gazebo,
        spawn_robot,
        gz_bridge_node,
        joint_state_controller,
        arm_controller,
        rviz_node,
        static_tf,
        ros2_control,
        robot_state_publisher,

        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_controller,
                on_exit=[move_group]
            )
        )
    ])