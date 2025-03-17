import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="True",
            description="Make MoveIt use simulation time. This is needed for trajectory planing in simulation.",
            choices=["True", "False"]
        ))

    # MoveIt config location
    moveit_dir = os.path.join(get_package_share_directory("robot_moveit_config"))

    # Planning Context
    moveit_config=(
        MoveItConfigsBuilder("robot")
        .robot_description(os.path.join(get_package_share_directory('robot_description'), 'urdf', 'robot.gazebo.xacro'))
        .robot_description_semantic(os.path.join(get_package_share_directory("robot_description"), "srdf", "crx10ia_l.srdf"))
        .trajectory_execution(os.path.join(moveit_dir, 'config', 'moveit_controllers.yaml'))
        .robot_description_kinematics(os.path.join(moveit_dir, 'config', 'kinematics.yaml'))
        .joint_limits(os.path.join(moveit_dir, 'config', 'joint_limits.yaml'))
        .planning_scene_monitor(
            publish_robot_description=True, 
            publish_robot_description_semantic=True, 
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True
        )
        .to_moveit_configs()
    )

    # Move Group Node
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": use_sim_time
            },
        ]
    )

    # Robot State Publisher Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
            {
                "use_sim_time": use_sim_time
            }
        ],
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
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", os.path.join(moveit_dir, 'config', 'moveit.rviz')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {
                "use_sim_time": use_sim_time
            },
        ],
    )

    # Add ros2_control_node for simulation
    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            os.path.join(moveit_dir,"config","controllers.yaml"), 
            {"use_sim_time": use_sim_time}
        ],
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
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Manipulator Controller
    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )

    # Gazebo nodes
    world = os.path.join(get_package_share_directory("robot_description"),
                         "world", "empty_world.sdf")

    # Launch Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch", "/gz_sim.launch.py"]),
        launch_arguments={
            "gz_args": f"-r -v 4 {world}",
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
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output='screen',
    )  

    return LaunchDescription(
        declared_arguments +
        [ 
        move_group,
        gazebo,
        spawn_robot,
        gz_bridge,
        robot_state_publisher,
        static_tf,
        ros2_control,
        joint_state_controller,
        arm_controller,
        rviz,
    ])