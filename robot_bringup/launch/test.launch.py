import os, yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def load_yaml(package_name, file_name):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_name)
    with open(absolute_file_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)

def generate_launch_description():

    # Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="True",
            description="Make MoveIt use simulation time. This is needed for trajectory planing in simulation.",
        ))

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([
            os.path.join(get_package_share_directory("robot_description"), "urdf", "robot.gazebo.xacro"),
        ]),
    ])
    robot_description = {"robot_description": robot_description_content}

    # Robot Description Content - MoveIt
    robot_description_content_moveit = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([
            os.path.join(get_package_share_directory("robot_description"), "urdf", "robot.moveit.xacro"),
        ]),
    ])
    robot_description_moveit = {
        "robot_description": robot_description_content_moveit}
    
    # Robot Description Semantic
    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([
            os.path.join(get_package_share_directory("robot_moveit_config"), "config", "crx10ia_l.srdf")
        ]),
    ])
    robot_description_semantic = {
        "robot_description_semantic": 
        ParameterValue(robot_description_semantic_content, value_type=str)    
    }
    
    # Robot Description Kinematics
    robot_description_kinematics = {
        "robot_description_kinematics":
        load_yaml("robot_moveit_config", os.path.join("config", "kinematics.yaml"))
    }

    # Joint Limits
    joint_limits = {
        "joint_limits":
        load_yaml("robot_moveit_config", os.path.join("config", "joint_limits.yaml"))
    }

    # Planning Scene Monitor
    planning_scene_monitor = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description_semantic": True,
    }

    # Trajectory Execution Configuration
    moveit_controller_manager = {
        "moveit_controller_manager":
        "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    moveit_controllers = ParameterFile(
        PathJoinSubstitution([
            os.path.join(get_package_share_directory("robot_moveit_config"), "config", "moveit_controllers.yaml")
        ]),
        allow_substs=True,
    )

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    # Planning Context
    moveit_config=(MoveItConfigsBuilder("robot").to_moveit_configs())

    # Move Group Node
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits,
            planning_scene_monitor,
            moveit_controller_manager,
            moveit_controllers,
            trajectory_execution,

            moveit_config.planning_pipelines,
            moveit_config.moveit_cpp,
            moveit_config.pilz_cartesian_limits,
            {
                "use_sim_time": use_sim_time
            },
        ]
    )

    # Robot State Publisher Node
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
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
        arguments=["-d", os.path.join(get_package_share_directory("robot_moveit_config"), 'config', 'moveit.rviz')],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            moveit_config.planning_pipelines,
            {
                "use_sim_time": use_sim_time
            },
        ],
    )

    # ROS2 Controller Node
    ros2_controllers_path = os.path.join(get_package_share_directory("robot_moveit_config"), "config", "controllers.yaml")
    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description_moveit, ros2_controllers_path], #Setting this to robot_description does not work..
        output="both",
    )

    # Load controllers
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
            "gz_args": f"-r -v 0 {world}",
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
        declared_arguments + [ 
        gz_bridge,
        gazebo,
        spawn_robot,
        robot_state_publisher,
        joint_state_controller,
        arm_controller,
        move_group,
        static_tf,
        rviz,
        ros2_control,
    ])