/**
 * Program to print end‑effector pose AND six joint positions,
 * then publish them as a single space‑separated string.
 */

 #include <moveit/move_group_interface/move_group_interface.h>
 #include <geometry_msgs/msg/pose.hpp>
 #include <std_msgs/msg/string.hpp>
 #include <rclcpp/rclcpp.hpp>
 #include <sstream>
 
int main(int argc, char* argv[]) {
    // Initialize ROS
    rclcpp::init(argc, argv);

    // Declare Node
    auto node = rclcpp::Node::make_shared(
    "publish_pose",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Publisher to /ping/ros
    auto publisher = node->create_publisher<std_msgs::msg::String>("/crx10ial/ping/ros", 10);
        
    // We spin up a SingleThreadedExecutor for the current state monitor to get robot state
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner = std::thread([&executor]() { executor.spin(); });

    //Listen to joint states
    
    // Create a variable to store the latest joint state message
    sensor_msgs::msg::JointState::SharedPtr latest_joint_state;

    // Create a mutex to protect access to latest_joint_state
    std::mutex joint_state_mutex;

    // Create a subscriber to /joint_states
    auto joint_state_sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [&latest_joint_state, &joint_state_mutex](const sensor_msgs::msg::JointState::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(joint_state_mutex);
            latest_joint_state = msg;
        }
    );

    // Wait until we receive at least one joint state message
    while (rclcpp::ok()) {
        {
            std::lock_guard<std::mutex> lock(joint_state_mutex);
            if (latest_joint_state) {
                break;
            }
        }
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    // The joint positions for joints 1-6 are stored in:
    // latest_joint_state->position[0] through latest_joint_state->position[5]
 
    // Create the MoveIt MoveGroup Interface
    moveit::planning_interface::MoveGroupInterface move_group_interface(node, "arm");

    // Get current pose
    geometry_msgs::msg::Pose current_pose = move_group_interface.getCurrentPose().pose;

    // Log the pose and joint positions
    {
        std::lock_guard<std::mutex> lock(joint_state_mutex);
        if (latest_joint_state && latest_joint_state->position.size() >= 6) {
            RCLCPP_INFO(node->get_logger(),
                "Joint positions: %f %f %f %f %f %f | Current pose: %f %f %f",
                latest_joint_state->position[0],
                latest_joint_state->position[1],
                latest_joint_state->position[2],
                latest_joint_state->position[3],
                latest_joint_state->position[4],
                latest_joint_state->position[5],
                current_pose.position.x,
                current_pose.position.y,
                current_pose.position.z
            );
        } else {
            RCLCPP_WARN(node->get_logger(), "Joint state message does not have enough positions.");
        }
    }

    // Format pose and joint positions into a single space-separated string
    std::ostringstream oss;
    if (latest_joint_state && latest_joint_state->position.size() >= 6) {
    oss << latest_joint_state->position[0] << ", "
        << latest_joint_state->position[1] << ", "
        << latest_joint_state->position[2] << ", "
        << latest_joint_state->position[3] << ", "
        << latest_joint_state->position[4] << ", "
        << latest_joint_state->position[5] << ", "
        << current_pose.position.x << ", "
        << current_pose.position.y << ", "
        << current_pose.position.z;
    }
    std_msgs::msg::String msg;
    msg.data = oss.str();
    // Log and publish
    RCLCPP_INFO(node->get_logger(), "Publishing: '%s'", msg.data.c_str());
    publisher->publish(msg);
    // Short sleep to allow message to propagate
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    // Shutdown ROS
    rclcpp::shutdown();
    spinner.join();
    return 0;
}