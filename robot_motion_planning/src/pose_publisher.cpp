/**
 * Program to print end-effector pose and publish it as a string
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
   auto publisher = node->create_publisher<std_msgs::msg::String>("/ping/ros", 10);
     
   // We spin up a SingleThreadedExecutor for the current state monitor to get robot state
   rclcpp::executors::SingleThreadedExecutor executor;
   executor.add_node(node);
   std::thread spinner = std::thread([&executor]() { executor.spin(); });
 
   // Create the MoveIt MoveGroup Interface
   moveit::planning_interface::MoveGroupInterface move_group_interface(node, "arm");
 
   // Get current pose
   geometry_msgs::msg::Pose current_pose = move_group_interface.getCurrentPose().pose;
 
   // Log the pose
   RCLCPP_INFO(node->get_logger(), "Current pose: %f %f %f %f %f %f %f",
     current_pose.position.x,
     current_pose.position.y,
     current_pose.position.z,
     current_pose.orientation.x,
     current_pose.orientation.y,
     current_pose.orientation.z,
     current_pose.orientation.w);
 
   // Format pose into a single space-separated string
   std::ostringstream oss;
   oss << current_pose.position.x << " "
       << current_pose.position.y << " "
       << current_pose.position.z << " "
       << current_pose.orientation.x << " "
       << current_pose.orientation.y << " "
       << current_pose.orientation.z << " "
       << current_pose.orientation.w;
 
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
 