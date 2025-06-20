#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/string.hpp>

#include <sstream>
#include <vector>
#include <string>
#include <algorithm>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  //auto node = rclcpp::Node::make_shared("ur_reaction_node");
  auto node = rclcpp::Node::make_shared("ur_reaction_node", rclcpp::NodeOptions().use_intra_process_comms(true).parameter_overrides({
    {"use_sim_time", rclcpp::ParameterValue(false)}
  }));

  moveit::planning_interface::MoveGroupInterface move_group_interface(node, "arm");

  // === Offsets (configurable) ===
  double x_offset = 1.0;
  double table_offset = 0.225; // your base 0.795 m, theirs 1.02 m

  std::vector<double> values;
  bool received = false;

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  auto sub = node->create_subscription<std_msgs::msg::String>(
      "/ur3e/pong/ros", qos,
      [&](const std_msgs::msg::String::SharedPtr msg)
      {
          std::istringstream iss(msg->data);
      std::string token;
      values.clear();

      while (std::getline(iss, token, ','))
      {
        // Trim whitespace
        token.erase(std::remove_if(token.begin(), token.end(), ::isspace), token.end());
        try {
          values.push_back(std::stod(token));
        } catch (...) {
          RCLCPP_WARN(node->get_logger(), "Failed to parse float from token: '%s'", token.c_str());
        }
      }

      if (values.size() >= 9)
      {
        received = true;
        RCLCPP_INFO(node->get_logger(), "Parsed %ld values from /ur3e/pong/ros", values.size());
      }
      else
      {
        RCLCPP_WARN(node->get_logger(), "Not enough values in string: %s", msg->data.c_str());
      }
      });

  rclcpp::Rate rate(10);
  int attempts = 0;
  const int max_attempts = 50;

  while (rclcpp::ok() && !received && attempts++ < max_attempts)
  {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  if (!received)
  {
    RCLCPP_ERROR(node->get_logger(), "No valid message received within timeout.");
    rclcpp::shutdown();
    return 1;
  }

  // Extract and transform position
  double ur3_x = values[6];
  double ur3_y = values[7];
  double ur3_z = values[8];

  double target_x = x_offset - ur3_x;
  double target_y = -ur3_y;
  double target_z = ur3_z + table_offset;

  RCLCPP_INFO(node->get_logger(), "Target: x=%.4f y=%.4f z=%.4f", target_x, target_y, target_z);

  // Build new pose
  geometry_msgs::msg::Pose target_pose = move_group_interface.getCurrentPose().pose;
  target_pose.position.x = target_x;
  target_pose.position.y = target_y;
  target_pose.position.z = target_z;

  move_group_interface.setPoseTarget(target_pose);
  bool success = static_cast<bool>(move_group_interface.move());

  if (success)
    RCLCPP_INFO(node->get_logger(), "Move executed.");
  else
    RCLCPP_WARN(node->get_logger(), "Move failed.");

  rclcpp::shutdown();
  return 0;
}
