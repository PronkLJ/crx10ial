#include <memory> // Necessary memory include
#include <rclcpp/rclcpp.hpp> // ROS2 client library
#include <moveit/move_group_interface/move_group_interface.h> // MoveIt! for motion planning
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // For converting Euler angles to quaternions
#include <tf2/LinearMath/Quaternion.h>
#include <string.h>
#include <iostream>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

// This MoveIt script allows for trajectory planning and executing of the robot
// For this code, inspiration was gained from the following sources:
// - Cartesian planning with MoveIt2: https://www.youtube.com/watch?v=RaQ8Ibd9vck
// - C++ and MoveIt 2 to Perform Motion Planning: https://www.youtube.com/watch?v=ggOROufX0tE

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_program");

int main (int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_program", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "arm"; 

  moveit::planning_interface::MoveGroupInterface move_group_arm(
    move_group_node, PLANNING_GROUP
  );

  const moveit::core::JointModelGroup *joint_model_group_arm = move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Get Current State
  moveit::core::RobotStatePtr current_state_arm = move_group_arm.getCurrentState(10);

  std::vector<double> joint_group_positions_arm;
  current_state_arm->copyJointGroupPositions(joint_model_group_arm, joint_group_positions_arm);

  // // Go Home (somehow necessary for maintaining roughly the same orientation)
  move_group_arm.setStartStateToCurrentState();
  RCLCPP_INFO(LOGGER, "Going home");
  joint_group_positions_arm[0] = 0.00;
  joint_group_positions_arm[1] = 0.00;
  joint_group_positions_arm[2] = 0.00;
  joint_group_positions_arm[3] = 0.00;
  joint_group_positions_arm[4] = 0.00;
  joint_group_positions_arm[5] = 0.00;
  
  move_group_arm.setJointValueTarget(joint_group_positions_arm);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  bool succes_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);

  
  // Execute (not sure if needed in the final program)
  if (succes_arm) {
    move_group_arm.execute(my_plan_arm);
  }
  else {
    RCLCPP_ERROR(LOGGER, "Not able to plan and execute.");
  }

  // Move to set position
  RCLCPP_INFO(LOGGER, "Move to set position");
  current_state_arm = move_group_arm.getCurrentState(10);
  current_state_arm->copyJointGroupPositions(joint_model_group_arm, joint_group_positions_arm);

  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.x = 0.0;
  target_pose1.orientation.y = 0.0;
  target_pose1.orientation.z = 0.0;
  target_pose1.orientation.w = 0.0;
  target_pose1.position.x = 0.6;
  target_pose1.position.y = -0.17;
  target_pose1.position.z = 0.7;
  move_group_arm.setPoseTarget(target_pose1);

  succes_arm = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);

  // Execute
  if (succes_arm) {
    move_group_arm.execute(my_plan_arm);
  }
  else {
    RCLCPP_ERROR(LOGGER, "Not able to plan and execute.");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
