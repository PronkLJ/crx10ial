#include <memory> 
#include <rclcpp/rclcpp.hpp> 
#include <moveit/move_group_interface/move_group_interface.h> 
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include <tf2/LinearMath/Quaternion.h>
#include <string>
#include <iostream>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_program");

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("move_program", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    static const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface move_group_arm(move_group_node, PLANNING_GROUP);

    // Move to home position first
    // RCLCPP_INFO(LOGGER, "Moving to home position...");
    // move_group_arm.setStartStateToCurrentState();

    // std::vector<double> joint_group_positions_arm = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // move_group_arm.setJointValueTarget(joint_group_positions_arm);

    // moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
    // if (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS) {
    //     move_group_arm.execute(my_plan_arm);
    // } else {
    //     RCLCPP_ERROR(LOGGER, "Not able to plan and execute.");
    //     rclcpp::shutdown();
    //     return 0;
    // }

    // Define multiple target positions
    std::vector<std::tuple<double, double, double, bool>> target_positions = {
        // Corrected z (height) by 0.79 because of the table height
        // Add more positions here to add to the path planning
        //{0.5, 0.0, 1.49, false},    // Position 1 (Facing forward)
        {0.4,  0.0,  1.09, true},   // Position 2 (Facing downward)
        {0.4,  0.0, 1.0, true},   // Position 3 (Facing downward)
        {0.4,  0.0,  1.09, true},   // Position 2 (Facing downward)
    };

    for (size_t i = 0; i < target_positions.size(); ++i) {
        double x, y, z;
        bool face_down;
        std::tie(x, y, z, face_down) = target_positions[i];
    
        RCLCPP_INFO(LOGGER, "Moving to position %ld...", i + 1);
    
        geometry_msgs::msg::Pose start_pose = move_group_arm.getCurrentPose().pose;
    
        // Orientation
        tf2::Quaternion quaternion;
        if (face_down) {
            quaternion.setRPY(0, M_PI_2, 0);
        } else {
            quaternion.setRPY(0, 0, 0);
        }
    
        geometry_msgs::msg::Pose target_pose = start_pose;
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;
        target_pose.orientation = tf2::toMsg(quaternion);
    
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose);
    
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.01;  // resolution of interpolation
        const double jump_threshold = 0.0;  // disables jump detection
    
        double fraction = move_group_arm.computeCartesianPath(
            waypoints, eef_step, jump_threshold, trajectory);
    
        if (fraction > 0.95) {
            moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
            cartesian_plan.trajectory_ = trajectory;
            move_group_arm.execute(cartesian_plan);
        } else {
            RCLCPP_ERROR(LOGGER, "Cartesian path for position %ld failed: %.2f%% achieved", i + 1, fraction * 100.0);
            rclcpp::shutdown();
            return 0;
        }
    }

    rclcpp::shutdown();
    return 0;
}