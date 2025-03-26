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
    RCLCPP_INFO(LOGGER, "Moving to home position...");
    move_group_arm.setStartStateToCurrentState();

    std::vector<double> joint_group_positions_arm = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    move_group_arm.setJointValueTarget(joint_group_positions_arm);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
    if (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS) {
        move_group_arm.execute(my_plan_arm);
    } else {
        RCLCPP_ERROR(LOGGER, "Not able to plan and execute.");
        rclcpp::shutdown();
        return 0;
    }

    // Define multiple target positions
    std::vector<std::tuple<double, double, double, bool>> target_positions = {
        {0.6, -0.17, 0.7, false},  // Position 1 (Facing forward)
        {0.5,  0.2,  0.3, true},   // Position 2 (Facing downward)
        //{0.4, -0.3,  0.75, false}, // Position 3 (Facing forward)
        //{0.55, 0.1,  0.5, true},   // Position 4 (Facing downward)
        {0.45, -0.2, 0.65, false}  // Position 5 (Facing forward)
    };

    for (size_t i = 0; i < target_positions.size(); ++i) {
        double x, y, z;
        bool face_down;
        std::tie(x, y, z, face_down) = target_positions[i];

        RCLCPP_INFO(LOGGER, "Moving to position %ld...", i + 1);

        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;

        // Set orientation
        tf2::Quaternion quaternion;
        if (face_down) {
            quaternion.setRPY(0, M_PI_2, 0); // Rotate 90° around Y-axis
        } else {
            quaternion.setRPY(0, 0, 0); // Facing forward
        }
        target_pose.orientation = tf2::toMsg(quaternion);

        move_group_arm.setPoseTarget(target_pose);
        if (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS) {
            move_group_arm.execute(my_plan_arm);
        } else {
            RCLCPP_ERROR(LOGGER, "Not able to plan and execute position %ld.", i + 1);
            rclcpp::shutdown();
            return 0;
        }
    }

    rclcpp::shutdown();
    return 0;
}