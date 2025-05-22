/*
Program that receives position and orientation of UR3, and moves to the same spot.
Might need to mirror the positions as robots are opposite each other.

For now, copy of dynamic_move_program, will adjust later.
*/

#include <memory> 
#include <rclcpp/rclcpp.hpp> 
#include <moveit/move_group_interface/move_group_interface.h> 
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include <tf2/LinearMath/Quaternion.h>
#include <string>
#include <sstream>
#include <iostream>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/string.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_program");

class MoveProgram : public rclcpp::Node {
public:
    MoveProgram() : Node("move_program") {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), PLANNING_GROUP);
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "/target_position", 10,
            std::bind(&MoveProgram::target_callback, this, std::placeholders::_1)
        );

        // Move to home position on startup
        move_to_home_position();
    }

private:
    void target_callback(const std_msgs::msg::String::SharedPtr msg) {
        double x, y, z;
        std::istringstream iss(msg->data);
        if (!(iss >> x >> y >> z)) {
            RCLCPP_ERROR(this->get_logger(), "Invalid format. Expected 'x y z' as string.");
            return;
        }
        //0.7 0.3 1.0

        RCLCPP_INFO(this->get_logger(), "Received target: x=%f, y=%f, z=%f", x, y, z);

        geometry_msgs::msg::Pose target_pose;
        target_pose.orientation.w = 1.0;
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;

        move_group_->setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            move_group_->execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan to target position.");
        }
    }

    void move_to_home_position() {
        const moveit::core::JointModelGroup *joint_model_group = 
            move_group_->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

        moveit::core::RobotStatePtr current_state = move_group_->getCurrentState(10);
        std::vector<double> joint_group_positions;
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

        // Home position
        joint_group_positions[0] = 0.00;
        joint_group_positions[1] = 0.00;
        joint_group_positions[2] = -0.523598776; // -30 degrees in radians
        joint_group_positions[3] = 0.00;
        joint_group_positions[4] = -1.04719755; // -60 degrees in radians
        joint_group_positions[5] = 0.00;

        move_group_->setJointValueTarget(joint_group_positions);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            move_group_->execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to move to home position.");
        }
    }

    static const std::string PLANNING_GROUP;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

const std::string MoveProgram::PLANNING_GROUP = "arm";

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveProgram>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
