#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

class AddCeilingPlane : public rclcpp::Node {
public:
    AddCeilingPlane() : Node("add_ceiling_plane") {
        RCLCPP_INFO(this->get_logger(), "Adding virtual ceiling to MoveIt planning scene...");

        // Create a Planning Scene Interface object
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        // Define a collision object for the ceiling plane
        moveit_msgs::msg::CollisionObject ceiling_plane;
        ceiling_plane.id = "ceiling_plane";
        ceiling_plane.header.frame_id = "base_link";

        // Define a box (ceiling plane)
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 2.0;   // X size
        primitive.dimensions[1] = 2.0;   // Y size
        primitive.dimensions[2] = 0.01;  // Z size (thin)

        // Position it at the correct height
        geometry_msgs::msg::Pose ceiling_pose;
        ceiling_pose.position.x = 0.0;
        ceiling_pose.position.y = 0.0;
        ceiling_pose.position.z = 1.20; // Height of ceiling, calculated from the base of the robot

        // Assign to the collision object
        ceiling_plane.primitives.push_back(primitive);
        ceiling_plane.primitive_poses.push_back(ceiling_pose);
        ceiling_plane.operation = ceiling_plane.ADD;

        // Apply to the planning scene
        planning_scene_interface.applyCollisionObjects({ceiling_plane});
        RCLCPP_INFO(this->get_logger(), "Ceiling plane added successfully!");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddCeilingPlane>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
