#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

class AddGroundPlane : public rclcpp::Node {
public:
    AddGroundPlane() : Node("add_ground_plane") {
        RCLCPP_INFO(this->get_logger(), "Adding virtual ground to MoveIt planning scene...");

        // Create a Planning Scene Interface object
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        // Define a collision object for the ground plane
        moveit_msgs::msg::CollisionObject ground_plane;
        ground_plane.id = "ground_plane";
        ground_plane.header.frame_id = "world";

        // Define a box (ground plane)
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 10.0;  // X size
        primitive.dimensions[1] = 10.0;  // Y size
        primitive.dimensions[2] = 0.01;  // Z size (thin)

        // Position it at the base (just below z=0)
        geometry_msgs::msg::Pose ground_pose;
        ground_pose.position.x = 0.0;
        ground_pose.position.y = 0.0;
        ground_pose.position.z = -0.01;  // Slightly below zero

        // Assign to the collision object
        ground_plane.primitives.push_back(primitive);
        ground_plane.primitive_poses.push_back(ground_pose);
        ground_plane.operation = ground_plane.ADD;

        // Apply to the planning scene
        planning_scene_interface.applyCollisionObjects({ground_plane});
        RCLCPP_INFO(this->get_logger(), "Ground plane added successfully!");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddGroundPlane>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
