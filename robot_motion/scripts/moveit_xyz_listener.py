import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MotionPlanRequest
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup

class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot')
        self.subscription = self.create_subscription(
            String,
            'target_position',  # Topic name
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning

        self.ik_service = self.create_client(GetPositionIK, 'compute_ik')
        self.move_action_client = ActionClient(self, MoveGroup, 'move_action')

    def listener_callback(self, msg):
        try:
            x, y, z = map(float, msg.data.split())
            self.move_to_position(x, y, z)
        except ValueError:
            self.get_logger().error("Invalid position format. Use 'x y z' format.")

    def move_to_position(self, x, y, z):
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"  # Adjust according to your robot
        pose_goal.pose.position.x = x
        pose_goal.pose.position.y = y
        pose_goal.pose.position.z = z

        ik_request = GetPositionIK.Request()
        ik_request.ik_request.group_name = "arm"
        ik_request.ik_request.pose_stamped = pose_goal

        future = self.ik_service.call_async(ik_request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        
        if response and response.error_code.val == 1:
            motion_request = MoveGroup.Goal()
            motion_request.request.start_state.is_diff = True
            motion_request.request.goal_constraints.append(response.solution)

            self.move_action_client.wait_for_server()
            future = self.move_action_client.send_goal_async(motion_request)
            rclpy.spin_until_future_complete(self, future)
            result = future.result()

            if result and result.result.error_code.val == 1:
                self.get_logger().info(f"Moved to ({x}, {y}, {z})")
            else:
                self.get_logger().error("Motion execution failed!")
        else:
            self.get_logger().error("IK solution could not be found!")


def main(args=None):
    rclpy.init(args=args)
    node = MoveRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
