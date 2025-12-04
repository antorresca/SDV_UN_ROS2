import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from nav_msgs.msg import Path

class SDVTranslate(Node):
    def __init__(self):
        super().__init__('sdv_translate')

        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        self.client = ActionClient(
            self,
            ComputePathToPose,
            '/planner_server/compute_path_to_pose'
        )

        self.path_pub = self.create_publisher(Path, '/path', 10)

    def goal_callback(self, pose):
        self.get_logger().info("Sending goal to planner...")

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = pose

        self.client.wait_for_server()

        send_goal_future = self.client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response)

    def goal_response(self, future):
        result = future.result().get_result()
        self.get_logger().info("Planner returned a path!")
        self.path_pub.publish(result.path)

def main(args=None):
    rclpy.init(args=args)
    node = SDVTranslate()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()