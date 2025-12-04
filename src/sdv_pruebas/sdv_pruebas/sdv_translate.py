import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class SDVTranslate(Node):
    def __init__(self):
        super().__init__('sdv_translate')
        self.sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.callback,
            10)
        self.pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

    def callback(self, msg):
        self.get_logger().info("Relay: received goal from RViz")
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SDVTranslate()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()