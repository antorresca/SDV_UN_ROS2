import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import numpy as np
import open3d as o3d
import tf_transformations

class SDVScanMatcher(Node):
    def __init__(self):
        super().__init__('lidar_icp_odometry')

        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pub = self.create_publisher(Odometry, '/odom', 10)

        self.prev_cloud = None

        # pose estimada
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    def laser_to_cloud(self, msg):
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.array(msg.ranges)

        # Filtrar rangos inválidos
        mask = np.isfinite(ranges)
        angles = angles[mask]
        ranges = ranges[mask]

        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)

        pts = np.vstack((xs, ys)).T

        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(pts)

        return cloud

    def scan_callback(self, msg):
        cloud = self.laser_to_cloud(msg)

        if self.prev_cloud is None:
            self.prev_cloud = cloud
            return

        # ICP para estimar la transformación
        icp = o3d.pipelines.registration.registration_icp(
            cloud,
            self.prev_cloud,
            0.5,  # max correspondence distance
            np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint()
        )

        T = icp.transformation

        # Extraer pose incremental
        dx = T[0, 3]
        dy = T[1, 3]
        dyaw = np.arctan2(T[1, 0], T[0, 0])

        # Integrar
        c = np.cos(self.yaw)
        s = np.sin(self.yaw)
        self.x += c*dx - s*dy
        self.y += s*dx + c*dy
        self.yaw += dyaw

        # Publicar odometría
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        q = tf_transformations.quaternion_from_euler(0, 0, self.yaw)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        self.pub.publish(odom)

        self.prev_cloud = cloud


def main(args=None):
    rclpy.init(args=args)
    node = SDVScanMatcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
