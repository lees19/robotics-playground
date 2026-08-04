import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# from .robot_pose import RobotPose
from tf2_ros import Buffer, TransformListener

# from visualization_msgs.msg import Marker
from .lidar_processor import LidarProcessor
from .occupancy_grid import OccupancyGridMapper
from .scan_matching import ICP


class SlamNode(Node):
    def __init__(self):
        super().__init__("slam_node")
        self.lidar = LidarProcessor()
        # self.robot_pose = RobotPose()
        self.scan_match = ICP()
        self.occupancy_grid = OccupancyGridMapper()
        self.lidar_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 1
        )
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", 1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose = np.eye(3)
        self.prev_odom = None

    def scan_callback(self, msg: LaserScan):
        scan_robot = self.lidar.process_scan(msg)

        # Keep track using own Odometry pose
        # world_points = self.robot_to_world(robot_points)

        # Use TF2 for odometry transform
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame="odom",
                source_frame=msg.header.frame_id,
                time=msg.header.stamp,
                timeout=Duration(seconds=0.1),
            )
        except Exception as e:
            self.get_logger().warn(str(e))
            return

        # predicted odometry pose
        T_odom = self.tf_to_matrix(transform)

        # Not sure exactly why this is important yet
        T_icp = self.scan_match.align(scan_robot, T_odom)
        self.pose = self.pose @ T_icp

        # predicted pose
        scan_world = self.transform_points(scan_robot, self.pose)

        self.occupancy_grid.update_grid(self.pose, scan_world)
        map = self.occupancy_grid.create_occupancy_map(
            frame_id="odom", stamp=msg.header.stamp
        )
        self.map_pub.publish(map)

    def odom_callback(self, msg: Odometry):
        self.robot_pose.update(msg)

    def tf_to_matrix(self, transform: TransformStamped):
        q = transform.transform.rotation
        yaw = np.arctan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        c = np.cos(yaw)
        s = np.sin(yaw)

        T = np.eye(3)
        T[:2, :2] = np.array([[c, -s], [s, c]])
        T[:2, 2] = [
            transform.transform.translation.x,
            transform.transform.translation.y,
        ]

        return T

    def transform_points(self, points, T):
        ones = np.ones((points.shape[0], 1))
        points_h = np.hstack([points, ones])
        transformed = (T @ points_h.T).T

        return transformed[:, :2]


def main():

    rclpy.init()

    node = SlamNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
