import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

from .lidar_processor import LidarProcessor
from .occupancy_grid import OccupancyGridMapper
from .robot_pose import RobotPose


class SlamNode(Node):
    def __init__(self):
        super().__init__("slam_node")
        self.lidar = LidarProcessor()
        self.robot_pose = RobotPose()
        self.occupancy_grid = OccupancyGridMapper()
        self.get_logger().info(
            f"robot: {self.robot_pose.robot_x}, {self.robot_pose.robot_y}"
        )
        self.lidar_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", 10)

        self.marker_pub = self.create_publisher(Marker, "/scan_points", 10)
        from tf2_ros import Buffer, TransformListener

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def scan_callback(self, msg: LaserScan):
        robot_points = self.lidar.process_scan(msg)
        world_points = self.robot_to_world(robot_points)
        self.occupancy_grid.update_grid(self.robot_pose, world_points)

        map = self.occupancy_grid.create_occupancy_map(
            frame_id="odom", stamp=self.get_clock().now().to_msg()
        )
        self.map_pub.publish(map)

        # marker = self.lidar.point_to_marker(
        #     points=world_points, frame_id="odom", stamp=self.get_clock().now().to_msg()
        # )
        # self.marker_pub.publish(marker)

    def odom_callback(self, msg: Odometry):
        self.robot_pose.update(msg)

    def robot_to_world(self, points: np.ndarray):
        cos_theta = np.cos(self.robot_pose.robot_yaw)
        sin_theta = np.sin(self.robot_pose.robot_yaw)

        R = np.array([[cos_theta, -sin_theta], [sin_theta, cos_theta]])

        world_points = (R @ points.T).T
        world_points[:, 0] += self.robot_pose.robot_x
        world_points[:, 1] += self.robot_pose.robot_y

        return world_points


def main():
    rclpy.init()

    node = SlamNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
