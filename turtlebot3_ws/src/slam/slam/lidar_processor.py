import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class LidarProcessor:
    def scan_to_points(self, msg: LaserScan):
        # turn lidar scans into cartesian points
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment

        valid = np.isfinite(ranges)

        x = ranges[valid] * np.cos(angles[valid])
        y = ranges[valid] * np.sin(angles[valid])

        return np.stack((x, y), axis=1)

    def _create_marker(self, frame_id: str, stamp):
        marker = Marker()

        marker.header.frame_id = frame_id
        marker.header.stamp = stamp

        marker.ns = "lidar"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        return marker

    def point_to_marker(self, points: np.ndarray, frame_id: str, stamp):
        marker = self._create_marker(frame_id, stamp)

        for point in points:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = 0.0

            marker.points.append(p)

        return marker

    def process_scan(self, msg: LaserScan):
        points = self.scan_to_points(msg)

        return points
