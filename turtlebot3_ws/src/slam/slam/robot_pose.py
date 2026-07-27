import numpy as np
from nav_msgs.msg import Odometry


class RobotPose:
    def __init__(self):
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

    def update(self, msg: Odometry):
        pose = msg.pose.pose

        self.robot_x = pose.position.x
        self.robot_y = pose.position.y

        q = pose.orientation

        self.robot_yaw = np.arctan2(
            2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
        )
