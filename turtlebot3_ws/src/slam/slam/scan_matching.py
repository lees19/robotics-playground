import numpy as np
from geometry_msgs.msg import TransformStamped
# from sklearn.neighbors import NearestNeighbors


class ICP:
    def __init__(self):
        self.prev_scan_world = None
        # self.nearest = NearestNeighbors()

    def align(self, scan_world: np.ndarray):
        if self.prev_scan_world is None:
            # save the first lidar scan we receive,
            # return the identity matrix, since there is no correction
            self.prev_scan_world = scan_world
            return np.eye(3)
