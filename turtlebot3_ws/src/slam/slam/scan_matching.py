import numpy as np
from geometry_msgs.msg import TransformStamped
from sklearn.neighbors import NearestNeighbors
from scipy.spatial import KDTree


class ICP:
    def __init__(self):
        self.target = None

        # self.nearest = NearestNeighbors()

    def align(self, source: np.ndarray):
        if self.target is None:
            # save the first lidar scan we receive,
            # return the identity matrix, since there is no correction
            self.target = source
            return np.eye(3)

        matched_source, matched_target = self.find_correspondence(source)
        self.kabsch(matched_source, matched_target)

    def kabsch(self, matched_source: np.ndarray, matched_target: np.ndarray):

        pass

    def find_correspondence(self, source: np.ndarray):
        # Create KDTree from previous scan
        target_kdtree = KDTree(self.target)
        source_kdtree = KDTree(source)

        # Query target points on the source points
        _, ts_idxs = source_kdtree.query(self.target)
        # Query source points on the target points
        _, st_idxs = target_kdtree.query(source)

        # reject points if they do not match
        source_idxs = np.arange(len(st_idxs))

        # inverse the transform to see if we get the source point back
        # example: the first source point -> target point
        #          using the target point, then should map back to
        #          the first source point
        mask = ts_idxs[st_idxs] == source_idxs

        matched_src = source[mask]
        matched_target = self.target[mask]

        return matched_src, matched_target
