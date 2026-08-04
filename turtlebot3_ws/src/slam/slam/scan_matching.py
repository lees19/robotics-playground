import numpy as np
from geometry_msgs.msg import TransformStamped
from sklearn.neighbors import NearestNeighbors
from scipy.spatial import KDTree


class ICP:
    def __init__(self):
        self.target = None
        self.prev_odom = None
        self.max_iter = 10
        self.min_error = 1e-3
        self.target_kdtree = None
        # self.nearest = NearestNeighbors()

    def align(self, source: np.ndarray, T_odom: np.ndarray):
        if self.target is None:
            # save the first lidar scan we receive,
            # return the identity matrix, since there is no correction
            self.target = source.copy()
            self.prev_odom = T_odom.copy()
            return np.eye(3)
        source_original = source.copy()
        self.target_kdtree = KDTree(self.target)
        T_motion = np.linalg.inv(self.prev_odom) @ T_odom
        source = self.transform_points(source, T_motion)
        T = T_motion
        for i in range(self.max_iter):
            matched_source, matched_target, _ = self.find_correspondence(
                source, self.target
            )

            diff = matched_source - matched_target
            error = np.mean(np.sum(diff**2, axis=1))
            if error < self.min_error:
                break

            if len(source) < 3 or len(matched_source) == 0:
                break
            T_new = self.kabsch(matched_source, matched_target)
            T = np.dot(T_new, T)
            source = self.transform_points(source, T_new)

        self.target = source_original
        self.prev_odom = T_odom
        return T

    def kabsch(self, source: np.ndarray, target: np.ndarray):
        source_mean = np.mean(source, axis=0)
        target_mean = np.mean(target, axis=0)

        source_centered = source - source_mean
        target_centered = target - target_mean

        covariance = np.dot(target_centered.T, source_centered)
        U, S, Vt = np.linalg.svd(covariance)
        R = np.dot(U, Vt)
        # special case: rotation becomes a reflection (?)
        # look up why
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = np.dot(U, Vt)

        t = target_mean - np.dot(R, source_mean)

        T = np.eye(3)
        T[:2, :2] = R
        T[:2, 2] = t

        return T

    def find_correspondence(self, source: np.ndarray, target: np.ndarray):
        # Create KDTree from previous scan
        source_kdtree = KDTree(source)

        # Query target points on the source points
        _, ts_idxs = source_kdtree.query(target)
        # Query source points on the target points
        _, st_idxs = self.target_kdtree.query(source)

        # reject points if they do not match
        source_idxs = np.arange(len(st_idxs))

        # inverse the transform to see if we get the source point back
        # example: the nth source point should map back
        # after going through the target point
        mask = ts_idxs[st_idxs] == source_idxs

        matched_src = source[mask]
        matched_target = target[st_idxs[mask]]

        return matched_src, matched_target, mask

    def transform_points(self, points: np.ndarray, T: np.ndarray):
        ones = np.ones((points.shape[0], 1))
        points_h = np.hstack([points, ones])
        transformed = (T @ points_h.T).T

        return transformed[:, :2]
