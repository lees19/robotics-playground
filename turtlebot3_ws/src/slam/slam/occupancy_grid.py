import numpy as np

# from .robot_pose import RobotPose
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid


class OccupancyGridMapper:
    def __init__(self, width: int = 400, height: int = 400, resolution: float = 0.05):
        self.resolution = resolution
        self.width = width
        self.height = height

        self.grid = np.zeros((height, width), dtype=np.int8)
        self.log_odds = np.zeros((height, width), dtype=np.float32)

        # still dont understand how to choose this origin point
        self.origin = np.array([-10.0, -10.0])

    def world_to_grid(self, world_points: np.ndarray):
        grid_points = ((world_points - self.origin) / self.resolution).astype(np.int32)
        return grid_points

    def log_to_grid(self):
        prob = 1 - 1 / (1 + np.exp(self.log_odds))

        self.grid = np.full(prob.shape, -1, dtype=np.int8)

        self.grid[prob < 0.35] = 0
        self.grid[prob > 0.65] = 100

        return self.grid

    def create_occupancy_map(self, frame_id: str, stamp):
        grid = self.log_to_grid()

        msg = OccupancyGrid()
        msg.header.frame_id = frame_id
        msg.header.stamp = stamp

        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height

        msg.info.origin.position.x = self.origin[0]
        msg.info.origin.position.y = self.origin[1]
        msg.info.origin.position.z = 0.0

        msg.info.origin.orientation.w = 1.0

        msg.data = grid.flatten().tolist()

        return msg

    def bresenham(self, robot_cell, obstacle):
        # basically from the robot, run bresenham on all of the given lidar points
        x0, y0 = robot_cell[0], robot_cell[1]
        x1, y1 = obstacle[0], obstacle[1]
        dx = np.abs(x1 - x0)
        dy = np.abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1

        error = dx - dy
        cells = []
        while True:
            cells.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * error
            if e2 > -dy:
                error -= dy
                x0 += sx

            if e2 < dx:
                error += dx
                y0 += sy

        return cells

    def update_grid(self, pose: np.ndarray, scan_world: np.ndarray):
        grid_points = self.world_to_grid(scan_world)

        robot_cell = self.world_to_grid(pose[:2, 2])

        for obstacle in grid_points:
            # free cells
            for cell in self.bresenham(robot_cell, obstacle)[:-1]:
                x, y = cell
                self.log_odds[y, x] -= 0.4

            # occupied endpoint
            x, y = obstacle
            self.log_odds[y, x] += 0.85

        self.log_odds = np.clip(self.log_odds, -5, 5)
