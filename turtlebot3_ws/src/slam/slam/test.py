import numpy as np


def bresenham(robot_cell, obstacle):
    # basically from the robot, run bresenham on all of the given lidar points
    robot_cells = robot_cell.reshape(obstacle.shape)
    x0, y0 = robot_cells[:, 0], robot_cells[:, 1]
    x1, y1 = obstacle[:, 0], obstacle[:, 1]
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


robot_start = np.array([0, 0])
robot_ends = np.array([[3, 0], [0, 3], [3, 3]])
solution = np.array([
    [(0, 0), (1, 0), (2, 0), (3, 0)],
    [(0, 0), (0, 1), (0, 2), (0, 3)],
    [(0, 0), (1, 1), (2, 2), (3, 3)],
])

assert bresenham(np.array((0, 0)), (3, 0)) == [(0, 0), (1, 0), (2, 0), (3, 0)]

assert bresenham((0, 0), (0, 3)) == [(0, 0), (0, 1), (0, 2), (0, 3)]

assert bresenham((0, 0), (3, 3)) == [(0, 0), (1, 1), (2, 2), (3, 3)]

assert bresenham(robot_start, robot_ends) == solution
