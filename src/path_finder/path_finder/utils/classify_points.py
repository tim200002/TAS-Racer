import numpy as np
from collections import deque


def classify_points(start_point, binary_occupancy_grid: np.ndarray):
    """"Classifies all points in map into categories depending on start point
    Categories:
    0) Non reachable
    1) reachable
    2) belonging to line bordering (is also fatal but extra class)
    3) fatal
    """
    (x_start, y_start) = start_point

    # convert all fatal points to value 3
    reachability_grid = binary_occupancy_grid.copy() * 3

    # check start point is valid
    assert x_start >= 0 and y_start >= 0 and y_start < reachability_grid.shape[
        0] and x_start < reachability_grid.shape[1]
    assert reachability_grid[y_start, x_start] != 2

    points_to_explore = deque()
    points_to_explore.append(start_point)

    while len(points_to_explore) != 0:
        # pop first element from stack and mark reachable
        (x, y) = points_to_explore.pop()
        reachability_grid[y, x] = 1

        # find new points and add to queue
        new_points = _find_new_points((x, y), reachability_grid)
        for point in new_points:
            points_to_explore.append(point)

    return reachability_grid


def _find_new_points(start_point, reachability_grid: np.ndarray):
    (x, y) = start_point
    assert reachability_grid[y, x] == 1

    points_of_iterst = []
    for x_dir in [-1, 0, 1]:
        for y_dir in [-1, 0, 1]:
            new_point = (x+x_dir, y+y_dir)
            if _check_point(new_point, reachability_grid):
                points_of_iterst.append(new_point)
    return points_of_iterst


def _check_point(point, reachability_grid: np.ndarray):
    (x, y) = point
    # we are inbound
    if x >= 0 and y >= 0 and y < reachability_grid.shape[0] and x < reachability_grid.shape[1]:
        if reachability_grid[y, x] == 0:
            return True
        if reachability_grid[y, x] == 3:
            # update from fatal to border
            reachability_grid[y, x] = 2
    return False
