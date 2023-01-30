import numpy as np
from collections import deque
from models.point import Point


def classify_points(start_point: Point, binary_occupancy_grid: np.ndarray):
    """"Classifies all points in map into categories depending on start point
    Categories:
    0) Non reachable
    1) reachable
    2) belonging to line bordering track (is also fatal but extra class)
    3) fatal
    """

    # convert all fatal points to value 3
    reachability_grid = binary_occupancy_grid.copy() * 3

    # check start point is valid
    assert start_point.x >= 0 and start_point.y >= 0 and start_point.y < reachability_grid.shape[
        0] and start_point.x < reachability_grid.shape[1]
    assert reachability_grid[start_point.y, start_point.x] != 2

    points_to_explore = deque()
    points_to_explore.append(start_point)

    while len(points_to_explore) != 0:
        # pop first element from stack and mark reachable
        current_point = points_to_explore.pop()
        reachability_grid[current_point.y, current_point.x] = 1

        # find new points and add to queue
        new_points = _find_new_points(current_point, reachability_grid)
        for point in new_points:
            points_to_explore.append(point)

    return reachability_grid


def _find_new_points(start_point: Point, reachability_grid: np.ndarray):
    assert reachability_grid[start_point.y, start_point.x] == 1

    points_of_iterst = []
    neighbors = [Point(0, 1), Point(0, -1), Point(1, 0), Point(-1, 0),
            Point(1, 1), Point(1, -1), Point(-1, 1), Point(-1, -1)]
    for direction in neighbors:
        new_point = start_point + direction
        if _check_point(new_point, reachability_grid):
            points_of_iterst.append(new_point)
    return points_of_iterst


def _check_point(point: Point, reachability_grid: np.ndarray):
    # we are inbound
    if point.x >= 0 and point.y >= 0 and point.y < reachability_grid.shape[0] and point.x < reachability_grid.shape[1]:
        if reachability_grid[point.y, point.x] == 0:
            return True
        if reachability_grid[point.y, point.x] == 3:
            # update from fatal to border
            reachability_grid[point.y, point.x] = 2
    return False
