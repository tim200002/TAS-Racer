import numpy as np
from collections import deque
from models.point import Point


def extract_track_borders(classified_grid: np.ndarray) -> list[Point]:
    lines_grid = ((classified_grid == 2) * 1).copy()
    lines: list[Point] = []

    while True:
        reamining_line_points = np.argwhere(lines_grid == 1)
        if not len(reamining_line_points):
            break

        start_point = reamining_line_points[0]
        start_point =  Point(start_point[1], start_point[0])
        line = _extract_line_segment(start_point, lines_grid)
        lines.append(line)

    return lines


def _extract_line_segment(start_point: Point, lines_grid: np.ndarray):
    line = []
    points_to_explore = deque()
    points_to_explore.append(start_point)

    def check_new_point(point: Point, lines_grid: np.ndarray):
        # we are inbound
        if point.x >= 0 and point.y >= 0 and point.y < lines_grid.shape[0] and point.x < lines_grid.shape[1]:
            if lines_grid[point.y, point.x] == 1:
                return True
        return False

    while points_to_explore:
        # pop first element from stack and mark as belonging to a line
        current_point = points_to_explore.pop()
        line.append(current_point)
        lines_grid[current_point.y, current_point.x] = 0

        # iterate over all bordering points to continue line
        neighbors = [Point(0, 1), Point(0, -1), Point(1, 0), Point(-1, 0),
            Point(1, 1), Point(1, -1), Point(-1, 1), Point(-1, -1)]
        for direction in neighbors:
            new_point =current_point + direction
            if check_new_point(new_point, lines_grid):
                points_to_explore.append(new_point)
    return line
