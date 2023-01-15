import numpy as np
from collections import deque


def extract_track_borders(classified_grid: np.ndarray):
    lines_grid = ((classified_grid == 2) * 1).copy()
    lines = []

    while True:
        reamining_line_points = np.argwhere(lines_grid == 1)
        if not len(reamining_line_points):
            break

        start_point = reamining_line_points[0]
        start_point = (start_point[1], start_point[0])
        line = _extract_line_segment(start_point, lines_grid)
        lines.append(line)

    return lines


def _extract_line_segment(start_point, lines_grid: np.ndarray):
    line = []
    points_to_explore = deque()
    points_to_explore.append(start_point)

    def check_new_point(point, lines_grid: np.ndarray):
        (x, y) = point
        # we are inbound
        if x >= 0 and y >= 0 and y < lines_grid.shape[0] and x < lines_grid.shape[1]:
            if lines_grid[y, x] == 1:
                return True
        return False

    while points_to_explore:
        # pop first element from stack and mark as belonging to a line
        (x, y) = points_to_explore.pop()
        line.append((x, y))
        lines_grid[y, x] = 0

        # iterate over all bordering points to continue line
        for x_dir in [-1, 0, 1]:
            for y_dir in [-1, 0, 1]:
                new_point = (x+x_dir, y+y_dir)
                if check_new_point(new_point, lines_grid):
                    points_to_explore.append(new_point)
    return line
