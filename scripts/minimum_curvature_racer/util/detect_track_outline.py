def detect_track_outline(classified_grid: np.ndarray):
    lines_grid = ((classified_grid == 2) * 1).copy()

    lines = []

    while True:
        reamining_line_points = np.argwhere(lines_grid == 1)
        if not len(reamining_line_points):
            break

        start_point = reamining_line_points[0]
        start_point = (start_point[1], start_point[0])
        line = []
        _recursive_find_new_points_on_line(start_point, line, lines_grid)
        lines.append(line)

    return lines

def _recursive_find_new_points_on_line(point, line, lines_grid: np.ndarray):
    x,y = point[0], point[1]
    assert lines_grid[y,x] == 1

    # Append pont to line
    line.append(point)

    # mark point as already belonging to line
    lines_grid[y, x] = 0

    # Step 2 -> go in every direction and start again
    # defin helper function
    def check_new_point(x, y, reachability_grid: np.ndarray):
        # we are inbound
        if x>=0 and y>= 0 and y < reachability_grid.shape[0] and x < reachability_grid.shape[1]:
            if lines_grid[y,x] == 1:
                return True
        return  False

    for x_dir in [-1, 0, 1]:
        for y_dir in [-1, 0, 1]:
            if check_new_point(x+x_dir, y+y_dir, lines_grid):
                _recursive_find_new_points_on_line((x+x_dir, y+y_dir), line, lines_grid)