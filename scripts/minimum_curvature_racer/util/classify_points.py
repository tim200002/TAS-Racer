import numpy as np

def classify_points(x_start, y_start, binary_occupancy_grid: np.ndarray):
    """"Classifies all points in map into categories depending on start point
    Categories:
    0) Non reachable
    1) reachable
    2) belonging to line bordering (is also fatal but extra class)
    3) fatal
    """
    
    # convert all fatal points to value 3
    reachability_grid = binary_occupancy_grid.copy() * 3

    # check start point is valid
    assert x_start >=0 and y_start >= 0 and y_start < reachability_grid.shape[0] and x_start < reachability_grid.shape[1]
    assert reachability_grid[y_start, x_start] != 2

    # start recursiv fnding
    recursive_find_new_points(x_start, y_start, reachability_grid)

    return reachability_grid


def recursive_find_new_points(x,y, reachability_grid: np.ndarray):
    assert reachability_grid[y,x] == 0

    # Step 1 -> set current point as reachable
    reachability_grid[y, x] = 1

    # Step 2 -> go in every direction and start again
    # defin helper function
    def check_new_point(x, y, reachability_grid: np.ndarray):
        # we are inbound
        if x>=0 and y>= 0 and y < reachability_grid.shape[0] and x < reachability_grid.shape[1]:
            if reachability_grid[y,x] == 0:
                return True
            if reachability_grid[y,x] == 3:
                # update from fatal to border
                reachability_grid[y, x] = 2
        return  False

    for x_dir in [-1, 0, 1]:
        for y_dir in [-1, 0, 1]:
            if check_new_point(x+x_dir, y+y_dir, reachability_grid):
                recursive_find_new_points(x+x_dir, y+y_dir, reachability_grid)