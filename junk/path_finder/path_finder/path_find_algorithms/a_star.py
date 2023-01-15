import numpy as np
import heapq


def a_star(start, goal, classified_grid: np.ndarray):
    """
    Finds shortest path between two points using a-star

    classified_grid must be of such form that all valid points have value 1 and invalid points have a different value
    If we do not check fir unrechable points before there is the danger that algorithm cannot find point.
    """
    start = (start[1], start[0])
    goal = (goal[1], goal[0])

    assert classified_grid[start] == 1 and classified_grid[goal] == 1

    # allow moves in all directions
    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0),
                 (1, 1), (1, -1), (-1, 1), (-1, -1)]

    open_heap = []
    closed_set = set()

    came_from = {}

    # add initial point to open list
    gscore = {start: 0}
    fscore = {start: _heuristic(start, goal)}

    heapq.heappush(open_heap, (fscore[start], start))

    while open_heap:
        current = heapq.heappop(open_heap)[1]

        # reached goal -> backwards calculate path
        if current == goal:
            return _backsolve_path(current, came_from)

        # Continue path finding
        closed_set.add(current)

        # go over all neighbors
        for i, j in neighbors:
            neighbor = current[0]+i, current[1]+j

            tentative_g_score = gscore[current] + _heuristic(current, neighbor)

            if not _check_new_point((neighbor[1], neighbor[0]), classified_grid):
                continue

            if neighbor in closed_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue

            if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in open_heap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + \
                    _heuristic(neighbor, goal)

                heapq.heappush(open_heap, (fscore[neighbor], neighbor))


def _heuristic(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)


def _backsolve_path(end, came_from):
    print("backsolve_path")
    current = end
    path = []
    while current in came_from:
        path.append(current)
        current = came_from[current]
    return path


def _check_new_point(point, classified_grid: np.ndarray):
    if point[0] < 0 or point[0] > classified_grid.shape[1]:
        return False
    if point[1] < 0 or point[1] > classified_grid.shape[0]:
        return False
    if classified_grid[point[1], point[0]] != 1:
        return False

    return True
