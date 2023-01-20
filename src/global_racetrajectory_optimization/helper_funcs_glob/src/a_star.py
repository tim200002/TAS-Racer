import sys, os
#base_path = os.path.split(os.path.split(os.path.dirname(os.path.abspath(__file__)))[0])[0]
base_path = "/home/tim/tas2-racer/src"
sys.path.append(base_path)

from global_helpers.models.point import Point
import math
import heapq
import numpy as np

class Node:
    def __init__(self, coordinate: Point, f_score):
        self.coordinate = coordinate
        self.f_score = f_score

    def __lt__(self, other):
        return self.f_score < other.f_score



class AStar:
    @classmethod
    def find_path(cls, start: Point, goal: Point, classified_grid: np.ndarray) -> list[Point]:
        """
        Finds shortest path between two points using a-star

        classified_grid must be of such form that all valid points have value 0 and all invalid points have value 1
        """
        # General initializazion
        assert classified_grid[start.y, start.x] == 0 and classified_grid[goal.y, goal.x] == 0

        # allow moves in all directions
        neighbors = [Point(0, 1), Point(0, -1), Point(1, 0), Point(-1, 0),
        Point(1, 1), Point(1, -1), Point(-1, 1), Point(-1, -1)]

        
        open_heap = []
        closed_set = set()

        came_from = {}

        # add initial point to open list
        gscore = {start: 0}
        fscore = {start: cls._heuristic(start, goal)}

        heapq.heappush(open_heap, Node(start, fscore[start]))

        while open_heap:
            current_point = heapq.heappop(open_heap).coordinate

            # reached goal -> backwards calculate path
            if current_point == goal:
                print("backsolve")
                return cls._backsolve_path(current_point, came_from)

            # Continue path finding
            closed_set.add(current_point)

            # go over all neighbors
            for direction in neighbors:
                neighbor = current_point + direction

                tentative_g_score = gscore[current_point] + math.sqrt(direction.x**2 + direction.y**2)

                if not cls._check_new_point(neighbor, classified_grid):
                    continue

                if neighbor in closed_set: 
                    continue

                if tentative_g_score < gscore.get(neighbor, float('inf')):
                    came_from[neighbor] = current_point
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + \
                        cls._heuristic(neighbor, goal)

                    heapq.heappush(open_heap, Node(neighbor, fscore[neighbor]))
        
        raise ValueError("No path exists between pints")



    @classmethod
    def _heuristic(cls, a: Point, b: Point):
        return np.sqrt((b.x - a.x) ** 2 + (b.y - a.y) ** 2)

    @classmethod
    def _backsolve_path(cls, end: Node, came_from):
        current = end
        path = []
        while current in came_from:
            path.insert(0, current)
            current = came_from[current]
        return path

    @classmethod
    def _check_new_point(cls, point: Point, classified_grid: np.ndarray):
        if point.x < 0 or point.x > classified_grid.shape[1]:
            return False
        if point.y < 0 or point.y > classified_grid.shape[0]:
            return False
        if classified_grid[point.y, point.x] == 1:
            return False

        return True