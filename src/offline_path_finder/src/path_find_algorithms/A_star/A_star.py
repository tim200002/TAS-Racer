import numpy as np
import math

from models.point import Point
from .node import Node


class AStar:
    @classmethod
    def find_path(cls, start: Point, goal: Point, grid: np.ndarray):
        """
        Finds shortest path between two points using a-star

        classified_grid must be of such form that all valid points have value 1 and invalid points have a different value
        If we do not check fir unrechable points before there is the danger that algorithm cannot find point.
        """

        assert grid[start.y, start.x] == 1 and grid[goal.y, goal.x] == 1

        # allow moves in all directions
        neighbors = [Point(0, 1), Point(0, -1), Point(1, 0), Point(-1, 0),
                    Point(1, 1), Point(1, -1), Point(-1, 1), Point(-1, -1)]
        
        current: Node = None
        open_set = []
        closed_set = []

        open_set.append(Node(start))

        while open_set:
            current = open_set[0]

            # see if there is not a smaller node
            optimal_index = 0
            for i, node in enumerate(open_set):
                if node.get_score() < current.get_score():
                    optimal_index = i
                    current = node
            
            print(current.get_score())

            # reached goal -> backwards calculate path
            if current.coordinate == goal:
                return cls._backsolve_path(current)

            del open_set[optimal_index]
            
            # Add element to closed list and continue path finding
            closed_set.append(current)
            # print(f"found best way to point {current.coordinate}")

            # go over all neighbors
            for direction in neighbors:
                new_point = Node(current.coordinate + direction, current)

                # check if valid movement
                if not cls._check_new_point(new_point.coordinate, grid):
                    continue

                # check if value in closed list, if so continue
                was_found = False
                for node in closed_set:
                    if node.coordinate == new_point.coordinate:
                        was_found = True
                        break
                if was_found:
                    continue


                g = current.G + math.sqrt(direction.x**2 + direction.y**2)
                h = cls._heuristic(new_point.coordinate, goal)

                new_point.G = g
                new_point.H = h

                add_node = True
                for node in open_set:
                    if node.coordinate == new_point.coordinate:
                        # there is a new better way -> update node
                        if new_point.G < node.G:
                            node.G = new_point.G
                            node.parent = new_point.parent
                        add_node = False
                        break
                    
                if add_node:
                    open_set.append(new_point)



    @classmethod
    def _heuristic(cls, a: Point, b: Point):
        return np.sqrt((b.x - a.x) ** 2 + (b.y - a.y) ** 2)

    @classmethod
    def _backsolve_path(cls, end: Node):
        current = end
        path = []
        while current != None:
            path.append(current.coordinate)
            current = current.parent

    @classmethod
    def _check_new_point(cls, point: Point, classified_grid: np.ndarray):
        if point.x < 0 or point.x > classified_grid.shape[1]:
            return False
        if point.y < 0 or point.y > classified_grid.shape[0]:
            return False
        if classified_grid[point.y, point.x] != 1:
            return False

        return True