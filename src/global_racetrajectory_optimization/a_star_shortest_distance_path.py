import sys, os
base_path = os.path.split(os.path.join(os.path.dirname(os.path.abspath(__file__))))[0]
sys.path.append(base_path)


from global_helpers.models.map import Map
from global_helpers.models.point import Point
from global_helpers.functions.export import export_trajcetory
from global_helpers.functions.trajectory_from_path import trajectory_from_path
from scipy import ndimage
from argparse import ArgumentParser
import numpy as np
import math
import heapq
import yaml


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
            path.append(current)
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

def main():
    parser = ArgumentParser()
    parser.add_argument("-p", "--base-path", default="../../out/tracks/track_4")
    parser.add_argument("-w", "--width", default=1)
    parser.add_argument("--start", type=int, nargs=2, default=[400, 100])
    parser.add_argument("--end", type=int, nargs=2, default=[160, 1200])
    args = parser.parse_args()

    base_path = args.base_path
    car_width_meters = args.width
    start_point = Point(args.start[0], args.start[1])
    end_point = Point(args.end[0], args.end[1])

    with open(os.path.join(base_path, "costmap/map.npy"),'rb') as f:
        occupancy_grid = np.load(f)

    with (open(os.path.join(base_path, "costmap/map.yaml"), 'rb')) as f:
        occupancy_grid_config = yaml.safe_load(f)

        occupancy_grid = (occupancy_grid != 0) * 1

        resolution = occupancy_grid_config["resolution"]
        occupancy_map = Map(occupancy_grid, resolution, Point(occupancy_grid_config["origin_x"] - resolution, occupancy_grid_config["origin_y"] -resolution))


    # transform map in a way,that each point has minimum distance too wall
    # i.e. car should drive straigt so distance should be half ot the width
    car_with_pixels = math.ceil(car_width_meters  / occupancy_map.resolution)
    margin_pixels = math.ceil(car_with_pixels / 2)

    distance_to_letal = ndimage.distance_transform_edt(occupancy_map.grid == 0)
    width_filtered_map = (distance_to_letal < margin_pixels)*1

    assert width_filtered_map[start_point.y, start_point.x] == 0
    assert width_filtered_map[end_point.y, end_point.x] == 0

    path = AStar.find_path(start_point, end_point, width_filtered_map)

    trajectory = trajectory_from_path(path, occupancy_map)
    export_trajcetory(trajectory, os.path.join(base_path, "trajectory_astar.csv"))




if __name__ == "__main__":
    main()