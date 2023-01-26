import sys, os
base_path = os.path.split(os.path.join(os.path.dirname(os.path.abspath(__file__))))[0]
sys.path.append(base_path)

from global_helpers.models.point import Point
from global_helpers.models.map import Map
import yaml
import matplotlib.pyplot as plt
import numpy as np


def read_trajectory(path: str, occupancy_map: Map):
    points = []
    with open(path,"r" ) as f:
        for line in f:
            if(line.startswith("#")):
                continue
            words = line.split(",")
            points.append(occupancy_map.world_point_to_grid_point(Point(float(words[0]), float(words[1]))))
    return points


def main():
    file_paths = ["../../out/tracks/demo/trajectory_mincurv.csv", "../../out/tracks/demo/trajectory_astar.csv", "../../out/tracks/demo/trajectory_shortest_path.csv"]

    map_base_path = "../../out/tracks/demo"
    with open(os.path.join(map_base_path, "costmap/map.npy"),'rb') as f:
        occupancy_grid = np.load(f)

    with (open(os.path.join(map_base_path, "costmap/map.yaml"), 'rb')) as f:
        occupancy_grid_config = yaml.safe_load(f)

        occupancy_grid = (occupancy_grid != 0) * 1

        resolution = occupancy_grid_config["resolution"]
        occupancy_map = Map(occupancy_grid, resolution, Point(occupancy_grid_config["origin_x"] - resolution, occupancy_grid_config["origin_y"] -resolution))
    
    paths = [read_trajectory(file_path, occupancy_map) for file_path in file_paths]

    plt.matshow(occupancy_map.grid)

    for path in paths:
        plt.plot([point.x for point in path], [point.y for point in path])
    
    plt.legend(["minimum curvature", "A*", "shortest path"])
    plt.show()


if __name__ == "__main__":
    main()