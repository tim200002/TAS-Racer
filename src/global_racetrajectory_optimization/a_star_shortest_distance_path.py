import sys, os
base_path = os.path.split(os.path.dirname(os.path.abspath(__file__)))[0]
sys.path.append(base_path)
import matplotlib.pyplot as plt


from global_helpers.models.map import Map
from global_helpers.models.point import Point
from global_helpers.functions.export import export_trajcetory
from global_helpers.functions.trajectory_from_path import trajectory_from_path
from scipy import ndimage
from argparse import ArgumentParser
import numpy as np
import yaml

from helper_funcs_glob.src.a_star import AStar


def main():
    parser = ArgumentParser()
    parser.add_argument("-p", "--base-path", default="../../out/tracks/demo")
    parser.add_argument("-w", "--width", default=2)

    parser.add_argument("--start", type=int, nargs=2, default=[290, 700])
    parser.add_argument("--end", type=int, nargs=2, default=[1100, 1000])
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

    # plt.matshow(occupancy_grid)
    # plt.scatter(start_point.x, start_point.y)
    # plt.scatter(end_point.x, end_point.y)
    # plt.show()

    # transform map in a way,that each point has minimum distance too wall
    # i.e. car should drive straigt so distance should be half ot the width
    margin_meters = car_width_meters / 2
    margin_pixels = margin_meters / occupancy_map.resolution

    distance_to_letal = ndimage.distance_transform_edt(occupancy_map.grid == 0)
    width_filtered_map = (distance_to_letal < margin_pixels)*1

    assert width_filtered_map[start_point.y, start_point.x] == 0
    assert width_filtered_map[end_point.y, end_point.x] == 0

    path = AStar.find_path(start_point, end_point, width_filtered_map)

    trajectory = trajectory_from_path(path, occupancy_map)
    export_trajcetory(trajectory, os.path.join(base_path, "trajectory_astar.csv"))

    plt.matshow(occupancy_map.grid)
    plt.scatter([coord.x for coord in path], [coord.y for coord in path], s=0.01)
    plt.show()




if __name__ == "__main__":
    main()