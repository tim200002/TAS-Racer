import sys, os
base_path = os.path.split(os.path.join(os.path.dirname(os.path.abspath(__file__))))[0]
sys.path.append(base_path)


from global_helpers.models.map import Map
from global_helpers.models.point import Point, RefPoint
from global_helpers.functions.export import export_centerline, export_trajcetory
from global_helpers.functions.trajectory_from_path import trajectory_from_path
from skimage import measure
from scipy import ndimage
from skimage.segmentation import watershed
from skimage.feature import peak_local_max
import sys
from argparse import ArgumentParser
import numpy as np
import os
import yaml

def main():
    parser = ArgumentParser()
    parser.add_argument("-p", "--base-path", default="../../out/tracks/track_4")
    args = parser.parse_args()
    base_path = args.base_path

    with open(os.path.join(base_path, "costmap/map.npy"),'rb') as f:
        occupancy_grid = np.load(f)

    with (open(os.path.join(base_path, "costmap/map.yaml"), 'rb')) as f:
        occupancy_grid_config = yaml.safe_load(f)

        occupancy_grid = (occupancy_grid != 0) * 1

        resolution = occupancy_grid_config["resolution"]
        occupancy_map = Map(occupancy_grid, resolution, Point(occupancy_grid_config["origin_x"] - resolution, occupancy_grid_config["origin_y"] -resolution))

    track_contours = measure.find_contours(occupancy_map.grid, 0)
    assert len(track_contours) == 2

    # find track contours
    for contour in track_contours:
        y_coord = np.empty(contour.shape[0], dtype=int)
        x_coord = np.empty(contour.shape[0],  dtype=int)
        for i, element in enumerate(contour):
            y_coord[i] = element[0]
            x_coord[i] = element[1]
        occupancy_map.grid[y_coord, x_coord] = 2

    # generate distance map, distance is distance to track contours
    binarized_grid = occupancy_map.grid != 2
    distances = ndimage.distance_transform_edt(binarized_grid)

    # user watershed algorithm to find centerline
    coords = peak_local_max(-distances, footprint=np.ones((3, 3)), labels=occupancy_map.grid)
    mask = np.zeros(distances.shape, dtype=bool)
    mask[tuple(coords.T)] = True
    markers, _ = ndimage.label(mask)
    labels = watershed(distances, markers)

    # now use contour to extract centerline as path
    center_line = measure.find_contours(labels, 1)
    assert len(center_line) == 1
    center_line = center_line[0]

    # generate centerline format (point cooridnates and distance to centerline)
    distace_to_track_border = ndimage.distance_transform_edt(occupancy_map.grid == 0)

    path = []
    for coord in center_line:
        y,x = int(coord[0]), int(coord[1])
        distance = distace_to_track_border[y,x]
        path.append(RefPoint(x, y, distance, distance))

    # store centerline
    export_centerline(path, occupancy_map, os.path.join(base_path, "center_line.csv"))

    trajectory = trajectory_from_path(path, occupancy_map)
    export_trajcetory(trajectory, os.path.join(base_path, "trajectory_center.csv"))


if __name__ == "__main__":
    main()