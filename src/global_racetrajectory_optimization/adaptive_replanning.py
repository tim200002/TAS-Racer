
import sys, os
base_path = os.path.split(os.path.join(os.path.dirname(os.path.abspath(__file__))))[0]
sys.path.append(base_path)


from global_helpers.models.map import Map
from global_helpers.models.point import Point
import yaml
import numpy as np
import math
from scipy import ndimage
from helper_funcs_glob.src.a_star import AStar

class DynamicRouteReplanner:
    def __init__(self, ):
        self.lookadhead_distance_m = 5
        self.merge_back_distance_m = 5
        self.margin_pixels = 8

    def init(self, reference_path_meters:list[Point], reference_path_pixels: list[Point]):
        self.reference_path_meters = reference_path_meters
        self.reference_path_pixels = reference_path_pixels
        # true when currenlty following path to avoid collisiosn
        self.isAvoidingCollision = False
        self.collision_avoidance_path_pixels = None

        self.idx_of_next_point_must_be_larger_than = -1


    def _find_closest_point_idx_on_path(self, path: list[Point], reference_point: Point, idx_must_be_larger_than = -1):
        idx_must_be_larger_than = -1
        closest_distance = np.inf
        closest_point_idx = None
        for i, point in enumerate(path):
            distance = math.sqrt((point.x - reference_point.x)**2 + (point.y - reference_point.y)**2)
            if(distance < closest_distance and i > idx_must_be_larger_than):
                closest_distance = distance
                closest_point_idx = i
        return closest_point_idx

    def _extend_path_meters_into_future(self, path_meters: list[Point], path_pixels: list[Point], start_idx: int, extension_distance_meters: float):
        current_pos_meters=path_meters[start_idx]
        current_pos_pixels=path_pixels[start_idx]
        running_distance = 0

        for i, point_meter in enumerate(path_meters[start_idx:]):
            if i == 0:
                extended_path_meters = [current_pos_meters]
                extended_path_pixels = [current_pos_pixels]
                continue

            distance = math.sqrt((point_meter.x - extended_path_meters[-1].x)**2 + (point_meter.y - extended_path_meters[-1].y)**2)
            running_distance += distance
            if(running_distance > extension_distance_meters):
                break
            extended_path_meters.append(point_meter)
            extended_path_pixels.append(path_pixels[start_idx + i])
        
        return extended_path_meters, extended_path_pixels

    def _check_path_for_collision(self, path: list[Point], min_margin: float, distance_grid: np.ndarray):
        has_collided = False
        collision_idx = None

        for i, point in enumerate(path):
            if(distance_grid[point.y, point.x] < min_margin):
                has_collided = True
                collision_idx = i
                break
        return has_collided, collision_idx

    def run_step(self, current_pos_meter: Point, occupancy_map: Map):
        # relevant preprocessing
        distance_to_objects_pixels = ndimage.distance_transform_edt(occupancy_map.grid != 2)

        if not self.isAvoidingCollision:
            print("Not Avoiding Collision")
            # Normal procedure check if reference trajectory is valid for next meters
            # otherwise calculate new trajectory
            closest_point_idx = self._find_closest_point_idx_on_path(self.reference_path_meters, current_pos_meter, self.idx_of_next_point_must_be_larger_than)
            print(f"closest pint id {closest_point_idx}")
            closest_point_pixels = self.reference_path_pixels[closest_point_idx]
            _, lookahead_path_pixels = self._extend_path_meters_into_future(self.reference_path_meters, self.reference_path_pixels, closest_point_idx, self.lookadhead_distance_m)


            has_collided, collision_idx = self._check_path_for_collision(lookahead_path_pixels, self.margin_pixels, distance_to_objects_pixels)
            if not has_collided:
                print("reference path is good, return")
                self.idx_of_next_point_must_be_larger_than = closest_point_idx
                return lookahead_path_pixels, None

            print("Reference path not good, calculate collision avoidance path")
            collision_idx = collision_idx + closest_point_idx

            # find closest point after object that is far enough away from object
            idx_after_initial_collision = collision_idx + 1

            first_valid_idx = None
            for i, point in enumerate(self.reference_path_pixels[idx_after_initial_collision:]):
                distance_pixels = distance_to_objects_pixels[point.y, point.x]
                if distance_pixels > self.margin_pixels:
                    first_valid_idx = idx_after_initial_collision + i
                    break
            
            _, extension_path_pxiels = self._extend_path_meters_into_future(self.reference_path_meters, self.reference_path_pixels, first_valid_idx, self.merge_back_distance_m)

            # plan new path using A star
            start_point = closest_point_pixels
            end_point = extension_path_pxiels[-1]

            distance_to_letal = ndimage.distance_transform_edt(occupancy_map.grid == 0)
            width_filterd_grid = (distance_to_letal < self.margin_pixels) * 1

            assert width_filterd_grid[start_point.y, start_point.x] == 0
            assert width_filterd_grid[end_point.y, end_point.x] == 0

            path_pixels = AStar.find_path(start_point, end_point, width_filterd_grid)[::10]

            # set to avoiding
            self.isAvoidingCollision = True
            self.collision_avoidance_path_pixels = path_pixels
            self.idx_of_next_point_must_be_larger_than = -1
            return path_pixels, 0
        
        else:
            print("Current avoiding collision")
            current_pos_pixels = occupancy_map.world_point_to_grid_point(current_pos_meter)
            # check if we reached end of collision avoidance maneauver
            distance_to_end_pixels = math.sqrt((self.collision_avoidance_path_pixels[-1].x - current_pos_pixels.x)**2 + (self.collision_avoidance_path_pixels[-1].y - current_pos_pixels.y)**2)
            print(f"distance to end {distance_to_end_pixels * occupancy_map.resolution}")
            delta_m = 0.5
            
            # reached end of collision avoidance
            if distance_to_end_pixels * occupancy_map.resolution < delta_m:
                print("reached end of collision avoidance phase")
                self.collision_avoidance_path_pixels = None
                self.isAvoidingCollision = False
                self.idx_of_next_point_must_be_larger_than = -1

                # regenerate new path
                return self.run_step(current_pos_meter, occupancy_map)
            
            # Take collision avoidance path and check if this path is still valid
            closest_point_idx = self._find_closest_point_idx_on_path(self.collision_avoidance_path_pixels, current_pos_pixels, idx_must_be_larger_than=0)
            has_collided, collision_idx = self._check_path_for_collision(self.collision_avoidance_path_pixels, self.margin_pixels, distance_to_objects_pixels)

            if not has_collided:
                print("Current avoidance path is good, keep it")
                return self.collision_avoidance_path_pixels, closest_point_idx
            
            print("Calculating new collision avoidance path")
            # there was a collision -> recalculate collision avoidance path
            start_point = self.collision_avoidance_path_pixels[collision_idx]
            end_point = self.collision_avoidance_path_pixels[-1]

            distance_to_letal = ndimage.distance_transform_edt(occupancy_map.grid == 0)
            width_filterd_grid = (distance_to_letal < self.margin_pixels) * 1

            assert width_filterd_grid[start_point.y, start_point.x] == 0
            assert width_filterd_grid[end_point.y, end_point.x] == 0

            path_pixels = AStar.find_path(start_point, end_point, width_filterd_grid)[::10]
            self.collision_avoidance_path_pixels = path_pixels
            return path_pixels, 0





def prepare():
    base_path ="../../out/tracks/track_4/"
     # Get Costmap
    with open(os.path.join(base_path, "costmap/map.npy"),'rb') as f:
        occupancy_grid = np.load(f)

    with (open(os.path.join(base_path, "costmap/map.yaml"), 'rb')) as f:
        occupancy_grid_config = yaml.safe_load(f)

        occupancy_grid = (occupancy_grid != 0) * 1

        resolution = occupancy_grid_config["resolution"]
        occupancy_map = Map(occupancy_grid, resolution, Point(occupancy_grid_config["origin_x"] - resolution, occupancy_grid_config["origin_y"] -resolution))

    path_meters = []
    path_pixels = []
    with open(os.path.join(base_path, "trajectory_mincurv.csv"), "r") as f:
        lines = f.readlines()
        for line in lines:
            # ignore comment lines
            if line.startswith('#'):
                continue
            parts = line.split(',')
            assert len(parts) == 3, line
            point = Point(float(parts[0]), float(parts[1]))
            path_meters.append(point)
            path_pixels.append(occupancy_map.world_point_to_grid_point(point))
    
    # Apply objects
    object = np.zeros_like(occupancy_grid)
    object[1085:1200, 780:850] = 2
    object[600:650, 1020:1200] = 2

    occupancy_grid_with_object = occupancy_map.grid + object
    occupancy_grid_with_object[occupancy_grid_with_object > 2] = 2
    occupancy_map.grid = occupancy_grid_with_object

    return occupancy_map, path_meters, path_pixels

def run(occupancy_map: Map, path_meters: list[Point], path_pixels: list[Point]):
    import numpy as np
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    dynamic_route_planner = DynamicRouteReplanner()
    dynamic_route_planner.init(path_meters, path_pixels)

    next_pos_m = path_meters[240]

    while True:
        new_path, current_idx = dynamic_route_planner.run_step(next_pos_m, occupancy_map)
        # current idx == NOne if we return completeyl new path
        if current_idx == None:
            current_pos_pixels = new_path[0]
            next_pos_m =  occupancy_map.grid_point_to_world_point(new_path[1])
        else:
            current_pos_pixels = new_path[current_idx]
            next_pos_m = occupancy_map.grid_point_to_world_point(new_path[current_idx + 1])
        
        

        
        plt.matshow(occupancy_map.grid)
        plt.scatter([point.x for point in new_path], [point.y for point in new_path], s=1)
        plt.scatter(current_pos_pixels.x, current_pos_pixels.y)
        plt.show()




def main():
    occupancy_map, path_meters, path_pixels = prepare()
    run(occupancy_map, path_meters, path_pixels)


if __name__ == "__main__":
    main()