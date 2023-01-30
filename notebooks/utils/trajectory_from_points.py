from models.point import Point
from models.pose import Pose
from models.map import Map
import math

def trajectory_from_path(path: list[Point], ros_map: Map):
    path_world_coordinates = list(map(lambda point: ros_map.grid_point_to_world_point(
    point), path))

    trajectory = []
    for (curr, next) in zip(path_world_coordinates[:-1], path_world_coordinates[1:]):
        vec_to_target = next - curr
        assert vec_to_target.norm() < 1, "Something is off points to far from one another"
        yaw = math.atan2(vec_to_target.y,  vec_to_target.x)
        trajectory.append(Pose.from_coordinate_and_yaw(coordinate=curr, yaw=yaw))
    
    return trajectory
