
from rclpy.node import Node
from custom_interfaces.srv import FindPath
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose as rosPose 
from geometry_msgs.msg import PoseStamped as rosPoseStamped

from global_helpers.models.map import Map 
from global_helpers.models.point import Point
from global_helpers.models.pose import Pose
import numpy as np

class PathFinder(Node):
    def __init__(self):
        print("setup")
        super().__init__('PathFinder')

        # relevant preprocessing
        self.path_meters, self.trajectory_meters = self._load_reference_trajectory("/home/tim/tas2-racer/out/tracks/track_3/trajectory_mincurv.csv")
        
        self.srv = self.create_service(
            FindPath, "find_path", self.super_simple_find_path_callback)
        
        # some expected values
        self.expected_resolution = 0.05

        
    
    
    def _load_reference_trajectory(self, path: str):
        path_meters = []
        trajectory_meters: list[Pose] = []
        with open(path, "r") as f:
            lines = f.readlines()
            for line in lines:
                # ignore comment lines
                if line.startswith('#'):
                    continue
                parts = line.split(',')
                assert len(parts) == 3, line
                point = Point(float(parts[0]), float(parts[1]))
                path_meters.append(point)
                pose = Pose(Point(float(parts[0]), float(parts[1])), float(parts[2]))
                trajectory_meters.append(pose)
        return path_meters, trajectory_meters
    
    
    def super_simple_find_path_callback(self, request, response):
        print("super simple callback")
        start_pose: rosPose = request.start_pose
        goal_pose: rosPose = request.goal_pose

        start_point: Point = Point(start_pose.position.x, start_pose.position.y)
        goal_point: Point = Point(goal_pose.position.x, goal_pose.position.y)
        
        def is_point_near_trajectory(point: Point, trajectory: list[Pose]):
            thresh = 1

            for pose_idx, pose in enumerate(trajectory):
                point_differece = point - pose.coordinate

                if(point_differece.norm() < thresh):
                    return True, pose_idx
            
            return False, None
        
        # check start pose is clear
        nearby, start_idx = is_point_near_trajectory(start_point, self.trajectory_meters)
        if not nearby:
            print("Handle error")
            return
        
        trajectory_filtered = self.trajectory_meters[start_idx:]
    
        nearby,_ = is_point_near_trajectory(goal_point, trajectory_filtered)
        if not nearby:
            print("Handl error")
            return

        # Convert to ros poses
        trajectory_filtered_ros: list[rosPose] = list(map(lambda pose: pose.to_ros_message(), trajectory_filtered))
        response.path_poses = trajectory_filtered_ros
        print(trajectory_filtered_ros)
        return response



    def find_path_callback(self, request, response):
        costmap: OccupancyGrid = request.costmap
        start_pose: rosPoseStamped = request.start_pose
        goal_pose: rosPoseStamped = request.goal_pose

        # create Map type from message type
        resolution = costmap.info.resolution
        assert abs(resolution - self.expected_resolution) < 0.01

        origin = Point(costmap.info.origin.position.x, costmap.info.origin.position.y)
        costmap_data = np.array(costmap.data).resize(new_shape=(costmap.info.height, costmap.info.width))
        costmap_map = Map(costmap_data,resolution, origin)


