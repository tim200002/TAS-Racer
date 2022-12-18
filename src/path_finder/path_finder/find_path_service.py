from rclpy.node import Node
from path_finder.utils.numpy_conversion import occupancy_grid_to_numpy, numpy_to_occupancy_grid
from path_finder.utils.coordinate_conversion import world_frame_point_to_occupancy_grid_coordinates, occupancy_grid_coordinates_to_world_frame_point
from path_finder.utils.classify_points import classify_points
from path_finder.path_find_algorithms.a_star import a_star
from path_finder.utils.angle_to_quaternion import get_quaternion_from_euler
from geometry_msgs.msg import Pose, PoseStamped
import path_finder.shared_variables as shared_variables
import math
from nav_msgs.msg import OccupancyGrid, MapMetaData
import numpy as np
import yaml
import matplotlib.pyplot as plt
import copy

from custom_interfaces.srv import FindPath

LETHAL_THRESHOLD = 30


class FindPathService(Node):
    def __init__(self, simulate_with_loaded_costmap=False):
        super().__init__('FindPathService')

        if not simulate_with_loaded_costmap:
            self.subscription_grid = self.create_subscription(
                OccupancyGrid,
                '/global_costmap/costmap',
                self.listener_callback_occupancy_grid,
                10)
        else:
            # Load occupancy grid for testing purposes
            with open('/home/parallels/Desktop/Parallels_Shared/Home/ros2_tas_project/to_be_saved/occupancy_grid.npy', 'rb') as f:
                occupancy_grid_np = np.load(f)
            print(np.unique(occupancy_grid_np))
            info = MapMetaData()
            info.resolution = 0.05
            origin = Pose()
            origin.position.x = -24.950000000372533
            origin.position.y = -24.950000000372533
            info.origin = origin
            occuancy_grid = numpy_to_occupancy_grid(
                occupancy_grid_np, info=info)
            shared_variables.occupancy_grid = occuancy_grid

        self.srv = self.create_service(
            FindPath, "find_path", self.find_path_callback)

    def find_path_callback(self, request, response):
        start_pose: PoseStamped = request.start_pose
        goal_pose: PoseStamped = request.goal_pose

        # deepcopy to have no issues if occupancy gird gets updated midway
        occupancy_grid = copy.deepcopy(shared_variables.occupancy_grid)

        # get start point
        start_point = (start_pose.pose.position.x, start_pose.pose.position.y)
        goal_point = (goal_pose.pose.position.x, goal_pose.pose.position.y)

        # convert start points from world frame coordinates to pixels in the numpy array
        start_point_np = world_frame_point_to_occupancy_grid_coordinates(
            start_point, occupancy_grid)
        goal_point_np = world_frame_point_to_occupancy_grid_coordinates(
            goal_point, occupancy_grid)

        self.get_logger().info(
            f'New Path finding request from start point {start_point} to goal point {goal_point} or in coordinates of the costmap start: {start_point_np}, goal_ {goal_point_np}')

        #! First version -> only support for A-star
        occupancy_grid_np = occupancy_grid_to_numpy(
            occupancy_grid)

        # binary split down occupancy grid only allow regions, where car would fir through in any pose
        # ToDo might improve later
        self.get_logger().info('Start Classifying points')
        occupancy_grid_np_binarized = (
            occupancy_grid_np > LETHAL_THRESHOLD) * 1

        classified_grid = classify_points(
            start_point_np, occupancy_grid_np_binarized)
        self.get_logger().info('Finished Classifying points')

        # show current position and classified grid
        plt.matshow(classified_grid)
        plt.scatter([start_point_np[1], goal_point_np[1]], [
                    start_point_np[0], goal_point_np[0]], c="red")
        plt.show()

        self.get_logger().info('Start A-star to calculate shortest path')
        path = a_star(start_point_np, goal_point_np, classified_grid)
        plt.matshow(classified_grid)
        plt.scatter([p[1] for p in path], [p[0] for p in path], s=0.5, c="red")
        plt.show()
        self.get_logger().info('Finished A-star to calculate shortest path')

        # back transform path to world frame coordinates
        path_world_coordinates = list(map(lambda point: occupancy_grid_coordinates_to_world_frame_point(
            point, occupancy_grid), path))

        # add poses, so that pose always shows towards next point
        path_world_coordinates_as_poses = [start_pose]
        for (prev, curr) in zip(path_world_coordinates[:-1], path_world_coordinates[1:]):
            vec_to_target = (curr[0]-prev[0], curr[1]-prev[1])
            angle = math.atan2(vec_to_target[1],  vec_to_target[0])
            [qx, qy, qz, qw] = get_quaternion_from_euler(0, 0, angle)

            newPose = PoseStamped()
            newPose.pose.position.x = curr[0]
            newPose.pose.position.y = curr[1]
            newPose.pose.position.z = 0.0
            newPose.pose.orientation.x = qx
            newPose.pose.orientation.y = qy
            newPose.pose.orientation.z = qz
            newPose.pose.orientation.w = qw
            path_world_coordinates_as_poses.append(newPose)

        path_world_coordinates_as_poses.append(goal_pose)

        self.get_logger().info('Finished Calculating path -> return')
        response.path_poses = path_world_coordinates_as_poses
        return response

    def listener_callback_occupancy_grid(self, msg: OccupancyGrid):
        self.get_logger().info('New global costmap received updating shared object')
        shared_variables.occupancy_grid = msg

        # store costmap for debugging purposes
        numpy_occupancy_grid = occupancy_grid_to_numpy(msg)
        np.save('/home/parallels/Desktop/Parallels_Shared/Home/ros2_tas_project/to_be_saved/occupancy_grid.npy',
                numpy_occupancy_grid.data)
        msg_info_yaml = {
            'resolution': msg.info.resolution,
            'width': msg.info.width,
            'height': msg.info.height,
            'map_load_time': msg.info.map_load_time,
            'origin_x': msg.info.origin.position.x,
            'origin_y': msg.info.origin.position.y
        }

        with open('/home/parallels/Desktop/Parallels_Shared/Home/ros2_tas_project/to_be_saved/occupancy_grid_info.yaml', 'w') as file:
            yaml.dump(msg_info_yaml, file)
