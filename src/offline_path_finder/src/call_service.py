from gazebo_msgs.srv import SetEntityState, GetEntityState
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseWithCovariance, PoseStamped
import time as python_time
from models.pose import Pose as my_Pose
from models.point import Point as my_Point
from utils.angle_to_quaternion import get_quaternion_from_euler
from nav2_simple_commander.robot_navigator import BasicNavigator

import rclpy
from rclpy.node import Node

# some global config
GLOBAL_FRAME = "map"
CAR_NAME = "tas_car"

# load trajectory and extract points
trajectory_file_path = "/home/tim/tas2-racer/to_be_saved/trajectory.txt"
trajectory = []
with open(trajectory_file_path, "r") as f:
    lines = f.readlines()
    for line in lines:
        parts = line.split(',')
        assert len(parts) == 3
        pose = my_Pose(my_Point(float(parts[0]), float(parts[1])), float(parts[2]))
        trajectory.append(pose)

# define start pose
start_pose: my_Pose = trajectory[0]
start_pose_ros = Pose()
start_pose_ros.position.x = start_pose.coordinate.x
start_pose_ros.position.y = start_pose.coordinate.y
start_pose_ros.position.z = 0.0
qx, qy, qz, qw = get_quaternion_from_euler(0,0, start_pose.angle)
start_pose_ros.orientation.x = qx
start_pose_ros.orientation.y = qy
start_pose_ros.orientation.z = qz
start_pose_ros.orientation.w = qw

# define goal pose
goal_pose: my_Pose = trajectory[-1]
goal_pose_ros = Pose()
goal_pose_ros.position.x = goal_pose.coordinate.x
goal_pose_ros.position.y = goal_pose.coordinate.y
goal_pose_ros.position.z = 0.0
qx, qy, qz, qw = get_quaternion_from_euler(0,0, goal_pose.angle)
goal_pose_ros.orientation.x = qx
goal_pose_ros.orientation.y = qy
goal_pose_ros.orientation.z = qz
goal_pose_ros.orientation.w = qw


rclpy.init()
node = Node("service")

# # get current state of tas car------------------------------------------------------
# print("start get current state of car in gazebo")
# get_entity_state_client = node.create_client(GetEntityState, "/gazebo/get_entity_state")
# while not get_entity_state_client.wait_for_service(timeout_sec=1.0):
#         print("Wait for service to become available")
# get_entity_state_request = GetEntityState.Request()
# get_entity_state_request.name = CAR_NAME
# get_entity_state_result = get_entity_state_client.call_async(get_entity_state_request)
# rclpy.spin_until_future_complete(node, get_entity_state_result)
# original_state = get_entity_state_result.result()
# print("end get current state of car in gazebo")

# # set gazebo to position--------------------------------------------------------------
# print("start set gazebo to position")
# set_entity_state_request = SetEntityState.Request()
# set_entity_state_request.state.name = CAR_NAME
# set_entity_state_request.state.pose = start_pose_ros
# set_entity_state_request.state.twist = original_state.state.twist

# set_entity_state_client = node.create_client(SetEntityState, "/gazebo/set_entity_state")
# while not set_entity_state_client.wait_for_service(timeout_sec=1.0):
#         print("Wait for service to become available")
# set_entity_state_result = set_entity_state_client.call_async(set_entity_state_request)
# rclpy.spin_until_future_complete(node, set_entity_state_result)
# print("end set gazebo to position")
# # set rviz to position------------------------------------------------------------------
# print("start set rviz to position")
# initialpose_publisher = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

# goal_pose_with_covariance_stamped = PoseWithCovarianceStamped()
# # set pose with covariance
# goal_pose_with_covariance = PoseWithCovariance()
# goal_pose_with_covariance.pose = start_pose_ros
# # this covariance I extracted from a message, it is the only one that works
# goal_pose_with_covariance.covariance[0] = 0.25
# goal_pose_with_covariance.covariance[7] = 0.25
# goal_pose_with_covariance.covariance[-1] = 0.06853891909122467
# goal_pose_with_covariance_stamped.pose = goal_pose_with_covariance
# # set stamp
# header = Header()
# header.frame_id = GLOBAL_FRAME
# time = Time()
# (seconds, nanosencods) = node.get_clock().now().seconds_nanoseconds()
# time.sec = seconds
# time.nanosec = nanosencods
# header.stamp = time
# goal_pose_with_covariance_stamped.header = header

# # spin service
# while(initialpose_publisher.get_subscription_count() < 1):
#     print("wait for subscription")
#     python_time.sleep(0.25)
# initialpose_publisher.publish(goal_pose_with_covariance_stamped)
# print("end set rviz to position")

nav = BasicNavigator()

nav.setInitialPose()

# set navigation goal -------------------------------------------------------------------------
print("start set goal pose")
goal_pose_publisher = node.create_publisher(PoseStamped , '/goal_pose', 10)

goal_pose_stamped = PoseStamped()
goal_pose_stamped.pose = goal_pose_ros
# set header
header = Header()
header.frame_id = GLOBAL_FRAME
time = Time()
(seconds, nanosencods) = node.get_clock().now().seconds_nanoseconds()
time.sec = seconds
time.nanosec = nanosencods
header.stamp = time
goal_pose_stamped.header = header

# spin service
while(goal_pose_publisher.get_subscription_count() < 1):
    print("wait for subscription")
    python_time.sleep(0.25)
goal_pose_publisher.publish(goal_pose_stamped)

print("end set goal pose")



