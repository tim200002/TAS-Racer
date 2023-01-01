from gazebo_msgs.srv import SetEntityState, GetEntityState, DeleteEntity, SpawnEntity, GetModelList
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseWithCovariance, PoseStamped, Twist
import time as python_time
from models.pose import Pose as my_Pose
from models.point import Point as my_Point
from models.quaternion import Quaternion as my_Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from rclpy.node import Node
import copy

# some global config
CAR_NAME = "tas_car"

def gazebo_get_model_list(node):
    get_model_listclient = node.create_client(GetModelList, "/get_model_list")
    while not get_model_listclient.wait_for_service(timeout_sec=1.0):
            print("Wait for service to become available")
    get_model_list_request = GetModelList.Request()
    get_model_list_result = get_model_listclient.call_async(get_model_list_request)
    rclpy.spin_until_future_complete(node, get_model_list_result)
    return get_model_list_result.result().model_names


def gazebo_delete_entity(node, name):
    delete_entity_client = node.create_client(DeleteEntity, "/delete_entity")
    while not delete_entity_client.wait_for_service(timeout_sec=1.0):
            print("Wait for service to become available")
    delete_entity_request = DeleteEntity.Request()
    delete_entity_request.name = name
    delete_entity_result = delete_entity_client.call_async(delete_entity_request)
    rclpy.spin_until_future_complete(node, delete_entity_result)

def gazebo_spawn_entity(node, name, xml, initial_pose):
    spawn_entity_client = node.create_client(SpawnEntity, "/spawn_entity")
    while not spawn_entity_client.wait_for_service(timeout_sec=1.0):
            print("Wait for service to become available")
    spawn_entity_request = SpawnEntity.Request()
    spawn_entity_request.xml = xml
    spawn_entity_request.name = name
    spawn_entity_request.initial_pose = initial_pose
    spawn_entity_result = spawn_entity_client.call_async(spawn_entity_request)
    res = rclpy.spin_until_future_complete(node, spawn_entity_result)


def gazebo_get_model_state(node, name, reference_frame=""):
    # get current state of tas car------------------------------------------------------
    print("start get current state of car in gazebo")
    get_entity_state_client = node.create_client(GetEntityState, "/gazebo/get_entity_state")
    while not get_entity_state_client.wait_for_service(timeout_sec=1.0):
            print("Wait for service to become available")
    get_entity_state_request = GetEntityState.Request()
    get_entity_state_request.reference_frame = reference_frame
    get_entity_state_request.name = name
    get_entity_state_result = get_entity_state_client.call_async(get_entity_state_request)
    rclpy.spin_until_future_complete(node, get_entity_state_result)
    original_state = get_entity_state_result.result()
    print("end get current state of car in gazebo")
    return original_state
    
def gazebo_set_model_state(node, name, goal_pose, reference_frame=""):
    # set gazebo to position--------------------------------------------------------------
    print("start set gazebo to position")
    set_entity_state_request = SetEntityState.Request()
    set_entity_state_request.state.reference_frame = reference_frame
    set_entity_state_request.state.name = name
    set_entity_state_request.state.pose = goal_pose

    set_entity_state_client = node.create_client(SetEntityState, "/gazebo/set_entity_state")
    while not set_entity_state_client.wait_for_service(timeout_sec=1.0):
            print("Wait for service to become available")
    set_entity_state_result = set_entity_state_client.call_async(set_entity_state_request)
    rclpy.spin_until_future_complete(node, set_entity_state_result)
    print("end set gazebo to position")

    
def rviz_set_initial_state(nav, node, goal_pose, global_frame="map"):
    print("start set rviz to position")
    start_pose_stamped = PoseStamped()
    start_pose_stamped.pose = goal_pose

    # set stamp
    header = Header()
    header.frame_id = global_frame
    time = Time()
    (seconds, nanosencods) = node.get_clock().now().seconds_nanoseconds()
    time.sec = seconds
    time.nanosec = nanosencods
    header.stamp = time
    start_pose_stamped.header = header

    nav.setInitialPose(start_pose_stamped)

def start_navigation(nav, node, goal_pose, global_frame="map"):
    print("start set goal pose")

    goal_pose_stamped = PoseStamped()
    goal_pose_stamped.pose = goal_pose
    # set header
    header = Header()
    header.frame_id = global_frame
    time = Time()
    (seconds, nanosencods) = node.get_clock().now().seconds_nanoseconds()
    time.sec = seconds
    time.nanosec = nanosencods
    header.stamp = time
    goal_pose_stamped.header = header

    nav.goToPose(goal_pose_stamped)

    while not nav.isTaskComplete():
        print("navigating")
        python_time.sleep(1)
    #   feedback = nav.getFeedback()
    #   if feedback.navigation_time > 600:
    #     nav.cancelTask()

    print("end set goal pose")


def main():
    # load trajectory and extract points
    trajectory_file_path = "/home/tim/tas2-racer/to_be_saved/trajectory.txt"
    trajectory = []
    with open(trajectory_file_path, "r") as f:
        lines = f.readlines()
        for line in lines:
            parts = line.split(',')
            assert len(parts) == 6, line
            pose = my_Pose(my_Point(float(parts[0]), float(parts[1])), my_Quaternion(float(parts[2]), float(parts[3]), float(parts[4]), float(parts[5])))
            trajectory.append(pose)


    start_pose: my_Pose = trajectory[0]
    goal_pose: my_Pose = trajectory[-1]

    rclpy.init()
    node = Node("service")
    nav = BasicNavigator()

    name_of_car_in_simulation = "tas_car"

    # spawn car
    model_list = gazebo_get_model_list(node)
    if name_of_car_in_simulation in model_list:
        print("Model already exist, deleting first")
        gazebo_delete_entity(node, "tas_car")
    
    print("Spawning model in gazebo")
    xml = open("/home/tim/tas2-racer/src/tas2-simulator/models/urdf/tas_car.sdf").read()
    gazebo_spawn_entity(node, "tas_car",xml , start_pose.to_ros_message())
    python_time.sleep(4)
    
    print("setup RVIZ")
    rviz_set_initial_state(nav,node, start_pose.to_ros_message())
    python_time.sleep(3)
    print("start navigation")
    start_navigation(nav, node, goal_pose.to_ros_message())



if __name__ == "__main__":
    main()



