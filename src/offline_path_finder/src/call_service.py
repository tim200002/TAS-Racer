from gazebo_msgs.srv import SetEntityState, GetEntityState
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
            assert len(parts) == 6
            pose = my_Pose(my_Point(float(parts[0]), float(parts[1])), my_Quaternion(float(parts[2]), float(parts[3]), float(parts[4]), float(parts[5])))
            trajectory.append(pose)

    # define start pose
    start_pose: my_Pose = trajectory[0]
    # start_pose_ros = Pose()
    # start_pose_ros.position.x = start_pose.coordinate.x
    # start_pose_ros.position.y = start_pose.coordinate.y
    # start_pose_ros.position.z = 0.0
    # qx, qy, qz, qw = get_quaternion_from_euler(0,0, start_pose.angle)
    # start_pose_ros.orientation.x = qx
    # start_pose_ros.orientation.y = qy
    # start_pose_ros.orientation.z = qz
    # start_pose_ros.orientation.w = qw

    # define goal pose
    goal_pose: my_Pose = trajectory[-1]
    # goal_pose_ros = Pose()
    # goal_pose_ros.position.x = goal_pose.coordinate.x
    # goal_pose_ros.position.y = goal_pose.coordinate.y
    # goal_pose_ros.position.z = 0.0
    # qx, qy, qz, qw = get_quaternion_from_euler(0,0, goal_pose.angle)
    # goal_pose_ros.orientation.x = qx
    # goal_pose_ros.orientation.y = qy
    # goal_pose_ros.orientation.z = qz
    # goal_pose_ros.orientation.w = qw


    rclpy.init()
    node = Node("service")
    nav = BasicNavigator()

    car_state_world = gazebo_get_model_state(node, 'tas_car', reference_frame="")
    print("car_state_world car")
    print(car_state_world)

    translational_offset = my_Point(start_pose.coordinate.x - car_state_world.state.pose.position.x, start_pose.coordinate.y - car_state_world.state.pose.position.y)

    target = start_pose.quaternion
    start = my_Quaternion.from_ros_message(car_state_world.state.pose.orientation)
    rotational_offset =  my_Quaternion.multiply(target, start.inv())
    rotational_transformation = my_Quaternion.multiply(rotational_offset, start)
    
    # convert back to car fram
    rotation_car = my_Quaternion.multiply(rotational_transformation, start.inv())

    translation = my_Pose(translational_offset, rotation_car)
    print("trasnaltion")
    print(translation.to_ros_message())

    gazebo_set_model_state(node, 'tas_car', translation.to_ros_message(), reference_frame="base_footprint")
    python_time.sleep(3)
    rviz_set_initial_state(nav,node, start_pose.to_ros_message())
    python_time.sleep(3)
    start_navigation(nav, node, goal_pose.to_ros_message())



if __name__ == "__main__":
    main()



