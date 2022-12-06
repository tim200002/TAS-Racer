# ===================================================================================
# Description: Launch file to visualize TAS car in Gazebo and test the car movement
#               by "rqt_robot_steering".
# Author: Salman Bari
# Date: Oct 29, 2022
# ===================================================================================
import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess
import time
from launch.actions import TimerAction



def generate_launch_description():
 # Constants for paths to different files and folders
  package_name = 'tas2-simulator'
  robot_name_in_model = 'tas_car'
  gazebo_models_path = 'models/gazeboModels' 
  pkg_share = FindPackageShare(package=package_name).find(package_name)
  urdf_file_path = os.path.join(pkg_share, 'models/urdf/tas_car.urdf')
  sdf_file_path = os.path.join(pkg_share, 'models/urdf/tas_car.sdf')
  gazebo_models_path = os.path.join(pkg_share, gazebo_models_path)
  os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
  slam_file_path = os.path.join(pkg_share, 'config/slam.yaml')


  # Pose where we want to spawn the robot in Gazebo
  spawn_at_x = '0.0'
  spawn_at_y = '0.0'
  spawn_at_z = '0.0'
  spawn_at_yaw = '0.0'

  # Launch configuration variables specific to simulation
  use_sim_time = LaunchConfiguration('use_sim_time')
  urdf = open(urdf_file_path).read()


  # Declare the launch arguments
  declare_urdf_model_path_cmd = DeclareLaunchArgument(
    name='urdf_model', 
    default_value=urdf_file_path, 
    description='Absolute path to robot urdf file')
    
  declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    name='use_robot_state_pub',
    default_value='True',
    description='Whether to start the robot state publisher')
    
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='True',
    description='Use simulation (Gazebo) clock if true')

  
   # Subscribe to the joint states of the robot and publish.    
  start_robot_state_publisher_cmd = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    output='screen',
    parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'robot_description': urdf
                }
            ]
        )

  delete_entity_cmd = ExecuteProcess(
    cmd=[[FindExecutable(name='ros2'),
        " service call ",
        "/delete_entity gazebo_msgs/DeleteEntity ",
         '"{name: ' + robot_name_in_model + '}"']],
         shell=True
  )

  # Spawn the robot in Gazebo
  spawn_entity_cmd = Node(
     package='gazebo_ros', 
     executable='spawn_entity.py',
     name='spawn_entity',
     arguments=['-entity', robot_name_in_model, 
                 '-file', sdf_file_path,
                 '-x', spawn_at_x,
                  '-y', spawn_at_y,
                  '-z', spawn_at_z,
                  '-Y', spawn_at_yaw],
                     output='screen')

#   start_nav2_pkg = IncludeLaunchDescription(
#       PythonLaunchDescriptionSource(os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')),
#       launch_arguments={
#         'use_sim_time': use_sim_time,
#         'params_file': nav2_config_file_path,
#         'map': map_config_file_path,
#         'rviz_config_file': rviz_config_file_path
#       }.items())
#   slam_node = Node(
#     parameters=[
#         slam_file_path,
#         {"use_sim_time": use_sim_time}
#     ],
#     package="slam_toolbox",
#     executable="async_slam_toolbox_node",
#     name="slam_toolbox",
#     output="screen"
#   )

  # Create the launch description and populate
  ld = LaunchDescription()

  # Declare the launch options
  ld.add_action(declare_urdf_model_path_cmd)
  ld.add_action(declare_use_robot_state_pub_cmd)  
  ld.add_action(declare_use_sim_time_cmd)


    # launch the nodes
  # ld.add_action(start_gazebo_cmd)
  # ld.add_action(TimerAction(period=12.0, actions=[delete_entity_cmd, spawn_entity_cmd]))
  # ld.add_action(delete_entity_cmd)
  ld.add_action(spawn_entity_cmd)
  ld.add_action(start_robot_state_publisher_cmd)
  # ld.add_action(slam_node)
  return ld