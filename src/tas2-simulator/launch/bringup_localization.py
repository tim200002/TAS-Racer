# ===================================================================================
# Description: Launch file to visualize TAS car in Gazebo and test the car movement
#               by "rqt_robot_steering".
# Author: Salman Bari
# Date: Oct 29, 2022
# ===================================================================================
import os
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import launch_ros


def generate_launch_description():

    subprocess.run(['killall', 'gzserver'])

   # Constants for paths to different files and folders
    package_name = 'tas2-simulator'
    gazebo_models_path = 'models/gazeboModels'
    gazebo_world_file_path = 'models/worlds/gazebo_world.world'
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    rviz_config_file_path = os.path.join(
        pkg_share, 'rviz/navigationTAScar.rviz')
    gazebo_world_path = os.path.join(pkg_share, gazebo_world_file_path)
    gazebo_models_path = os.path.join(pkg_share, gazebo_models_path)
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
    urdf_file_path = os.path.join(pkg_share, 'models/urdf/turtlebot.urdf')

    # Launch configuration variables specific to simulation
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    urdf = open(urdf_file_path).read()

    # Declare the launch arguments


    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')


   

    start_localizer = Node(
        package='localization',
        executable='localizer',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
            }
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(start_localizer)
    return ld
