# ===================================================================================
# Description: Launch file to visualize TAS car in Gazebo and test the car movement
#               by "rqt_robot_steering".
# Author: Salman Bari
# Date: Oct 29, 2022
# ===================================================================================
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
 # Constants for paths to different files and folders
    package_name = 'tas2-simulator'
    gazebo_models_path = 'models/gazeboModels'
    pkg_nav2 = FindPackageShare(package='nav2_bringup').find('nav2_bringup')
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    rviz_config_file_path = os.path.join(
        pkg_share, 'rviz/navigationTAScar.rviz')
    gazebo_models_path = os.path.join(pkg_share, gazebo_models_path)
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
    nav2_config_file_path = os.path.join(pkg_share, 'config/navigation.yaml')
    map_config_file_path = os.path.join(pkg_share, 'maps/map.yaml')

    # Launch configuration variables specific to simulation
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    # path finder node
    path_finder_node = Node(
        package='path_finder',
        executable='path_finder_service',
        name='path_finder',
    )

    # robot localization via EKF
    robot_EKF_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml'),
                    {'use_sim_time': use_sim_time}],
        #  remappings=[("/cmd_vel", "cmd_vel"), ("/odom", "odom")]
    )

    start_nav2_pkg = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_nav2, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_config_file_path,
            'map': map_config_file_path,
            'rviz_config_file': rviz_config_file_path
        }.items())

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)

    # launch the nodes
    ld.add_action(path_finder_node)
    ld.add_action(robot_EKF_localization_node)
    ld.add_action(start_nav2_pkg)

    return ld
