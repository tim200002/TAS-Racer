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
    gazebo_world_file_path = 'models/worlds/world_track1_no_obstacles.world'
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    rviz_config_file_path = os.path.join(
        pkg_share, 'rviz/navigationTAScar.rviz')
    gazebo_world_path = os.path.join(pkg_share, gazebo_world_file_path)
    gazebo_models_path = os.path.join(pkg_share, gazebo_models_path)
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
    urdf_file_path = os.path.join(pkg_share, 'models/urdf/tas_car.urdf')

    # Launch configuration variables specific to simulation
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    urdf = open(urdf_file_path).read()

    # Declare the launch arguments
    declare_simulator_cmd = DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='Whether to execute gzclient and rviz')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=rviz_config_file_path,
        description='Full path to the RVIZ config file to use')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=gazebo_world_path,
        description='Full path to the world model file to load')

    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model',
        default_value=urdf_file_path,
        description='Absolute path to robot urdf file')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world
        }.items()
    )

    # Start Gazebo client
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(PythonExpression(['not ', headless])))

    # Launch RViz
    start_rviz_cmd = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file_path],
        parameters=[{'use_sim_time': use_sim_time, '__log_level': "error"}],
        condition=IfCondition(PythonExpression(['not ', headless]))
    )

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
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)

    # launch the nodes
    ld.add_action(start_localizer)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_rviz_cmd)

    return ld
