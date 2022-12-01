# Author: Addison Sears-Collins
# Date: September 19, 2021
# Description: Load a world file into Gazebo.
# https://automaticaddison.com
 
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
 
def generate_launch_description():
    
    # constants
    package_name = 'tas2-simulator'
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros') 
    pkg_share = FindPackageShare(package=package_name).find(package_name)

    # For TAS car
    urdf_file_path = os.path.join(pkg_share, 'models/urdf/tas_car.urdf')
    sdf_file_path = os.path.join(pkg_share, 'models/urdf/tas_car.sdf')
    rviz_config_file_path = os.path.join(pkg_share, 'rviz/navigationTAScar.rviz')

    #  For Gazebo to bringup world
    gazebo_models_path = 'models/gazeboModels'
    gazebo_world_file_path = 'models/worlds/cafe.world'
    gazebo_world_path = os.path.join(pkg_share, gazebo_world_file_path)
    gazebo_models_path = os.path.join(pkg_share, gazebo_models_path)
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    # Pose where we want to spawn the robot in Gazebo
    spawn_at_x = '-6.0'
    spawn_at_y = '-7.0'
    spawn_at_z = '0.0'
    spawn_at_yaw = '1.57'

    # Launch configuration variables specific to simulation
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
    world = LaunchConfiguration('world')

    declare_simulator_cmd = DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='Whether to execute gzclient')
        
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_use_simulator_cmd = DeclareLaunchArgument(
        name='use_simulator',
        default_value='True',
        description='Whether to start the simulator')
    
    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=gazebo_world_path,
        description='Full path to the world model file to load')

    # Specify the actions
    
    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        condition=IfCondition(use_simulator),
        launch_arguments={'world': world}.items())
    
    # Start Gazebo client    
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))
    
    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_world_cmd)
    
    # Launch nodes actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    
    return ld