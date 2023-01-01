# ===================================================================================
# Description: Launch file to visualize TAS car in Gazebo and test the car movement
#               by "rqt_robot_steering".
# Author: Salman Bari
# Date: Oct 29, 2022
# ===================================================================================
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable, LocalSubstitution
from launch.actions import ExecuteProcess
from launch.actions import TimerAction
from launch.event_handlers import OnShutdown, OnProcessExit, OnExecutionComplete


def generate_launch_description():
 # Constants for paths to different files and folders
    package_name = 'tas2-simulator'
    robot_name_in_model = 'tas_car'
    gazebo_models_path = 'models/gazeboModels'
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    urdf_file_path = os.path.join(pkg_share, 'models/urdf/tas_car.urdf')
    sdf_file_path = os.path.join(pkg_share, 'models/urdf/tas_car.sdf')
    gazebo_models_path = os.path.join(pkg_share, gazebo_models_path)
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

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

    # delete robot must be done before spwaning new one
    delete_entity_cmd = ExecuteProcess(
        cmd=[
            [FindExecutable(name='ros2'),
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

    shutdown = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                delete_entity_cmd,
                LogInfo(
                    msg=['Launch was asked to shutdown: ',
                         LocalSubstitution('event.reason')])
            ]
        )
    )

    wait_for_delete = RegisterEventHandler(
        OnExecutionComplete(
            target_action=delete_entity_cmd,
            on_completion=[
                LogInfo(msg='Spawn finished'),
            ]
        )
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # launch the nodes
    # ld.add_action(delete_entity_cmd)
    ld.add_action(spawn_entity_cmd)
    # ld.add_action(start_robot_state_publisher_cmd)
    # ld.add_action(shutdown)
    # ld.add_action(wait_for_delete)

    return ld
