# ===================================================================================
# Description: Launch file to visualize TAS car in Gazebo and test the car movement
#               by "rqt_robot_steering".
# Author: Salman Bari
# Date: Oct 29, 2022
# ===================================================================================
from launch import LaunchDescription
from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess


def generate_launch_description():
 # Constants for paths to different files and folders
    robot_name_in_model = 'tas_car'

    # delete robot must be done before spwaning new one
    delete_entity_cmd = ExecuteProcess(
        cmd=[
            [FindExecutable(name='ros2'),
             " service call ",
             "/delete_entity gazebo_msgs/DeleteEntity ",
             '"{name: ' + robot_name_in_model + '}"']],
        shell=True
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    # launch the nodes
    ld.add_action(delete_entity_cmd)
    return ld
