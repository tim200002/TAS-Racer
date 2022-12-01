# TAS2 simulator


## Installation Guide
The TAS2 simulator is designed using ROS2 and tested for Ubuntu 22.04.

Following are the steps to install the TAS2 simulator.

- Install ROS2. Check the guidelines [here](https://docs.ros.org/en/humble/index.html) for installation. 
- Install the colcon
  ```
  sudo apt install python3-colcon-common-extensions 
  ```
- create the workspace and src directories. [Further Information Here](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
  ```
  mkdir -p ~/ros2_ws/src
  ```
- Clone this repository to the 'src' directory
  ```
  cd ros2_ws/src
  git clone ""link to the repository here""

  ```
- The workspace should have the source code of the TAS2-simulator. Your workspace directory should look like this;
  ```
  ros2_ws
  │   build
  │   install    
  │   log
  └───src
      └───TAS2-simulator
          │   config
          │   launch
          |   maps
          |   models
          |   rviz
          |   CMakeLists.txt
          │   package.xml
  ```
-  Now build the workspace.
    ```
    cd ros2_ws
    colcon build --symlink-install
    ```

- Remember to source your workspace
  ```
    echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
    echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc
  ```
- Before trying the demo, you may need to install extra packages
  ```
    sudo apt install ros-humble-joint-state-publisher
    sudo apt install ros-humble-joint-state-publisher-gui
    sudo apt install ros-humble-xacro
    sudo apt install ros-humble-gazebo-ros-pkgs
    sudo apt install ros-humble-nav2-bringup
    sudo apt install ros-humble-navigation2
    sudo apt install ros-humble-robot-localization
    sudo apt install ros-humble-ackermann-msgs
    sudo apt install ros-humble-ros2-control
    sudo apt install ros-humble-ros2-controllers
    sudo apt install ros-humble-slam-toolbox
    sudo apt-get install ros-humble-tf2-tools 
    sudo apt-get install ros-humble-tf-transformations
    sudo apt-get install ros-humble-nav2-smac-planner
    sudo apt install ros-humble-navigation2
    sudo apt install ros-humble-rviz2  
   ```
## Demo - Visuaize TAS car 
Use the following command to visualize TAS car in rviz and gazebo
  ```
  ros2 launch tas2-simulator visualizeTAScar.py
  ```
Note that this launch file only spawn the TAS car in Gazebo and Rviz. You will not be able to move the car.

## Demo - Move the car in Gazebo
Run the following launch file to spawn the TAS car in Gazebo
  ```
  ros2 launch tas2-simulator gazeboBringupTAScar.py
  ```
Now open another terminal and run the following command
  ```
  rqt_robot_steering
  ```
You will see a GUI where you can move the car up/down and also turn. Try publishing on topic /cmd_vel via this GUI and you will see car moving in Gazebo. (Hint: For turns, keep the angle value minimum)

## Demo - Autonomous Navigation via NAV2
In order to try navigation demo, run the following command first,
```
  ros2 launch tas2-simulator gazeboBringupTAScar.py
  ```
When the TAS car has spawned in Gazebo, then run the following command in separate terminal
  ```
  ros2 launch tas2-simulator navigationTAScar.py
  ```
You will now see the rviz starting up and loading of Nav2 related nodes. 
Confirm that you see the map in the rviz otherwise you have to restart.
Now, on the upper side of rviz, you will see a tab with the name "2D Goal Pose", press it and click somewhere on the map and you will see the car moving to that goal pose. (In order to rotate in the corridor or complex goal position its recommended to tune the nav2 parameters such as inflation_radius etc. )

## Possible Issues and their solution
 - Sometime gazebo does not start and keeps crashing. Try this,
    ```
    source /usr/share/gazebo-11/setup.sh
    ```
 - **In case the map in rviz does not load due to low performance PC then make the following adjustments**
   - uncomment the line 'ld.add_action(start_rviz_cmd)' from the file 'gazeboBringupTAScar.py'
   - comment the same line 'ld.add_action(start_rviz_cmd)' from the file 'navigationTAScar.py'. It will launch the rviz before the nav2 and the map will appear in rviz.
 - Remember to check the file paths/directory names in case you encounter issues while installing TAS2 simulator. 

### Known bugs or performance related remarks
  - Sometimes car jitters/move itself in Gazebo
    - Confirm that there is no node active that is publishing on cmd_vel topic. Otherwise it could be due to link inertia and should be resolved by adjusting its values
  - Autonomous navigation performance
    - The performance of autonomous navigation can be improved by changing the nav2 parameters in navigation.yaml file.
## TODO (Work in progress)
- For now the differential drive plugin is being used. For ackermann steering, two new files 'tas_car_v2.urdf' and 'tas_car_v2.sdf' have been created. These files have gazebo ackermann steering plugin. However, its a work in progress and not fully finished yet.
- EKF localization is only considering the wheel odometry for now. The imu is not configured yet (check ekf.yaml file).
## Thanks
Special thanks to Christian Ott for providing TAS car mesh files and Martin Schuck for designing Gazebo world for N5 basement and related URDF/SDF files.
