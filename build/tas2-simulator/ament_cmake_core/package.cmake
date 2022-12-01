set(_AMENT_PACKAGE_NAME "tas2-simulator")
set(tas2-simulator_VERSION "0.0.1")
set(tas2-simulator_MAINTAINER "user <salmanbari03@gmail.com>")
set(tas2-simulator_BUILD_DEPENDS "std_msgs")
set(tas2-simulator_BUILDTOOL_DEPENDS "ament_cmake")
set(tas2-simulator_BUILD_EXPORT_DEPENDS )
set(tas2-simulator_BUILDTOOL_EXPORT_DEPENDS )
set(tas2-simulator_EXEC_DEPENDS "ament_index_python" "ros2launch" "joint_state_publisher" "joint_state_publisher_gui" "robot_state_publisher" "xacro" "rviz" "launch_ros" "gazebo_ros2_control" "ros2_controllers" "robot_localization" "gazebo_ros" "control_msgs")
set(tas2-simulator_TEST_DEPENDS "ament_lint_auto" "ament_lint_common")
set(tas2-simulator_GROUP_DEPENDS )
set(tas2-simulator_MEMBER_OF_GROUPS )
set(tas2-simulator_DEPRECATED "")
set(tas2-simulator_EXPORT_TAGS)
list(APPEND tas2-simulator_EXPORT_TAGS "<build_type>ament_cmake</build_type>")
list(APPEND tas2-simulator_EXPORT_TAGS "<exec_depend>nav2_bringup</exec_depend>")
list(APPEND tas2-simulator_EXPORT_TAGS "<exec_depend>slam_toolbox</exec_depend>")
list(APPEND tas2-simulator_EXPORT_TAGS "<gazebo_ros gazebo_model_path=\"models\"/>")
list(APPEND tas2-simulator_EXPORT_TAGS "<gazebo_ros gazebo_media_path=\"models\"/>")
