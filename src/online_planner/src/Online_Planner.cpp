
#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>

#include "nav_msgs/msg/occupancy_grid.h"
#include "nav_msgs/msg/map_meta_data.h"
#include "geometry_msgs/msg/pose.h"
#include "std_msgs/msg/header.h"

#include "online_planner/Online_Planner.hpp"

using namespace std::literals::chrono_literals;

void OnlinePlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    node_ = parent.lock();
    name_ = name;
    tf_ = tf;
    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();
}

void OnlinePlanner::cleanup()
{
    RCLCPP_INFO(
        node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
        name_.c_str());
}

void OnlinePlanner::activate()
{
    RCLCPP_INFO(
        node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
        name_.c_str());

    // setup client_ for communication
    client_ = node_->create_client<custom_interfaces::srv::FindPath>("find_path");
    while (!client_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
}

void OnlinePlanner::deactivate()
{
    RCLCPP_INFO(
        node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
        name_.c_str());
}

nav_msgs::msg::Path OnlinePlanner::createPlan(
    const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal)
{

    using namespace std::chrono_literals;
    nav_msgs::msg::Path global_path;

    // Checking if the goal and start state is in the global frame
    if (start.header.frame_id != global_frame_)
    {
        RCLCPP_ERROR(
            node_->get_logger(), "Planner will only except start position from %s frame",
            global_frame_.c_str());
        return global_path;
    }

    if (goal.header.frame_id != global_frame_)
    {
        RCLCPP_INFO(
            node_->get_logger(), "Planner will only except goal position from %s frame",
            global_frame_.c_str());
        return global_path;
    }

    // Call our service
    auto request = std::make_shared<custom_interfaces::srv::FindPath::Request>();

    nav_msgs::msg::OccupancyGrid costmap_msg;

    // fill data
    int costmap_size = costmap_->getSizeInCellsY() * costmap_->getSizeInCellsY();
    uint8_t* costmap_buffer = costmap_->getCharMap();
    std::vector<int8_t> costmap_vector;
    costmap_vector.reserve(costmap_size);
    for(int i = 0; i<costmap_size; ++i){
        costmap_vector[i] = costmap_buffer[i];
    }

    costmap_msg.data = costmap_vector;
    
    nav_msgs::msg::MapMetaData info;
    info.height = costmap_->getSizeInCellsY();
    info.width = costmap_->getSizeInCellsY();
    info.resolution = costmap_->getResolution();

    geometry_msgs::msg::Pose origin;
    origin.position.x = costmap_->getOriginX();
    origin.position.y = costmap_->getOriginY();
    origin.position.z = 0;

    origin.orientation.x = 0;
    origin.orientation.y = 0;
    origin.orientation.z = 0;
    origin.orientation.w = 1;

    info.origin = origin;

    costmap_msg.info = info;
    
    

    request->start_pose = start;
    request->goal_pose = goal;
    request->costmap = costmap_msg;


    auto result = client_->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node_, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {

        global_path.poses.clear();
        global_path.header.stamp = node_->now();
        global_path.header.frame_id = global_frame_;
        std::vector<geometry_msgs::msg::PoseStamped> path = result.get()->path_poses;

        // update the stamp_signature
        for (auto it = path.begin(); it != path.end(); ++it)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = it->pose.position.x;
            pose.pose.position.y = it->pose.position.y;
            pose.pose.position.z = it->pose.position.z;
            pose.pose.orientation.x = it->pose.orientation.x;
            pose.pose.orientation.y = it->pose.orientation.y;
            pose.pose.orientation.z = it->pose.orientation.z;
            pose.pose.orientation.w = it->pose.orientation.w;
            pose.header.stamp = node_->now();
            pose.header.frame_id = global_frame_;
            global_path.poses.push_back(pose);
        }
        return global_path;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service path_finder");
        return global_path;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(OnlinePlanner, nav2_core::GlobalPlanner)