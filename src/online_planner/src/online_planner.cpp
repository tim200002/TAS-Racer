
#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <fstream>
#include <boost/algorithm/string.hpp>

#include "online_planner/online_planner.hpp"

namespace online_planner
{

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
        RCLCPP_INFO(
            node_->get_logger(), "Global fram %s ",
            global_frame_.c_str());

        // get parameter value
        nav2_util::declare_parameter_if_not_declared(node_, name_ + ".trajectory_file", rclcpp::ParameterValue("/home/tim/tas2-racer/to_be_saved/trajectory.txt"));
        node_->get_parameter(name_ + ".trajectory_file", trajectory_file);
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

        // Load trajetory
        std::vector<Pose> trajectory;
        std::fstream file;
        file.open(trajectory_file, std::ios::in);
        if (file.is_open())
        {
            std::string line;
            while (std::getline(file, line))
            {
                // ignore lines that start with #, they are comments
                if (line[0] == '#')
                {
                    continue;
                }

                // each line is of form x,y,yaw
                std::vector<std::string> results;
                boost::split(results, line, [](char c)
                             { return c == ','; });
                // assert(results.size() == 3);
                WorldPoint point = WorldPoint(stod(results[0]), stod(results[1]));
                Quaternion orientation = Quaternion::from_euler(0, 0, stod(results[2]));
                Pose pose = Pose(point, orientation);
                trajectory.push_back(pose);
            }
            file.close();
        }

        route_planner = new RoutePlanner(trajectory);
    }

    void OnlinePlanner::deactivate()
    {
        RCLCPP_INFO(
            node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
            name_.c_str());

        delete route_planner;
    }

    nav_msgs::msg::Path OnlinePlanner::createPlan(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal)
    {

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

        std::vector<Pose> trajectory_m = route_planner->runStep(WorldPoint(start.pose.position.x, start.pose.position.y), *costmap_);

        global_path.poses.clear();
        global_path.header.stamp = node_->now();
        global_path.header.frame_id = global_frame_;
        for (Pose &pose : trajectory_m)
        {
            geometry_msgs::msg::PoseStamped ros_pose;
            ros_pose.pose = pose.toRosPose();
    
            ros_pose.header.stamp = node_->now();
            ros_pose.header.frame_id = global_frame_;
            global_path.poses.push_back(ros_pose);
        }

        return global_path;
    }

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(online_planner::OnlinePlanner, nav2_core::GlobalPlanner)