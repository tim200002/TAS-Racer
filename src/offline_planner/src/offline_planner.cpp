
#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <fstream>
#include <boost/algorithm/string.hpp>

#include "offline_planner/offline_planner.hpp"

namespace offline_planner
{

    void OfflinePlanner::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        node_ = parent.lock();
        name_ = name;
        tf_ = tf;
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();

        // get parameter value
        nav2_util::declare_parameter_if_not_declared(node_, name_ + ".trajectory_file", rclcpp::ParameterValue("/home/tim/tas2-racer/to_be_saved/trajectory.txt"));
        node_->get_parameter(name_ + ".trajectory_file", trajectory_file);
    }

    void OfflinePlanner::cleanup()
    {
        RCLCPP_INFO(
            node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
            name_.c_str());
    }

    void OfflinePlanner::activate()
    {
        RCLCPP_INFO(
            node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
            name_.c_str());
    }

    void OfflinePlanner::deactivate()
    {
        RCLCPP_INFO(
            node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
            name_.c_str());
    }

    nav_msgs::msg::Path OfflinePlanner::createPlan(
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

        // Load trajetory
        std::vector<Pose> trajectory;
        std::fstream file;
        file.open(trajectory_file, std::ios::in);
        if (file.is_open())
        {
            std::string line;
            while (std::getline(file, line))
            {   
                // each line is of form x,y,yaw
                std::vector<std::string> results;
                boost::split(results, line, [](char c)
                             { return c == ','; });
                // assert(results.size() == 3);
                Point point = Point(stod(results[0]), stod(results[1]));
                Quaternion orientation = Quaternion(stod(results[2]), stod(results[3]),stod(results[4]), stod(results[5]));
                Pose pose = Pose(point, orientation);
                trajectory.push_back(pose);
            }
            file.close();
        }


        //! ToDo Implement better Reachability checks
        // convert start and end pose to my type
        // 

        // if (start_pose != trajectory[0])
        // {   
        //     std::string error_msg = "Invalid start pose, start pose is " + start_pose.toString() + " and first pose of trajectory is " + trajectory[0].toString();
        //     RCLCPP_ERROR(
        //         node_->get_logger(), error_msg.c_str());
        //     return global_path;
        // }
        
        // check if final pose is reachable
        // Pose goal_pose = Pose(goal.pose);
        // if (goal_pose != trajectory.back())
        // {   std::string error_msg = "Invalid goal pose, goal pose is " + goal_pose.toString() + " and goal pose of trajectory is " + trajectory.back().toString();
        //     RCLCPP_ERROR(
        //         node_->get_logger(), error_msg.c_str());
        //     return global_path;
        // }

        Pose start_pose = Pose(start.pose);
        const int thresh = 1;
        
        // find first point such that distance to car below some threshold
        std::vector<Pose>::iterator newBegin = trajectory.end();
        
        for(auto it =trajectory.begin(); it != trajectory.end(); ++it){
            Point difference = it->coordinate - start_pose.coordinate;
            if(difference.norm() <= thresh){
                newBegin = it;
                break;
            }
        }

        if(newBegin == trajectory.end()){
             RCLCPP_ERROR(
                node_->get_logger(), "Invalid start point, too far away from trajectory");
            return global_path;
        }

        std::vector<Pose> trajectoryFiltered;
        for(auto it = newBegin; it != trajectory.end(); ++it){
            trajectoryFiltered.push_back(*it);
        }


        // convert loaded trajectory to stamped poses
        global_path.poses.clear();
        global_path.header.stamp = node_->now();
        global_path.header.frame_id = global_frame_;
        for (Pose &pose : trajectoryFiltered)
        {
            geometry_msgs::msg::PoseStamped ros_pose;
            ros_pose.pose = pose.toRosPose();
    
            ros_pose.header.stamp = node_->now();
            ros_pose.header.frame_id = global_frame_;
            global_path.poses.push_back(ros_pose);
        }

        return global_path;
    }

} // namespace nav2_straightline_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(offline_planner::OfflinePlanner, nav2_core::GlobalPlanner)