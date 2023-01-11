
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
                // ignore lines that start with #, they are comments
                if(line[0] == '#'){
                continue;
                }

                // each line is of form x,y,yaw
                std::vector<std::string> results;
                boost::split(results, line, [](char c)
                             { return c == ','; });
                // assert(results.size() == 3);
                Point point = Point(stod(results[0]), stod(results[1]));
                Quaternion orientation = Quaternion::from_euler(0,0, stod(results[2]));
                Pose pose = Pose(point, orientation);
                trajectory.push_back(pose);
            }
            file.close();
        }

        // check start
        Pose start_pose = Pose(start.pose);
        std::vector<Pose>::iterator newBegin;
        bool start_valid = is_point_nearby(start_pose.coordinate, trajectory, newBegin);
        if(!start_valid){
             RCLCPP_ERROR(
                node_->get_logger(), "Invalid start point, too far away from trajectory");
            return global_path;
        }

        std::vector<Pose> trajectoryFiltered;
        for(auto it = newBegin; it != trajectory.end(); ++it){
            trajectoryFiltered.push_back(*it);
        }

        // check goal
        Pose goal_pose = Pose(goal.pose);
        std::vector<Pose>::iterator temp;
        bool goal_valid = is_point_nearby(goal_pose.coordinate, trajectory, temp);
        if(!goal_valid){
             RCLCPP_ERROR(
                node_->get_logger(), "Invalid goal point, too far away from trajectory");
            return global_path;
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

    bool OfflinePlanner::is_point_nearby(Point point, std::vector<offline_planner::Pose>& trajectory, std::vector<Pose>::iterator& first_occurence){
        const int thresh = 1;
        std::vector<Pose>::iterator occurence = trajectory.end();
        
        for(auto it =trajectory.begin(); it != trajectory.end(); ++it){
            Point difference = it-> coordinate - point;
            if(difference.norm() <= thresh){
                occurence = it;
                break;
            }
        }
        if(occurence == trajectory.end()){
            return false;
        }
        first_occurence = occurence;
        return true;
    }

} // namespace nav2_straightline_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(offline_planner::OfflinePlanner, nav2_core::GlobalPlanner)