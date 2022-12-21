
#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>

#include "custom_interfaces/srv/find_path.hpp"

#include "minimum_curvature_planner/minimum_curvature_planner.hpp"

namespace minimum_curvature_planner
{

    void MinimumCurvaturePlanner::configure(
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

    void MinimumCurvaturePlanner::cleanup()
    {
        RCLCPP_INFO(
            node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
            name_.c_str());
    }

    void MinimumCurvaturePlanner::activate()
    {
        RCLCPP_INFO(
            node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
            name_.c_str());
    }

    void MinimumCurvaturePlanner::deactivate()
    {
        RCLCPP_INFO(
            node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
            name_.c_str());
    }

    nav_msgs::msg::Path MinimumCurvaturePlanner::createPlan(
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
        rclcpp::Client<custom_interfaces::srv::FindPath>::SharedPtr client =
            node_->create_client<custom_interfaces::srv::FindPath>("find_path");

        auto request = std::make_shared<custom_interfaces::srv::FindPath::Request>();

        request->start_pose = start;
        request->goal_pose = goal;
        ;

        while (!client->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return global_path;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        auto result = client->async_send_request(request);
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

        // // calculating the number of loops for current value of interpolation_resolution_
        // int total_number_of_loop = std::hypot(
        //                                goal.pose.position.x - start.pose.position.x,
        //                                goal.pose.position.y - start.pose.position.y) /
        //                            interpolation_resolution_;
        // double x_increment = (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
        // double y_increment = (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;

        // for (int i = 0; i < total_number_of_loop; ++i)
        // {
        //     geometry_msgs::msg::PoseStamped pose;
        //     pose.pose.position.x = start.pose.position.x + x_increment * i;
        //     pose.pose.position.y = start.pose.position.y + y_increment * i;
        //     pose.pose.position.z = 0.0;
        //     pose.pose.orientation.x = 0.0;
        //     pose.pose.orientation.y = 0.0;
        //     pose.pose.orientation.z = 0.0;
        //     pose.pose.orientation.w = 1.0;
        //     pose.header.stamp = node_->now();
        //     pose.header.frame_id = global_frame_;
        //     global_path.poses.push_back(pose);
        // }

        // global_path.poses.push_back(goal);

        // return global_path;
    }

} // namespace nav2_straightline_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(minimum_curvature_planner::MinimumCurvaturePlanner, nav2_core::GlobalPlanner)