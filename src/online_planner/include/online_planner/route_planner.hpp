#pragma once

#include <string>
#include <memory>



#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "pose.hpp"
#include "point.hpp"
#include "online_planner/distance_transform.h"
#include "online_planner/grid.hpp"
#include "distance_transform.h"
#include "online_planner/a_star.hpp"
#include "rclcpp/rclcpp.hpp"


class RoutePlanner
{
public:
RoutePlanner(std::vector<Pose> reference_trajectory_m):reference_trajectory_m(reference_trajectory_m){
    reference_path_m.reserve(reference_path_m.size());
    for(size_t i = 0; i < reference_trajectory_m.size(); ++i){
        reference_path_m.push_back( reference_trajectory_m[i].coordinate);
    }
}

std::vector<Pose> runStep(WorldPoint current_pos_m, nav2_costmap_2d::Costmap2D &costmap);

private:
    std::vector<Pose> reference_trajectory_m;
    std::vector<WorldPoint> reference_path_m;

    int lookaheadDistance = 4;
    int mergeBackDistance = 1;
    int marginPixels = 12;
    int marginPixels_untight = 20;

    bool isAvoidingCollision = false;
    std::vector<Pose> collision_avoidance_trajectory_m;

    template<typename T>
    unsigned int find_closest_point_idx_on_path(std::vector<Point<T>> path, Point<T> reference_point);

    std::tuple<std::vector<Pose>, std::vector<GridPoint>> extend_path_meters_into_future(std::vector<Pose> trajectory_m, std::vector<GridPoint> path_p, int start_idx, float extension_distance, int& end_idx_return);
    std::tuple<std::vector<Pose>, std::vector<GridPoint>> extend_path_meters_into_future(std::vector<Pose> trajectory_m, std::vector<GridPoint> path_p, int start_idx, float extension_distance);

    std::tuple<bool, int> check_path_for_collision(std::vector<GridPoint> path, float min_margin, Grid<double>& distanceFilteredGrid);
    std::tuple<bool, int> check_path_for_collision_interpolated(std::vector<GridPoint> path, float min_margin, Grid<double>& distanceFilteredGrid);

    DistanceTransform distance_transformer = DistanceTransform();

    std::vector<GridPoint> runAstarStep(GridPoint start, GridPoint early_end, GridPoint merge_back_end, Grid<double>& distanceToObjectPixels, unsigned char sampling_distance);
    //std::vector<GridPoint> runAstarSubStep(MapSearchNode start, MapSearchNode end);
    std::vector<Pose> pathPixelsToTrajectoryMeters(std::vector<GridPoint>& path, nav2_costmap_2d::Costmap2D &costmap);
    AStar::Generator generator;

    rclcpp::Logger logger = rclcpp::get_logger("route_planner");
};