#pragma once
#include "online_planner/route_planner.hpp"
#include "online_planner/grid.hpp"
#include <cmath>
#include <algorithm>
#include "online_planner/a_star_efficient.hpp"
#include "online_planner/a_str_node.hpp"
#include "online_planner/bezier.hpp"

template <typename T>
std::vector<T> splitVector(const std::vector<T> &vec, unsigned int start, int end)
{
    if (end == -1)
    {
        end = vec.size();
    }
    std::vector<T> new_vector;
    new_vector.reserve(end - start);

    for (size_t i = start; i < end; ++i)
    {
        new_vector.push_back(vec[i]);
    }
    return new_vector;
}

template <typename T>
std::vector<T> concatenateVectors(const std::vector<T> &vec1, const std::vector<T> &vec2)
{
    std::vector<T> resVector = vec1;
    resVector.insert(resVector.end(), vec2.begin(), vec2.end());
    return resVector;
}

template <typename T>
unsigned int RoutePlanner::find_closest_point_idx_on_path(std::vector<Point<T>> path, Point<T> reference_point)
{
    double closest_distance = std::numeric_limits<double>::infinity();
    unsigned int closest_point_idx;

    for (size_t i = 0; i < path.size(); ++i)
    {
        Point point = path[i];
        double distance = sqrt(pow(point.x - reference_point.x, 2) + pow(point.y - reference_point.y, 2));
        if (distance < closest_distance)
        {
            closest_distance = distance;
            closest_point_idx = i;
        }
    }

    return closest_point_idx;
}

std::tuple<std::vector<Pose>, std::vector<GridPoint>> RoutePlanner::extend_path_meters_into_future(std::vector<Pose> trajectory_m, std::vector<GridPoint> path_p, int start_idx, float extension_distance)
{
    Pose current_pose_meters = trajectory_m[start_idx];
    GridPoint current_pos_pixels = path_p[start_idx];

    double running_distance = 0;

    std::vector<Pose> extended_trajectory_m = {current_pose_meters};
    std::vector<GridPoint> extended_path_p = {current_pos_pixels};

    for (size_t i = start_idx + 1; i < trajectory_m.size(); ++i)
    {
        Pose previous_pose = trajectory_m[i - 1];
        Pose current_pose = trajectory_m[i];
        double distance = sqrt(pow(current_pose.coordinate.x - previous_pose.coordinate.x, 2) + pow(current_pose.coordinate.y - previous_pose.coordinate.y, 2));
        running_distance += distance;
        // RCLCPP_INFO(logger, "Extend path itration i %d, delta distace %f, extended distance %f", i, distance, running_distance);
        extended_trajectory_m.push_back(current_pose);
        extended_path_p.push_back(path_p[i]);
        if (running_distance > extension_distance)
        {
            break;
        }
    }

    return std::make_tuple(extended_trajectory_m, extended_path_p);
}

std::tuple<std::vector<Pose>, std::vector<GridPoint>> RoutePlanner::extend_path_meters_into_future(std::vector<Pose> trajectory_m, std::vector<GridPoint> path_p, int start_idx, float extension_distance, int &end_index_return)
{
    Pose current_pose_meters = trajectory_m[start_idx];
    GridPoint current_pos_pixels = path_p[start_idx];

    double running_distance = 0;

    std::vector<Pose> extended_trajectory_m = {current_pose_meters};
    std::vector<GridPoint> extended_path_p = {current_pos_pixels};

    for (size_t i = start_idx + 1; i < trajectory_m.size(); ++i)
    {
        Pose previous_pose = trajectory_m[i - 1];
        Pose current_pose = trajectory_m[i];
        double distance = sqrt(pow(current_pose.coordinate.x - previous_pose.coordinate.x, 2) + pow(current_pose.coordinate.y - previous_pose.coordinate.y, 2));
        running_distance += distance;
        // RCLCPP_INFO(logger, "Extend path itration i %d, delta distace %f, extended distance %f", i, distance, running_distance);
        extended_trajectory_m.push_back(current_pose);
        extended_path_p.push_back(path_p[i]);
        if (running_distance > extension_distance)
        {
            end_index_return = i;
            break;
        }
    }

    return std::make_tuple(extended_trajectory_m, extended_path_p);
}

std::tuple<bool, int> RoutePlanner::check_path_for_collision(std::vector<GridPoint> path, float min_margin, Grid<double> &distanceFilteredGrid)
{
    for (size_t i = 0; i < path.size(); ++i)
    {
        GridPoint point = path[i];
        if (point.x < distanceFilteredGrid.size_x && point.y < distanceFilteredGrid.size_y && distanceFilteredGrid.get_value(point.x, point.y) < min_margin)
        {
            return std::make_tuple(true, i);
        }
    }

    return std::make_tuple(false, 0);
}

std::tuple<bool, int> RoutePlanner::check_path_for_collision_interpolated(std::vector<GridPoint> path, float min_margin, Grid<double> &distanceFilteredGrid)
{
    RCLCPP_INFO(logger, "start check collision interpolated");

    int interpolation_steps = 1000;
    for (size_t i = 0; i < path.size() - 1; ++i)
    {

        Point<float> current = Point<float>(path[i].x, path[i].y);
        Point<float> next = Point<float>(path[i + 1].x, path[i + 1].y);
        Point<float> vec = Point<float>(next.x - current.x, next.y - current.y);

        for (int k = 0; k < interpolation_steps; ++k)
        {
            // RCLCPP_INFO(logger, "Start interpolation for index %d and step %d", i, k);
            Point<float> pose_interpolated = Point<float>(current.x + vec.x / interpolation_steps * k, current.y + vec.y / interpolation_steps * k);
            // RCLCPP_INFO(logger, "Start interpolation for index %d and step %d interpolated pose(%f, %f)", i, k, pose_interpolated.x, pose_interpolated.y);
            int x_min = floor(pose_interpolated.x);
            int x_max = ceil(pose_interpolated.x);
            int y_min = floor(pose_interpolated.y);
            int y_max = ceil(pose_interpolated.y);
            std::vector<GridPoint> points = {GridPoint(x_min, y_min), GridPoint(x_min, y_max), GridPoint(x_max, y_min), GridPoint(x_max, y_max)};

            for (GridPoint &point : points)
            {
                if (point.x < distanceFilteredGrid.size_x && point.y < distanceFilteredGrid.size_y)
                {
                    if (distanceFilteredGrid.get_value(point.x, point.y) < min_margin)
                    {
                        return std::make_tuple(true, i);
                    }
                }
            }
        }
    }
    return std::make_tuple(false, 0);
}

std::vector<Pose> RoutePlanner::runStep(WorldPoint current_pos_m, nav2_costmap_2d::Costmap2D &costmap)
{
    RCLCPP_INFO(logger, "start new iteration current pos %f, %f", current_pos_m.x, current_pos_m.y);

    // convert reference trajectory to pixels
    std::vector<GridPoint> reference_path_pixels;
    reference_path_pixels.reserve(reference_path_m.size());
    for (size_t i = 0; i < reference_path_m.size(); ++i)
    {
        int x, y;
        costmap.worldToMapNoBounds(reference_path_m[i].x, reference_path_m[i].y, x, y);
        reference_path_pixels.push_back(GridPoint(x, y));
    }

    // Reformat Costmap to binary, all valid points have value 0 and invalid points have value 1
    Grid<unsigned char> binary_grid = Grid<unsigned char>(costmap.getSizeInCellsX(), costmap.getSizeInCellsY());
    for (int x = 0; x < binary_grid.size_x; ++x)
    {
        for (int y = 0; y < binary_grid.size_y; ++y)
        {

            int thresh = 95;
            if (costmap.getCost(x, y) > thresh)
            {
                binary_grid.set_value(x, y, 1);
            }
            else
            {
                binary_grid.set_value(x, y, 0);
            }
        }
    }

    // calculate eulidean distance transform
    RCLCPP_INFO(logger, "Calculate EDT");
    Grid<double> distanceToObjectPixels = distance_transformer.calculate_distmap(binary_grid);

    if (!isAvoidingCollision)
    {
        RCLCPP_INFO(logger, "Not avoiding collision");

        RCLCPP_INFO(logger, "Get Closes point idx");
        int closest_point_idx = find_closest_point_idx_on_path(reference_path_m, current_pos_m);
        GridPoint closest_point_pixels = reference_path_pixels[closest_point_idx];

        RCLCPP_INFO(logger, "Extend Path");
        std::tuple<std::vector<Pose>, std::vector<GridPoint>> res_extension = extend_path_meters_into_future(reference_trajectory_m, reference_path_pixels, closest_point_idx, lookaheadDistance);
        std::vector<Pose> lookahead_trajectory_meters = std::get<0>(res_extension);
        std::vector<GridPoint> lookahed_path_pixels = std::get<1>(res_extension);

        RCLCPP_INFO(logger, "Lookahead path ends at (%f, %f)", lookahead_trajectory_meters.back().coordinate.x, lookahead_trajectory_meters.back().coordinate.y);

        RCLCPP_INFO(logger, "Check collision");
        std::tuple<bool, int> res_check_collision = check_path_for_collision(lookahed_path_pixels, marginPixels, distanceToObjectPixels);
        // Did not collide
        if (!std::get<0>(res_check_collision))
        {
            RCLCPP_INFO(logger, "reference path is good, return");
            return lookahead_trajectory_meters;
        }

        // collided
        int collision_idx = std::get<1>(res_check_collision) + closest_point_idx;
        RCLCPP_INFO(logger, "Reference path not good, instead Collided at collision idx %d", collision_idx);

        // find closest point after object that is far enough away from object
        size_t idx_after_initial_collision = collision_idx + 1;

        int first_valid_idx = -1;
        for (size_t i = idx_after_initial_collision; i < reference_path_pixels.size(); ++i)
        {
            GridPoint point_px = reference_path_pixels[i];
            double distance_pixels = distanceToObjectPixels.get_value(point_px.x, point_px.y);
            if (distance_pixels > marginPixels)
            {
                first_valid_idx = i;
                break;
            }
        }

        RCLCPP_INFO(logger, "Closes point that is ave to merge back into %d", idx_after_initial_collision);

        res_extension = extend_path_meters_into_future(reference_trajectory_m, reference_path_pixels, first_valid_idx, 0.5);
        std::vector<GridPoint> extension_path_pixels_1 = std::get<1>(res_extension);

        int end_idx_extension;
        res_extension = extend_path_meters_into_future(reference_trajectory_m, reference_path_pixels, first_valid_idx, mergeBackDistance, end_idx_extension);
        std::vector<GridPoint> extension_path_pixels_2 = std::get<1>(res_extension);

        RCLCPP_INFO(logger, "Extended path int future");

        // plan new path using A Star

        int offset_to_collision = 8;
        int collision_idx_offseted = std::max(collision_idx - offset_to_collision, 0);
        GridPoint start_point = reference_path_pixels[collision_idx_offseted];
        GridPoint early_stop_point = extension_path_pixels_1.back(); // reference_path_pixels[first_valid_idx + 5];
        GridPoint merge_back_point = extension_path_pixels_2.back();

        RCLCPP_INFO(logger, "Before A star");
        std::vector<GridPoint> path_pixels = runAstarStep(start_point, early_stop_point, extension_path_pixels_2[extension_path_pixels_2.size() -2],  extension_path_pixels_2.back(), distanceToObjectPixels, 10);
        RCLCPP_INFO(logger, "After A star");


        std::vector<Pose> start_trajectory = splitVector(reference_trajectory_m, 0, collision_idx_offseted - 1);
        std::vector<Pose> trajectory_a_star = pathPixelsToTrajectoryMeters(path_pixels, costmap);
        std::vector<Pose> trajectory_combined = concatenateVectors(start_trajectory, trajectory_a_star);

        // set to Collision avoidance
        isAvoidingCollision = true;
        collision_avoidance_trajectory_m = trajectory_combined;

        return collision_avoidance_trajectory_m;

        // return pathPixelsToTrajectoryMeters(path_pixels, costmap);
    }
    else
    {
        RCLCPP_INFO(logger, "Avoiding collision");

        // convefrt stored trajector yto path
        // convert reference trajectory to pixels
        std::vector<GridPoint> collision_avoidance_path_pixels;
        collision_avoidance_path_pixels.reserve(collision_avoidance_trajectory_m.size());
        for (size_t i = 0; i < collision_avoidance_path_pixels.size(); ++i)
        {
            int x, y;
            costmap.worldToMapNoBounds(collision_avoidance_trajectory_m[i].coordinate.x, collision_avoidance_trajectory_m[i].coordinate.y, x, y);
            collision_avoidance_path_pixels.push_back(GridPoint(x, y));
        }

        // Check if we reached end of collision avoidance maneuver
        double distance_to_end_pixel = sqrt(pow(current_pos_m.x - collision_avoidance_trajectory_m.back().coordinate.x, 2) + pow(current_pos_m.y - collision_avoidance_trajectory_m.back().coordinate.y, 2));
        float delta_m = 1;

        RCLCPP_INFO(logger, "Did not reach end of collision avoidance: distance %f", distance_to_end_pixel);

        if (distance_to_end_pixel < delta_m)
        {
            RCLCPP_INFO(logger, "Reached end of collision avoidance");
            // collisionAvoidancePAthPixels = None
            isAvoidingCollision = false;

            return runStep(current_pos_m, costmap);
        }

        RCLCPP_INFO(logger, "Did not reach end of collision avoidance");
        // Take collision avoidance path and check if it is still valid
        // int closest_point_idx = find_closest_point_idx_on_path(collision_avoidance_path_pixels, current_pos_pixels);
        std::tuple<bool, int> res_check_collision = check_path_for_collision(collision_avoidance_path_pixels, marginPixels, distanceToObjectPixels);
        RCLCPP_INFO(logger, "AFter check fo collision");

        // Path still valid
        if (!std::get<0>(res_check_collision))
        {
            RCLCPP_INFO(logger, "Path still valid");
            return collision_avoidance_trajectory_m;
        }

        RCLCPP_INFO(logger, "Path invalid, recalculate");

        // Calculating new collision avoidance path

        // We dont want to recalculate whole path only partially
        // go some steps from collision back only recalculate path from here and add it to new one
        int collision_idx = std::get<1>(res_check_collision);

        int offset_to_collision = 2;
        int collision_idx_offseted = std::max(collision_idx - offset_to_collision, 0);
        GridPoint start_point = collision_avoidance_path_pixels[collision_idx_offseted];

        // recalculate end pont once again to admit offfset to objects

        // calculate closes point on reference trajectory
        int closest_point_idx_ref_trajectory = find_closest_point_idx_on_path(reference_path_pixels, start_point);

        // Check if closes posize_t is colliding
        GridPoint closes_point_pixels = reference_path_pixels[closest_point_idx_ref_trajectory];

        // not quite sure how to handle this case, probably we are lucky and pretty much at end of collision
        // use old end point
        GridPoint end_point = GridPoint(0, 0);
        if (distanceToObjectPixels.get_value(closes_point_pixels.x, closes_point_pixels.y) > marginPixels)
        {
            end_point = collision_avoidance_path_pixels.back();
        }
        // insert margin for merging back
        else
        {
            size_t idx_after_initial_collision = closest_point_idx_ref_trajectory + 1;

            // once again go to back of collision
            int first_valid_idx = -1;
            for (size_t i = idx_after_initial_collision; i < reference_path_pixels.size(); ++i)
            {
                GridPoint point_px = reference_path_pixels[i];
                double distance_pixels = distanceToObjectPixels.get_value(point_px.x, point_px.y);
                if (distance_pixels > marginPixels)
                {
                    first_valid_idx = i;
                    break;
                }
            }

            std::tuple<std::vector<Pose>, std::vector<GridPoint>> res_extension = extend_path_meters_into_future(reference_trajectory_m, reference_path_pixels, first_valid_idx, mergeBackDistance);
            end_point = std::get<1>(res_extension).back();
        }

        // TODO
        // std::vector<GridPoint> path_pixels = runAstarStep(start_point, end_point, end_point, distanceToObjectPixels, 10);

        // std::vector<Pose> start_trajectory = splitVector(collision_avoidance_trajectory_m, 0, collision_idx_offseted - 1);
        // std::vector<Pose> trajectory_a_star = pathPixelsToTrajectoryMeters(path_pixels, costmap);
        // std::vector<Pose> trajectory_combined = concatenateVectors(start_trajectory, trajectory_a_star);
        // collision_avoidance_trajectory_m = trajectory_combined;

        return collision_avoidance_trajectory_m;
    }
}

std::vector<GridPoint> runAstarSubStep(MapSearchNode start, MapSearchNode end, rclcpp::Logger logger)
{
    AStarSearch<MapSearchNode> astarsearch;
    astarsearch.SetStartAndGoalStates(start, end);

    unsigned int SearchState;
    unsigned int SearchSteps = 0;

    do
    {
        SearchState = astarsearch.SearchStep();

        SearchSteps++;
    } while (SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING);

    RCLCPP_INFO(logger, "seacrh state %d search steps %d", SearchState, SearchSteps);
    if (SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED)
    {
        RCLCPP_INFO(logger, "A Star was succesful");
        std::vector<GridPoint> path_pixels;
        cout << "Search found goal state\n";

        MapSearchNode *node = astarsearch.GetSolutionStart();
        path_pixels.push_back(GridPoint(node->x, node->y));

        for (;;)
        {
            node = astarsearch.GetSolutionNext();

            if (!node)
            {
                break;
            }

            path_pixels.push_back(GridPoint(node->x, node->y));
        };

        astarsearch.FreeSolutionNodes();

        return path_pixels;

        // if (sampling_distance == 1)
        // {
        //     return path_pixels;
        // }

        // std::vector<GridPoint> path_sampled;
        // path_sampled.reserve(path_pixels.size() / sampling_distance + 1);
        // for (size_t i = 0; i < path_pixels.size(); i += sampling_distance)
        // {
        //     path_sampled.push_back(path_pixels[i]);
        // }

        // // last element should alway be part of path
        // if (path_sampled.back() != path_pixels.back())
        // {
        //     path_sampled.push_back(path_pixels.back());
        // }
        // return path_sampled;
    }
    else if (SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED)
    {
        RCLCPP_INFO(logger, "A Star NOT succesful");
    }
}

std::vector<GridPoint> RoutePlanner::runAstarStep(GridPoint start, GridPoint early_end, GridPoint merge_back_end_1, GridPoint merge_back_end_2, Grid<double> &distanceToObjectPixels, unsigned char sampling_distance)
{

    // Step 1 do a broad seach until end
    RCLCPP_INFO(logger, "A Star 1");
    MapSearchNode nodeStart = MapSearchNode(start.x, start.y, distanceToObjectPixels.size_x, distanceToObjectPixels.size_y, distanceToObjectPixels.getGrid(), marginPixels_untight, true);
    MapSearchNode nodeEnd = MapSearchNode(early_end.x, early_end.y, distanceToObjectPixels.size_x, distanceToObjectPixels.size_y, distanceToObjectPixels.getGrid(), marginPixels_untight, true);

    std::vector<GridPoint> early_path = runAstarSubStep(nodeStart, nodeEnd, logger);

    if (sampling_distance != 1)
    {
        std::vector<GridPoint> path_sampled;
        path_sampled.reserve(early_path.size() / sampling_distance + 1);
        for (size_t i = 0; i < early_path.size(); i += sampling_distance)
        {
            path_sampled.push_back(early_path[i]);
        }

        // last element should alway be part of path
        if (path_sampled.back() != early_path.back())
        {
            path_sampled.push_back(early_path.back());
        }
        early_path = path_sampled;
    }

    // Step2 do fine seach until merge back point
    RCLCPP_INFO(logger, "Smooth rest");

    GridPoint point1 = early_path[early_path.size() - 2];
    GridPoint point2 = early_path.back();
    GridPoint point3 = merge_back_end_1;
    GridPoint point4 = merge_back_end_2;

    std::vector<GridPoint> smoothEnd;
    Bezier::Bezier<3> cubicBezier({{point1.x, point1.y}, {point2.x, point2.y}, {point3.x, point3.y}, {point4.x, point4.y}});
    for (double i = 0; i < 1; i += 0.1)
    {
        Bezier::Point p = cubicBezier.valueAt(i);
        smoothEnd.push_back(GridPoint((int)p.x, (int)p.y));
        RCLCPP_INFO(logger, "Bezier at %f, %f", p.x, p.y);
    }

    std::vector<GridPoint> pathPixelsSplit = splitVector(early_path, 0, early_path.size() - 2);
    std::vector<GridPoint> pathCombined = concatenateVectors(pathPixelsSplit, smoothEnd);

    return pathCombined;
}

std::vector<Pose> RoutePlanner::pathPixelsToTrajectoryMeters(std::vector<GridPoint> &path, nav2_costmap_2d::Costmap2D &costmap)
{
    std::vector<WorldPoint> path_m;
    path_m.reserve(path.size());
    for (size_t i = 0; i < path.size(); ++i)
    {
        GridPoint current_pos = path[i];
        double x, y;
        costmap.mapToWorld(current_pos.x, current_pos.y, x, y);
        path_m.push_back(WorldPoint(x, y));
    }

    std::vector<Pose> trajectory;
    trajectory.reserve(path.size());

    for (size_t i = 0; i < path_m.size() - 1; ++i)
    {
        WorldPoint vec_to_target = path_m[i + 1] - path_m[i];
        double yaw = atan2(vec_to_target.y, vec_to_target.x);

        trajectory.push_back(Pose(path_m[i], Quaternion::from_euler(0, 0, yaw)));
    }

    return trajectory;
}