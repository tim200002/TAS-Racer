
#pragma once
#include <opencv2/opencv.hpp>
#include "online_planner/grid.hpp"
#include "rclcpp/rclcpp.hpp"

class DistanceTransform
{
public:
    Grid<double> calculate_distmap(Grid<unsigned char>& binaryGrid);

private:
    rclcpp::Logger logger = rclcpp::get_logger("distance_transform");
};