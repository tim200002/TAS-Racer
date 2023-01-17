
#pragma once

#include "costmap_2d.hpp"
#include <opencv2/opencv.hpp>
class DistanceTransform
{
public:
    std::vector<double> calculate_distmap(nav2_costmap_2d::Costmap2D &costmap);

private:
    unsigned int size_x = 0;
    unsigned int size_y = 0;
    unsigned int lethal_thresh = 95;
};