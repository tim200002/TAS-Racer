#include "online_planner/distance_transform.h"

Grid<double> DistanceTransform::calculate_distmap(Grid<unsigned char>& binaryGrid)
{
    unsigned int size_x = binaryGrid.size_x;
    unsigned int size_y = binaryGrid.size_y;

     RCLCPP_INFO(logger, "distmap transform sizex , size y:  %d %d", size_x, size_y);


    // claculate edt
    cv::Mat gridMapImage(size_y, size_x, CV_8UC1);
    uchar *uchar_ptr = gridMapImage.ptr<uchar>(0);
    for (size_t x = 0; x < size_x; ++x)
    {
        for (size_t y = 0; y < size_y; ++y)
        {   
            if (binaryGrid.get_value(x, y) == 1)
            {
                uchar_ptr[y * size_x + x] = 0;
            }
            else if (binaryGrid.get_value(x, y) == 0)
            {
                uchar_ptr[y * size_x + x] = 255;
            }
            else
            {
                throw std::runtime_error("The value of occupancy map should be 0 or 1.");
            }
        }
    }
    cv::imwrite("/home/tim/tas2-racer/out/debug/before_distmap.png", gridMapImage);

    RCLCPP_INFO(logger, "distmap before check some values %d %d %d", uchar_ptr[0], uchar_ptr[1], uchar_ptr[499*size_x + 500]);

    // calculate the educlidean distance transform via OpenCV distanceTransform
    // function
    cv::Mat distanceFieldImage;
    cv::distanceTransform(gridMapImage, distanceFieldImage, cv::DIST_L2,
                          cv::DIST_MASK_PRECISE);
    
    //cv::normalize(distanceFieldImage, distanceFieldImage, 0, 1.0, cv::NORM_MINMAX);

    cv::imwrite("/home/tim/tas2-racer/out/debug/distmap.png", distanceFieldImage);

    std::vector<double> distmap;
    distmap.reserve(size_x * size_y);

    float *float_ptr = distanceFieldImage.ptr<float>(0);
    for (size_t i = 0; i < distanceFieldImage.rows * distanceFieldImage.cols; ++i)
    {
        distmap.push_back(float_ptr[i]);
    }

     RCLCPP_INFO(logger, "distmap after check some values %f %f %f", float_ptr[0], float_ptr[1], float_ptr[2]);
      RCLCPP_INFO(logger, "distmap after check vector some values %f %f %f", distmap[0], distmap[1], distmap[2]);

    return Grid<double>(size_x, size_y, distmap);
}