#include "online_planner/Distance_Transform.h"

std::vector<double> DistanceTransform::calculate_distmap(nav2_costmap_2d::Costmap2D& costmap){
        unsigned inst size_x = costmap.getSizeInCellsX();
        unsigned int size_y = costmap.getSizeInCellsY();

        const unsigned char *charmap = costmap.getCharMap();

        std::vector<unsigned char> binary_map;
        binary_map.reserve(size_x * size_y)

            for (unsigned int i; i < size_x * size_y; ++i)
        {
            if (charmap[i] >= lethal_thresh)
            {
                binary_map_[i] = 1;
            }
            else
            {
                binary_map_[i] = 0;
            }
        }

        // claculate edt
        cv::Mat gridMapImage(last_size_y_, last_size_x_, CV_8UC1);
        uchar *uchar_ptr = gridMapImage.ptr<uchar>(0);
        for (int i = 0; i < gridMapImage.rows * gridMapImage.cols; ++i)
        {
            if (binary_map_[i] == 1)
            {
                uchar_ptr[i] = 0;
            }
            else if (binary_map_[i] == 0)
            {
                uchar_ptr[i] = 255;
            }
            else
            {
                throw std::runtime_error("The value of occupancy map should be 0 or 1.");
            }
        }

        // calculate the educlidean distance transform via OpenCV distanceTransform
        // function
        cv::Mat distanceFieldImage;
        cv::distanceTransform(gridMapImage, distanceFieldImage, cv::DIST_L2,
                              cv::DIST_MASK_PRECISE);

        float *float_ptr = distanceFieldImage.ptr<float>(0);
        for (int i = 0; i < distanceFieldImage.rows * distanceFieldImage.cols; ++i)
        {
            distmap_[i] = static_cast<double>(float_ptr[i]) * resolution_;
        }
    }