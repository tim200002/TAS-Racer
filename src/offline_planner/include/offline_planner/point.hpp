#pragma once
#include <iostream>
#include "geometry_msgs/msg/point.hpp"
#include <cmath>

namespace offline_planner
{
    struct Point
    {
        double x, y;

        Point(double x, double y) : x(x), y(y){};

        inline std::ostream &operator<<(std::ostream &_stream)
        {
            // print something from v to str, e.g: Str << v.getX();
            _stream << "(" << x << "," << y << ")";
            return _stream;
        }

        bool operator==(Point &p)
        {
            return (std::abs(p.x - x) <= accuracy && std::abs(p.y - y) <= accuracy);
        }

        bool operator!=(Point &p)
        {
            return !(*this == p);
        }

        Point operator-(const Point &rhs)
        {
            return Point(x - rhs.x, y - rhs.y);
        }


        geometry_msgs::msg::Point toRos2Point()
        {
            geometry_msgs::msg::Point point;
            point.x = x;
            point.y = y;
            point.z = 0.0;
            return point;
        }

        double norm()
        {
            return sqrt(x * x + y * y);
        }

    private:
        double accuracy = 0.01;
    };
} // namespace offline_planner
