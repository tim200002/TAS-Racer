#include <iostream>  
#include "geometry_msgs/msg/point.hpp"
#include <cmath>

namespace offline_planner
{
    struct Point
    {   
         double x, y;

        Point(double x, double y) : x(x), y(y){};

        inline std::ostream &operator<<(std::ostream & _stream)
        {
            // print something from v to str, e.g: Str << v.getX();
            _stream << "(" << x << "," << y << ")";
            return _stream;
        }

         bool operator==(Point& p){
            return (std::abs(p.x - x) <= accuracy  && std::abs(p.y - y) <= accuracy);
        }

         bool operator!=(Point& p ){
            return !(*this == p);
        }

        geometry_msgs::msg::Point toRos2Point(){
            geometry_msgs::msg::Point point;
            point.x = x;
            point.y = y;
            point.z = 0.0;
            return point;
        }
        
        private:
            double accuracy = 0.01;
    };
} // namespace offline_planner
