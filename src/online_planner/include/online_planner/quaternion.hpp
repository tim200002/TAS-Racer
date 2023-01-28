#pragma once
#include "geometry_msgs/msg/quaternion.hpp"
#include <cmath>


    struct Quaternion
    {
         double x,y,z,w;

        Quaternion(double x, double y, double z, double w):x(x),y(y),z(z),w(w){}
        
        static Quaternion from_euler(double roll, double pitch, double yaw){
            double x = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - 
            cos(roll/2) * sin(pitch/2) * sin(yaw/2);
            double y = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + 
                sin(roll/2) * cos(pitch/2) * sin(yaw/2);
            double z = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - 
                sin(roll/2) * sin(pitch/2) * cos(yaw/2);
            double w = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + 
                sin(roll/2) * sin(pitch/2) * sin(yaw/2);
            return Quaternion(x,y,z,w);       
        }

        inline std::ostream &operator<<(std::ostream & _stream)
        {
            // print something from v to str, e.g: Str << v.getX();
            _stream << "(" << x << "," << y << "," << z << "," << w <<")";
            return _stream;
        }

         bool operator==(Quaternion& p){
            return std::abs(p.x - x) <= accuracy && std::abs(p.y - y) <= accuracy && std::abs(p.z - z) <= accuracy && std::abs(p.w - w) <= accuracy;
        }

         bool operator!=(Quaternion& p){
            return !(*this == p);
        }

        geometry_msgs::msg::Quaternion toRosQuaternion(){
            geometry_msgs::msg::Quaternion quaternion;
            quaternion.x = x;
            quaternion.y = y;
            quaternion.z = z;
            quaternion.w = w;
            return quaternion;
        }

        private:
            double accuracy = 0.01;
    };
    