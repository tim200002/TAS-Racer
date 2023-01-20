#pragma once
#include "geometry_msgs/msg/pose.hpp"
#include "utils.hpp"
#include "point.hpp"
#include "quaternion.hpp"
#include <sstream>  
#include <cmath>

    struct Pose
    {
         WorldPoint coordinate;
         Quaternion quaternion;

        Pose(WorldPoint coordinate, Quaternion quaternion):coordinate(coordinate), quaternion(quaternion){}
        

        Pose(geometry_msgs::msg::Pose pose):coordinate(WorldPoint(pose.position.x, pose.position.y)), quaternion(Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)){}

        inline std::ostream &operator<<(std::ostream & _stream)
        {
            // print something from v to str, e.g: Str << v.getX();
            _stream << "(" << coordinate.x << "," << coordinate.y << "," << quaternion.x << "," << quaternion.y << "," << quaternion.z << "," << quaternion.w << ")";
            return _stream;
        }

        std::string toString(){
            std::stringstream sstream ;
            sstream << "(" << coordinate.x << "," << coordinate.y << "," << quaternion.x << "," << quaternion.y << "," << quaternion.z << "," << quaternion.w << ")";
            return sstream.str();
        }


        bool operator==(Pose& p){
            return (p.coordinate == coordinate && p.quaternion == quaternion);
        }

         bool operator!=(Pose& p){
            return (p.coordinate != coordinate || p.quaternion != quaternion);
        }


        geometry_msgs::msg::Pose toRosPose(){
            geometry_msgs::msg::Pose pose;
            pose.position = coordinate.toRos2Point();
            pose.orientation = quaternion.toRosQuaternion();
            return pose;
        }

       
    };
    