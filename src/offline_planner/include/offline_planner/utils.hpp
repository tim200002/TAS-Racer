#pragma once
#include "geometry_msgs/msg/quaternion.hpp"
#include <cmath>

namespace offline_planner
{
    class Utils
    {
    public:
        static geometry_msgs::msg::Quaternion quaternionFromEuler(double roll, double pitch, double yaw)
        {
            double qx = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
            double qy = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) +
                        sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
            double qz = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) -
                        sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
            double qw = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) +
                        sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);

            geometry_msgs::msg::Quaternion quaternion;
            quaternion.set__x(qx);
            quaternion.set__y(qy);
            quaternion.set__z(qz);
            quaternion.set__w(qw);

            return quaternion;
        }

        static double yawFromQuaternion(const geometry_msgs::msg::Quaternion &quaternion)
        {
            return atan2(2.0 * (quaternion.y * quaternion.z + quaternion.w * quaternion.x), pow(quaternion.w, 2) - pow(quaternion.x, 2) - pow(quaternion.y, 2) - pow(quaternion.z, 2));
        }
    };
} // namespace offline_planner
