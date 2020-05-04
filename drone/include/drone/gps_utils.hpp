#include "ros/ros.h"

#include <drone/math.hpp>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <vector>

class GPS
{
public:
    static Vec3 convertMeasurement(const sensor_msgs::NavSatFix &msg, const Vec3 &initial_ecef, const Vec3 &origin);
    static Vec3 GPS2ECEF(const sensor_msgs::NavSatFix &msg);
};
