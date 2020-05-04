#include "drone/gps_utils.hpp"


Vec3 GPS::convertMeasurement(const sensor_msgs::NavSatFix &msg, const Vec3 &initial_ecef, const Vec3 &origin)
{
    const double initial_compass = 0;

    // Convert gps message to map coordinate system
    double longitude = msg.longitude*M_PI/180.0;
    double latitude = msg.latitude*M_PI/180.0;
    double altitude = msg.altitude;

    Eigen::Matrix3d Ren;
    Eigen::Matrix3d Rmn;

    Ren <<  -sin(latitude)*cos(longitude), -sin(longitude), -cos(latitude)*cos(longitude),
            -sin(latitude)*sin(longitude),  cos(longitude), -cos(latitude)*sin(longitude),
                            cos(latitude),               0,                -sin(latitude);

    Rmn << cos(initial_compass),  sin(initial_compass),  0,
           sin(initial_compass), -cos(initial_compass),  0,
                              0,                     0, -1;

    Vec3 curr_ecef = GPS2ECEF(msg);
    Vec3 ned = Ren.transpose() * (curr_ecef - initial_ecef);

    return Rmn * (ned - origin);
}

Vec3 GPS::GPS2ECEF(const sensor_msgs::NavSatFix &msg)
{
    // Convert gps messages to ecef system
    double longitude = msg.longitude*M_PI/180.0;
    double latitude = msg.latitude*M_PI/180.0;
    double altitude = msg.altitude;

    double a = 6378137;
    double b = 6356752;
    double e_sq = 1-b*b/(a*a);
    double N = a/sqrt(1-e_sq*pow(sin(latitude),2));

    Vec3 ecef;
    ecef.x() = (N+altitude)*cos(latitude)*cos(longitude);
    ecef.y() = (N+altitude)*cos(latitude)*sin(longitude);
    ecef.z() = (N*b*b/(a*a)+altitude)*sin(latitude);

    return ecef;
}
