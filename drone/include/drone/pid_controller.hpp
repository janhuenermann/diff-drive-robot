#include "ros/ros.h"
#include <math/vector2.hpp>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

class PIDController
{
public:

    PIDController();

    void setParameters_freq(double freq);
    void setParameters_height(double p,double i, double d);
    void setParameters_pos(double p,double i, double d);

    void set_true_pos(geometry_msgs::Point pos);
    void set_true_angles(geometry_msgs::Quaternion q);

    void set_target_pos(geometry_msgs::Point target);

    void update(double &vel_x, double &vel_y,double &vel_z);

    void publish_e(double ex, double ey, double ez);
    void publish_int(double ix, double iy, double iz);

    inline double getTimeDelta()
    {
        return prev_time_delta;
    }

protected:
    const double maxv_pos = 2;
    const double maxv_height = 1;
    const double minv_height = -0.5;
    const double anti_windup = 0.5;
    const double threshold_ss = 0.5;

    double dt;
    double p_height,i_height, d_height;
    double p_pos,i_pos, d_pos;

    Point2 goal_pos;
    double goal_height;
    Point2 drone_pos;
    double drone_height;
    geometry_msgs::Quaternion drone_q;
    bool height_saturated;

    double max_int_pos;
    double max_int_height;
    double pos_integrated_x=0;
    double pos_integrated_y=0;
    double height_integrated=0;
    Point2 e_prev_pos;
    double e_prev_height;

    double prev_time_delta;
    ros::Time prev_update_time;

    ros::Publisher pub_err;
    ros::Publisher pub_int;
};
