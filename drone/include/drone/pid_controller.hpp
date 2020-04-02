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

    void set_true_pos(geometry_msgs::Point pos);
    void set_true_angles(geometry_msgs::Quaternion q);

    void set_target_pos(geometry_msgs::Point target);

    void update(double &vel_x, double &vel_y,double &vel_z);

    void set_phase(int phase);

    void publish_e(double ex, double ey, double ez);
    void publish_int(double ix, double iy, double iz);

protected:
    // Flight to stationary target
    const double kp_xy = 1.3, ki_xy = 0, kd_xy = 0.1;
    const double kp_z = 10, ki_z = 0.7, kd_z = 2.5;
    // Flight to TURTLE
    const double tp_xy = 1.7, ti_xy = 0.1, td_xy = 0.3;
    const double tp_z = 10, ti_z = 0.7, td_z = 2.5;
    // Start and landing
    const double lp_xy = 1, li_xy = 0.7, ld_xy = 0;
    const double lp_z = 8, li_z = 0, ld_z = 2;

    // max params
    const double maxv_xy = 2;
    const double maxv_z = 1;
    const double minv_z = -0.5;
    const double threshold_ss = 0.5;
    const double windup_xy = 0.5;
    const double windup_z = 0.2;

    double dt;
    double p_xy, i_xy, d_xy;
    double p_z, i_z, d_z;
    int curr_state=0;

    Point2 goal_xy;
    double goal_z;
    Point2 drone_xy;
    double drone_z;
    geometry_msgs::Quaternion drone_q;

    double integrated_x=0;
    double integrated_y=0;
    double integrated_z=0;
    Point2 e_prev_xy;
    double e_prev_z;
    bool saturated_z = false;

    ros::Time prev_update_time;

    ros::Publisher pub_err;
    ros::Publisher pub_int;
};
