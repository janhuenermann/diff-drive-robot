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
    void set_true_vel(geometry_msgs::Vector3 vel);

    void set_target_pos(geometry_msgs::Point target);

    void update(double &vel_x, double &vel_y,double &vel_z);

    void set_phase(int phase);

    void publish_e(double ex, double ey, double ez);
    void publish_int(double ix, double iy, double iz);

protected:
    const double Kp_xy = 0.8, Ki_xy = 0.0, Kd_xy = 0.0;

    // Flight to stationary target
    const double kp_xy = Kp_xy, ki_xy = Ki_xy, kd_xy = Kd_xy;
    const double kp_z = 3, ki_z = 0.7, kd_z = 0;
    // Flight to TURTLE
    const double tp_xy = Kp_xy, ti_xy = Ki_xy, td_xy = Kd_xy;
    const double tp_z = 3, ti_z = 0.7, td_z = 0;
    // Start and landing
    const double lp_xy = Kp_xy, li_xy = Ki_xy, ld_xy = Kd_xy;
    const double lp_z = 5, li_z = 0, ld_z = 0.2;
    // final landing
    const double lap_xy = Kp_xy, lai_xy = Ki_xy, lad_xy = Kd_xy;
    const double lap_z = 0.2, lai_z = 0.05, lad_z = 0;

    // max params
    const double maxv_xy = 1.6;
    const double maxv_z = 0.9;
    const double minv_z = -0.4;
    const double threshold_ss = 0.5;
    const double windup_xy = 0.5;
    const double windup_z = 0.2;
    const double a_max = 40;

    double dt;
    double p_xy, i_xy, d_xy;
    double p_z, i_z, d_z;
    int curr_state=0;

    Point2 goal_xy;
    double goal_z;
    Point2 drone_xy;
    double drone_z;
    Point2 drone_v_xy;
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
