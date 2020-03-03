#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "ros/ros.h"

#include <math/vector2.hpp>

class PIDController
{
public:

    PIDController() : prev_update_time_(ros::Time(0))
    {
    }

    void setParameters(double freq, Vec2 p, Vec2 i, Vec2 d);

    void setTruePosition(Point2 pos);
    void setTrueAngle(double theta);

    void setTargetPosition(Point2 target);

    void update(double &linear_vel, double &angular_vel);

    inline double getTimeDelta()
    {
        return prev_time_delta_;
    }

protected:

    double dt_;
    Vec2 p_;
    Vec2 i_;
    Vec2 d_;

    Point2 position_target_;
    Point2 position_true_;
    double angle_true_;

    Vec2 integrated_err_;
    Vec2 prev_err_;

    double prev_time_delta_;
    ros::Time prev_update_time_;

};

#endif