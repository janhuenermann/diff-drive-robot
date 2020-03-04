#include <local_planner/pid_controller.hpp>

void PIDController::setParameters(double freq, Vec2 p, Vec2 i, Vec2 d)
{
    dt_ = 1.0 / freq;
    p_ = p;
    i_ = i * dt_;
    d_ = d / dt_;
}

void PIDController::setTruePosition(Point2 pos)
{
    position_true_ = pos;
}

void PIDController::setTrueAngle(double theta)
{
    angle_true_ = theta;
}

void PIDController::setTargetPosition(Point2 target)
{
    position_target_ = target;
}

void PIDController::update(double &linear_vel, double &angular_vel)
{
    prev_time_delta_ = (ros::Time::now() - prev_update_time_).toSec();

    Vec2 delta = position_target_ - position_true_;

    double angle_target = delta.atan2();
    double err_pos = delta.norm<double>();
    double err_ang = angle_target - angle_true_;

    if (err_ang >= M_PI)
    {
        err_ang -= 2 * M_PI;
    }
    else if (err_ang < -M_PI)
    {
        err_ang += 2 * M_PI;
    }

    Vec2 err(err_pos, err_ang);

    // ROS_INFO("dx: %.3lf, dy: %.3lf", delta.x, delta.y);
    // ROS_INFO("Error: %.4lf; %.4lf; %.4lf", angle_target, angle_true_, err_ang);

    Vec2 p_term = p_ * err;
    Vec2 i_term = i_ * integrated_err_;
    Vec2 d_term = d_ * (err - prev_err_);

    Vec2 action = p_term + i_term + d_term;
    linear_vel = action.x; // std::max(action.x - 0.5 * err_ang * err_ang, 0.0);
    angular_vel = action.y;

    integrated_err_ += err;
    prev_err_ = err;

    prev_update_time_ = ros::Time::now();
}