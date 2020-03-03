#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "ros/ros.h"

#include <global_planner/math.hpp>
#include <local_planner/spline.hpp>

class Trajectory
{

public:

    Trajectory(double v = 0.0, double F = 1.0) : n_(0), s_(0.0), at_end_(true), has_path_(false)
    {
        setVelocity(v);
        setFrequency(F);
    }

    inline bool hasPath()
    {
        return has_path_;
    };

    inline Point2 getNextTargetPoint()
    {
        return next_point_;
    };

    inline void setFrequency(double F)
    {
        dt_ = 1.0 / F;
    };

    inline void setVelocity(double v)
    {
        v_ = v;
    };

    double getRemainingPathLength();

    void next(bool move = true);
    void update(std::vector<Point2> path, Point2 robot_pos, Point2 robot_vel);

protected:

    Point2 next_point_;
    
    double dt_;
    double v_;

    SplinePath *spline_;
    int n_;
    double s_;
    bool at_end_;
    bool has_path_;

    std::vector<Point2> path_;

    double getStepSize();
    void reset();

};

#endif