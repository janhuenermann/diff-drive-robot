#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "ros/ros.h"

#include <math/vector2.hpp>
#include <math/spline.hpp>

// fit and follow a spline
// (try to) keep constant distance to point that is controlled
class Trajectory
{

public:

    Trajectory() : s_(0.0), is_at_end_(true), has_path_(false)
    {
    }

    inline void setDistance(double dist) { dist_ = dist; }
    inline void setStepSize(double step_size) { step_size_ = step_size; }

    inline bool hasPath()
    {
        return has_path_;
    };

    inline SplinePath *getSplinePath()
    {
        assert(hasPath());
        return spline_;
    }

    inline bool isAtEnd()
    {
        return is_at_end_;
    }

    // gets next point on trajectory that is at least dist_ distance away
    // if not at the end of the trajectory
    Point2 nextPoint(Point2 robot_pos);

    // update trajectory
    void update(std::vector<Point2> path, Point2 robot_vel);

    // returns the length of the path after the current point
    double getRemainingPathLength();

protected:

    Point2 current_point_;
    
    double dist_;
    double step_size_;

    SplinePath *spline_;
    double s_;
    bool has_path_;
    bool is_at_end_;

    std::vector<Point2> path_;

    // double getDistance();
    void updateCurrentPoint();

};

#endif