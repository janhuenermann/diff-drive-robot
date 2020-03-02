#include "ros/ros.h"

#include <global_planner/math.hpp>
#include <local_planner/spline.hpp>

class Trajectory
{

public:

    Trajectory(double step_size) :
        step_size_(step_size)
    {
    }

    inline Point2 getTargetPoint()
    {
        return next_point_;
    };

    void next();
    void update(std::vector<Point2> path, Point2 robot_pos, Point2 robot_vel);

protected:

    Point2 next_point_;
    double step_size_;
    double s_;

    SplinePath *spline_;
    std::vector<Point2> path_;

    double getStepSize();
    void generate();

};