#include <local_planner/trajectory.hpp>


void Trajectory::update(std::vector<Point2> path, Point2 robot_pos, Point2 robot_vel);
{
    path_ = path;

    if (spline_ != nullptr)
    {
        delete spline_;

        // avoid drift if we update the trajectory often
        double ds = getStepSize();
        double rem = (robot_pos - next_point_).norm<double>();

        s_ = std::max(0.0, std::min(ds, rem - ds));
    }
    else
    {
        s_ = 0.0;
    }

    spline_ = SplinePath::fitCardinal(0.0, path, robot_vel, Point2(0, 0));

    next();
}

void Trajectory::next()
{
    s_ += getStepSize();
    next_point_ = spline_->position(s_);
}

double Trajectory::getStepSize()
{
    return step_size_;
}