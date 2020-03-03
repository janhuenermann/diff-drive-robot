#include <local_planner/trajectory.hpp>


void Trajectory::update(std::vector<Point2> path, Point2 robot_pos, Point2 robot_vel)
{
    if (spline_ != nullptr)
    {
        delete spline_;
    }

    s_ = 0.0; // pretend to be at the beginning of our new spline, for calculating the step size
    has_path_ = true;
    at_end_ = false;
    path_ = path;
    spline_ = SplinePath::fitCardinal(0.0, path, robot_vel, Point2(0, 0));

    // figure out our next point on the spline
    double ds = getStepSize();

    if (n_ > 0)
    {
        // avoid drift if we update the trajectory often
        s_ = (robot_pos - next_point_).norm<double>();
        s_ = std::max(0.0, std::min(ds, s_));
    }
    else
    {
        // first spline, set to step size
        s_ = ds;
    }

    next(false);
}

void Trajectory::next(bool move)
{
    if (!has_path_)
    {
        throw std::invalid_argument("trajectory does not contain path");
    }

    if (at_end_)
    {
        return ;
    }

    if (move)
    {
        s_ += getStepSize();
    }

    if (s_ > spline_->length)
    {
        s_ = spline_->length;
        at_end_ = true;
        reset();
    }

    next_point_ = spline_->position(s_);
    n_++;
}

void Trajectory::reset()
{
    n_ = 0;
}

double Trajectory::getStepSize()
{
    // TODO: v as function of spline derivative magnitude
    return v_ * dt_;
}

double Trajectory::getRemainingPathLength()
{
    if (!has_path_)
    {
        return 0.0;
    }

    return spline_->length - s_;
}