#include <local_planner/trajectory.hpp>


void Trajectory::update(std::vector<Point2> path, Point2 robot_vel)
{
    if (spline_ != nullptr)
    {
        delete spline_;
        spline_ = nullptr;
    }

    if (path.size() < 2)
    {
        s_ = 0;
        has_path_ = false;
        is_at_end_ = true;
        return ;
    }

    spline_ = SplinePath::fitCardinal(1.0, path, robot_vel * 0.5, Point2(0, 0));
    
    s_ = dist_;
    is_at_end_ = false;
    has_path_ = true;
    updateCurrentPoint();
}

Point2 Trajectory::nextPoint(Point2 robot_pos)
{
    assert(hasPath());

    while (!isAtEnd() && (current_point_ - robot_pos).norm<double>() < dist_)
    {
        s_ += step_size_;
        updateCurrentPoint();
    }

    return current_point_;
}

void Trajectory::updateCurrentPoint()
{
    if (s_ >= spline_->length)
    {
        is_at_end_ = true;
        s_ = spline_->length;
    }

    current_point_ = spline_->position(s_);
}

double Trajectory::getRemainingPathLength()
{
    if (!has_path_)
    {
        return 0.0;
    }

    return spline_->length - s_;
}