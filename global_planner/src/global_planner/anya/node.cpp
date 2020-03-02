#include <global_planner/anya.hpp>

using namespace ANYA;

Node::Node(Interval interval, Point2 root, Node *parent, bool start) :
    interval(interval),
    root(root),
    parent(parent),
    start(start),
    visited(false),
    g_cost(0)
{
}

void Node::update(Point2 target)
{
    if (parent != nullptr)
    {
        g_cost = parent->g_cost + (root - parent->root).norm<double>();
    }
    else
    {
        g_cost = 0;
    }

    h_cost = calculateHCost(target);
    f_cost = g_cost + h_cost;
}

double Node::calculateHCost(Point2 target)
{
    int iy = interval.row;
    double ixa = interval.a;
    double ixb = interval.b;

    double tx = target.x;
    double ty = target.y;

    double sx = root.x;
    double sy = root.y;

    if (sy < iy && ty < iy)
    {
        ty += 2 * (iy - ty);
    }
    else if (sy > iy && ty > iy)
    {
        ty -= 2 * (ty - iy);
    }

    double start_to_interval = std::abs(sy - iy);
    double interval_to_target = std::abs(iy - ty);

    double left_space = sx - ixa;
    double right_space = ixb - sx;

    double left_proj = ixa - interval_to_target * (left_space / start_to_interval);
    double right_proj = ixb + interval_to_target * (right_space / start_to_interval);

    double parent_cost = parent != nullptr ? parent->g_cost : 0;

    if (tx <= left_proj)
    {
        // pass through left end point
        return norm(sx, sy, ixa, (double)iy) + norm(ixa, (double)iy, tx, ty);
    }
    else if (tx >= right_proj)
    {
        // pass through right end point
        return norm(sx, sy, ixb, (double)iy) + norm(ixb, (double)iy, tx, ty);
    }

    // pass straight
    return norm(sx, sy, tx, ty);
}