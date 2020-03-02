#include <global_planner/anya.hpp>

using namespace ANYA;

bool Interval::contains(Point2 p)
{
    return (row == (int)p.y) && (p.x >= a - epsilon) && (p.x <= b + epsilon);
}