#ifndef DISTANCE_H
#define DISTANCE_H

#include <limits>
#include <eigen3/Eigen/Dense>

#define INF_D std::numeric_limits<double>::infinity()


typedef Eigen::Vector2i Index;
typedef Eigen::Vector2d Point;

struct Distance
{
public:
    double ordinals;
    double cardinals;
    double sum;

    Distance(double ordinals, double cardinals) :
        ordinals(ordinals),
        cardinals(cardinals)
    {
        sum = ordinals * M_SQRT2 + cardinals;
    }

    static Distance octileDistance(Index a, Index b)
    {
        double di = static_cast<double>(std::abs(a.y() - b.y()));
        double dj = static_cast<double>(std::abs(a.x() - b.x()));

        if (dj > di)
        {
            return Distance(di, dj-di);
        }
        else
        {
            return Distance(dj, di-dj);
        }
    }

};

const Distance DISTANCE_INFINITY = Distance(INF_D, INF_D);

inline bool operator ==(const Distance& lhs, const Distance& rhs)
{
    return lhs.ordinals == rhs.ordinals && lhs.cardinals == rhs.cardinals;
}

inline bool operator < (const Distance& lhs, const Distance& rhs)
{
    return lhs.sum < rhs.sum;
}

inline bool operator >(const Distance& lhs, const Distance &rhs)
{
    return lhs.sum > rhs.sum;
}

inline bool operator <=(const Distance& lhs, const Distance &rhs)
{
    return lhs.sum < rhs.sum || (lhs.ordinals == rhs.ordinals && lhs.cardinals == rhs.cardinals);
}

inline bool operator >=(const Distance& lhs, const Distance &rhs)
{
    return lhs.sum > rhs.sum || (lhs.ordinals == rhs.ordinals && lhs.cardinals == rhs.cardinals);
}

inline Distance operator +(const Distance& lhs, const double& rhs)
{
    return Distance(lhs.ordinals, lhs.cardinals + rhs);
}

inline Distance operator +(const Distance& lhs, const Distance& rhs)
{
    return Distance(lhs.ordinals + rhs.ordinals, lhs.cardinals + rhs.cardinals);
}

inline Distance operator -(const Distance& lhs, const double& rhs)
{
    return Distance(lhs.ordinals, lhs.cardinals - rhs);
}

inline Distance operator -(const Distance& lhs, const Distance& rhs)
{
    return Distance(lhs.ordinals - rhs.ordinals, lhs.cardinals - rhs.cardinals);
}

#endif
