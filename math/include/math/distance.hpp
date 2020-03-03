#ifndef DISTANCE_H
#define DISTANCE_H

#include <cmath>
#include <iostream>

#include <math/constants.hpp>

struct Distance
{
public:
    double ordinals;
    double cardinals;
    double sum;

    Distance() : Distance(0,0)
    {}

    Distance(double ordinals, double cardinals) :
        ordinals(ordinals),
        cardinals(cardinals)
    {
        sum = ordinals * M_SQRT2 + cardinals;
    }

    Distance operator-() const {
        return Distance(-ordinals, -cardinals);
    }

    Distance& operator= (const double &value)
    {
        ordinals = 0;
        cardinals = value;

        return *this;
    }

    static Distance octileDistance(Index2 a, Index2 b)
    {
        double di = static_cast<double>(std::abs(a.y - b.y));
        double dj = static_cast<double>(std::abs(a.x - b.x));

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

namespace std {
    template<> class numeric_limits<Distance> {
    public:
       static Distance infinity() { return Distance(infd, infd); };
    };
}

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

inline std::ostream& operator<<(std::ostream &strm, const Distance &a) {
  return strm << "Distance(ordinal: " << a.ordinals << ", cardinal: " << a.cardinals << ")";
}

#endif