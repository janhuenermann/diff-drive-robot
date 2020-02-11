#ifndef DISTANCE_H
#define DISTANCE_H

#include <limits>

#define INF_D std::numeric_limits<double>::infinity()

struct Index
{
    int x, y;

    Index(int x, int y) : x(x), y(y) {}

};

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


    Distance operator +(const Distance &rhs)
    {
        return Distance(ordinals + rhs.ordinals, cardinals + rhs.cardinals);
    }

    Distance operator -(const Distance &rhs)
    {
        return Distance(ordinals - rhs.ordinals, cardinals - rhs.cardinals);
    }

    static Distance octileDistance(Index a, Index b)
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

    static Distance infinity()
    {
        return Distance(INF_D, INF_D);
    }

};


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

inline bool operator ==(const Index& lhs, const Index& rhs)
{
    return lhs.x == rhs.x && lhs.y == rhs.y;
}

#endif
