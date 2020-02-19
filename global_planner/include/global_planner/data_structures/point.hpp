#ifndef DISTANCE_H
#define DISTANCE_H

/**
 * Author: Jan Huenermann <jan@huenermann.de>
 */

#include <limits>

#define infd std::numeric_limits<double>::infinity()

template<class T>
struct Vector2
{

    Vector2(T x, T y) : x(x), y(y)
    {}

    Vector2() : Vector2(0, 0)
    {}

    T x;
    T y;

    template<class V>
    inline Vector2<V> cast()
    {
        return Vector2<V>(static_cast<V>(x), static_cast<V>(y));
    }

    template<class V>
    inline V norm()
    {
        return std::sqrt(static_cast<V>(x*x + y*y));
    }

    template<class V>
    inline Vector2<V> round()
    {
        return Vector2<V>(static_cast<V>(std::round(x)), static_cast<V>(std::round(y)));
    }

};

template<class T>
inline Vector2<T> operator +(const Vector2<T>& lhs, const Vector2<T>& rhs)
{
    return Vector2<T>(lhs.x + rhs.x, lhs.y + rhs.y);
}

template<class T>
inline Vector2<T> operator -(const Vector2<T>& lhs, const Vector2<T>& rhs)
{
    return Vector2<T>(lhs.x - rhs.x, lhs.y - rhs.y);
}

template<class T>
inline Vector2<T> operator *(const Vector2<T>& lhs, const Vector2<T>& rhs)
{
    return Vector2<T>(lhs.x * rhs.x, lhs.y * rhs.y);
}

template<class T>
inline Vector2<T> operator /(const Vector2<T>& lhs, const Vector2<T>& rhs)
{
    return Vector2<T>(lhs.x / rhs.x, lhs.y / rhs.y);
}

template<class T>
inline bool operator ==(const Vector2<T>& lhs, const Vector2<T>& rhs)
{
    return lhs.x == rhs.x && lhs.y == rhs.y;
}

template<class T>
inline bool operator !=(const Vector2<T>& lhs, const Vector2<T>& rhs)
{
    return lhs.x != rhs.x || lhs.y != rhs.y;
}

template<class T>
std::ostream& operator<<(std::ostream &strm, const Vector2<T> &a) {
  return strm << "Vector2(" << a.x << ", " << a.y << ")";
}

typedef Vector2<int> Index2;
typedef Vector2<double> Point2;

/**
 * DISTANCE
 */

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

    Distance operator-() const {
        return Distance(-ordinals, -cardinals);
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

#endif
