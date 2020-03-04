#ifndef VECTOR2_H
#define VECTOR2_H

#include <cmath>
#include <iostream>

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
        return std::sqrt(sq_norm<V>());
    }

    template<class V>
    inline V sq_norm()
    {
        return static_cast<V>(x*x + y*y);
    }

    template<class V>
    inline Vector2<V> round()
    {
        return Vector2<V>(static_cast<V>(std::round(x)), static_cast<V>(std::round(y)));
    }

    inline Vector2<T>& operator += (const Vector2<T>& v)
    {
        x += v.x;
        y += v.y;

        return *this;
    };

    inline Vector2<T>& operator += (const T& v)
    {
        x += v;
        y += v;

        return *this;
    };

    inline Vector2<T>& operator -= (const Vector2<T>& v)
    {
        x -= v.x;
        y -= v.y;

        return *this;
    };

    inline Vector2<T>& operator -= (const T& v)
    {
        x -= v;
        y -= v;

        return *this;
    };

    inline Vector2<T> rotate(double theta)
    {
        return Vector2<T>(std::cos(theta) * x - std::sin(theta) * y, std::sin(theta) * x + std::cos(theta) * y);
    }

    inline double atan2()
    {
        return std::atan2(y, x);
    };

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
inline Vector2<T> operator *(const Vector2<T>& lhs, const T& rhs)
{
    return Vector2<T>(lhs.x * rhs, lhs.y * rhs);
}

template<class T>
inline Vector2<T> operator /(const Vector2<T>& lhs, const Vector2<T>& rhs)
{
    return Vector2<T>(lhs.x / rhs.x, lhs.y / rhs.y);
}

template<class T>
inline Vector2<T> operator /(const Vector2<T>& lhs, const T& rhs)
{
    return Vector2<T>(lhs.x / rhs, lhs.y / rhs);
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
inline std::ostream& operator<<(std::ostream &strm, const Vector2<T> &a)
{
  return strm << "Vector2(" << a.x << ", " << a.y << ")";
}

typedef Vector2<int> Index2;
typedef Vector2<double> Point2;
typedef Vector2<double> Vec2;



#endif