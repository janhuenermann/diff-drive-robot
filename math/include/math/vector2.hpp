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

    inline Vector2<T> abs() const
    {
        return Vector2<T>(std::abs(x), std::abs(y));
    }

    inline Vector2<T> sgn() const
    {
        return Vector2<T>(copysign(1, x), copysign(1, y));
    }

    template<class V>
    inline Vector2<V> cast() const
    {
        return Vector2<V>(static_cast<V>(x), static_cast<V>(y));
    }

    template<class V>
    inline V norm() const
    {
        return std::sqrt(sq_norm<V>());
    }

    template<class V>
    inline V sq_norm() const
    {
        return static_cast<V>(x*x + y*y);
    }

    inline Vector2<int> floor() const
    {
        return Vector2<int>((int)(std::floor(x)), (int)(std::floor(y)));
    }

    inline Vector2<int> ceil() const
    {
        return Vector2<int>((int)(std::ceil(x)), (int)(std::ceil(y)));
    }

    inline Vector2<int> round() const
    {
        return Vector2<int>((int)(std::round(x)), (int)(std::round(y)));
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

    static inline Vector2<T> unitCircle(T theta)
    {
        return Vector2<T>(std::cos(theta), std::sin(theta));
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
inline Vector2<T> operator *(const Vector2<T>& lhs, const T& rhs)
{
    return Vector2<T>(lhs.x * rhs, lhs.y * rhs);
}

template<class T>
inline Vector2<T> operator *(const T& lhs, const Vector2<T>& rhs)
{
    return rhs * lhs;
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
inline bool operator <(const Vector2<T>& lhs, const Vector2<T>& rhs)
{
    return lhs.y == rhs.y ? lhs.x < rhs.x : lhs.y < rhs.y;
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