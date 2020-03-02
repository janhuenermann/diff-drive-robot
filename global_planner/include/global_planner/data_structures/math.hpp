#ifndef DISTANCE_H
#define DISTANCE_H

/**
 * Author: Jan Huenermann <jan@huenermann.de>
 */

#include <limits>
#include <execinfo.h>


#define infd std::numeric_limits<double>::infinity()

/**
 * Vector2
 */

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


/**
 * FRACTION
 */

template<class T>
struct Fraction
{
    T n;
    T d;

    Fraction(T n) : n(n), d(1)
    {}

    Fraction(T n, T d)
    {
        if (d < 0)
        {
            n = -n;
            d = -d;
        }

        int gcd = calculate_gcd(n, d);
        
        this->n = n / gcd;
        this->d = d / gcd;
    }

    Fraction<T>& operator= (const T &value)
    {
        n = value;
        d = 1;

        return *this;
    }

    bool isWholeNumber()
    {
        return d == 1;
    }

    T floor()
    {
        if (d == 1)
        {
            return n;
        }

        if (n > 0)
        {
            return n/d;
        }
        else
        {
            return (n+1)/d - 1;
        }
    }

    T ceil()
    {
        if (d == 1)
        {
            return n;
        }

        if (n > 0)
        {
            return (n-1)/d + 1;
        }
        else
        {
            return n/d;
        }
    }

    template<class V>
    V cast()
    {
        return (V)(double)(n)/(double)(d);
    }

    operator double() const { return (double)(n)/(double)(d); }

};

template<class T>
inline bool operator <(const Fraction<T>& lhs, const Fraction<T>& rhs)
{
    return lhs.n * rhs.d - rhs.n * lhs.d < 0;
}

template<class T>
inline bool operator <=(const Fraction<T>& lhs, const Fraction<T>& rhs)
{
    return lhs.n * rhs.d - rhs.n * lhs.d <= 0;
}

template<class T>
inline bool operator >(const Fraction<T>& lhs, const Fraction<T>& rhs)
{
    return lhs.n * rhs.d - rhs.n * lhs.d > 0;
}

template<class T>
inline bool operator >=(const Fraction<T>& lhs, const Fraction<T>& rhs)
{
    return lhs.n * rhs.d - rhs.n * lhs.d >= 0;
}

template<class T>
inline bool operator <(const Fraction<T>& lhs, const T& rhs)
{
    return lhs.n < rhs * lhs.d;
}

template<class T>
inline bool operator <=(const Fraction<T>& lhs, const T& rhs)
{
    return lhs.n <= rhs * lhs.d;
}

template<class T>
inline bool operator >(const Fraction<T>& lhs, const T& rhs)
{
    return lhs.n > rhs * lhs.d;
}

template<class T>
inline bool operator >=(const Fraction<T>& lhs, const T& rhs)
{
    return lhs.n >= rhs * lhs.d;
}

template<class T>
inline Fraction<T> operator *(const Fraction<T>& lhs, const Fraction<T>& rhs)
{
    return Fraction<T>(lhs.n * rhs.n, lhs.d * rhs.d);
}

template<class T>
inline Fraction<T> operator *(const Fraction<T>& lhs, const T& rhs)
{
    return Fraction<T>(lhs.n * rhs, lhs.d);
}

template<class T>
inline Fraction<T> operator /(const Fraction<T>& lhs, const Fraction<T>& rhs)
{
    return Fraction<T>(lhs.n * rhs.d, lhs.d * rhs.n);
}

template<class T>
inline Fraction<T> operator /(const Fraction<T>& lhs, const T& rhs)
{
    return Fraction<T>(lhs.n, lhs.d * rhs);
}

template<class T>
inline Fraction<T> operator +(const Fraction<T>& lhs, const Fraction<T>& rhs)
{
    return Fraction<T>(lhs.n * rhs.d + rhs.n * lhs.d, lhs.d * rhs.d);
}

template<class T>
inline Fraction<T> operator -(const Fraction<T>& lhs, const Fraction<T>& rhs)
{
    return Fraction<T>(lhs.n * rhs.d - rhs.n * lhs.d, lhs.d * rhs.d);
}

template<class T>
inline Fraction<T> operator +(const Fraction<T>& lhs, const T& rhs)
{
    return Fraction<T>(lhs.n + rhs * lhs.d, lhs.d);
}

template<class T>
inline Fraction<T> operator +(const T& lhs, const Fraction<T>& rhs)
{
    return rhs + lhs;
}

template<class T>
inline Fraction<T> operator -(const Fraction<T>& lhs, const T& rhs)
{
    return Fraction<T>(lhs.n - rhs * lhs.d, lhs.d);
}

template<class T>
inline Fraction<T> operator -(const T& lhs, const Fraction<T>& rhs)
{
    return Fraction<T>(rhs * lhs.d - lhs.n, lhs.d);
}

// Euclidean algorithm
inline int calculate_gcd(int a, int b)
{
    if (b == 0)
        return a;
    else
        return calculate_gcd(b, a % b);
}


/**
 * DISTANCE
 */

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

/**
 * Various methods
 */

template<class T>
T norm(T ax, T ay, T bx, T by)
{
    T dx = bx-ax;
    T dy = by-ay;
    return std::sqrt(dx*dx + dy*dy);
}

template<class T>
T sign(T a)
{
    if (a >= 0)
        return 1;
    else
        return -1;
}

inline void print_trace() {
    void* callstack[128];
    int i, frames = backtrace(callstack, 128);
    char** strs = backtrace_symbols(callstack, frames);
    for (i = 0; i < frames; ++i) {
        printf("%s\n", strs[i]);
    }
    free(strs);
}

#endif
