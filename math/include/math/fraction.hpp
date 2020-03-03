#ifndef FRACTION_H
#define FRACTION_H

#include <cmath>

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

#endif