#ifndef SPLINE_H
#define SPLINE_H

#include <vector>
#include <cstring>

#include <math/SplineData.h>
#include <math/SplinePathData.h>
#include <math/vector2.hpp>

struct Spline
{

    double a[6];
    double b[6];
    double length;

    Spline(const double a[6], const double b[6])
    {
        std::memcpy(this->a, a, sizeof(double) * 6);
        std::memcpy(this->b, b, sizeof(double) * 6);
        calculateLength();
    }

    Point2 position(double t) const;
    Point2 velocity(double t) const;
    Point2 acceleration(double t) const;

    math::SplineData toData() const;

    static Spline fromData(const math::SplineData &data);
    static Spline calculate(Point2 i, Point2 f, Point2 di, Point2 df, Point2 ddi, Point2 ddf);

protected:

    void calculateLength();

};

struct SplinePath
{
public:

    SplinePath() : length(0)
    {
    }

    double length;
    std::vector<Spline> children;

    // unnormalized functions take path length as argument
    Point2 position(double ss) const;
    Point2 velocity(double ss) const;
    Point2 acceleration(double ss) const;

    // returns individual spline along with t in that spline from global tt parameter.
    const Spline& select(double ss, double &t) const;

    Point2 normalizedPosition(double tt) const
    {
        return position(tt * length);
    }

    Point2 normalizedVelocity(double tt) const
    {
        return velocity(tt * length);
    }

    Point2 normalizedAcceleration(double tt) const
    {
        return acceleration(tt * length);
    }

    inline double normalize(double ss) const
    {
        return ss / length;
    }

    math::SplinePathData toData() const;

    static SplinePath *fromData(const math::SplinePathData &data);
    static SplinePath *fitCardinal(double c, std::vector<Point2> pts, Point2 dstart, Point2 dend, double lower_limit_len = NAN);
    
    inline static SplinePath *fitCatmullRom(std::vector<Point2> pts, Point2 dstart, Point2 dend)
    {
        return fitCardinal(0.0, pts, dstart, dend);
    }

};

#endif