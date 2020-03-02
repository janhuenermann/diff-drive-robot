#include <local_planner/spline.hpp>
#include <cassert>
#include <array>

void findCoefficientsQuinticHermiteSpline(double *coeffs,
                                          double a, double da, double dda,
                                          double b, double db, double ddb)
{
    // t_f = 1.0
    coeffs[0] = a;
    coeffs[1] = da;
    coeffs[2] = 0.5 * dda;
    coeffs[3] = -10 * a - 6 * da - 1.5 * dda + 10 * b - 4 * db + 0.5 * ddb;
    coeffs[4] =  15 * a + 8 * da + 1.5 * dda - 15 * b + 7 * db - 1.0 * ddb;
    coeffs[5] = - 6 * a - 3 * da - 0.5 * dda +  6 * b - 3 * db + 0.5 * ddb;
}

double interpolateQuinticHermiteSpline(double t, const double coeffs[6])
{
    const double t1 = t;
    const double t2 = t * t1;
    const double t3 = t * t2;
    const double t4 = t * t3;
    const double t5 = t * t4;

    return coeffs[0] + coeffs[1] * t1 + coeffs[2] * t2 + coeffs[3] * t3 + coeffs[4] * t4 + coeffs[5] * t5;
}

double derivativeQuinticHermiteSpline(double t, const double coeffs[6])
{
    const double t1 = t;
    const double t2 = t * t1;
    const double t3 = t * t2;
    const double t4 = t * t3;

    return coeffs[1] + 2 * coeffs[2] * t1 + 3 * coeffs[3] * t2 + 4 * coeffs[4] * t3 + 5 * coeffs[5] * t4;
}

double derivative2QuinticHermiteSpline(double t, const double coeffs[6])
{
    const double t1 = t;
    const double t2 = t * t1;
    const double t3 = t * t2;

    return 2 * coeffs[2] + 6 * coeffs[3] * t1 + 12 * coeffs[4] * t2 + 20 * coeffs[5] * t3;
}

Spline Spline::calculate(Point2 i, Point2 f, Point2 di, Point2 df, Point2 ddi, Point2 ddf)
{
    double a[6], b[6];

    findCoefficientsQuinticHermiteSpline(a, i.x, di.x, ddi.x, f.x, df.x, ddf.x);
    findCoefficientsQuinticHermiteSpline(b, i.y, di.y, ddi.y, f.y, df.y, ddf.y);

    return Spline(a, b);
}


Point2 Spline::position(double t) const
{
    assert(t >= 0.0 && t <= 1.0);

    return Point2(
        interpolateQuinticHermiteSpline(t, a),
        interpolateQuinticHermiteSpline(t, b)
    );
}

Point2 Spline::velocity(double t) const
{
    assert(t >= 0.0 && t <= 1.0);

    return Point2(
        derivativeQuinticHermiteSpline(t, a),
        derivativeQuinticHermiteSpline(t, b)
    );
}

Point2 Spline::acceleration(double t) const
{
    assert(t >= 0.0 && t <= 1.0);

    return Point2(
        derivative2QuinticHermiteSpline(t, a),
        derivative2QuinticHermiteSpline(t, b)
    );
}

// { weight, abscissa }
const std::vector<std::array<double, 2>> GAUSS_LENGENDRE_COEFFICIENTS = {
   { 0.4179591836734694, 0.5 * (1.0 + 0.0000000000000000) },
   { 0.3818300505051189, 0.5 * (1.0 + 0.4058451513773972) },
   { 0.3818300505051189, 0.5 * (1.0 + -0.4058451513773972) },
   { 0.2797053914892766, 0.5 * (1.0 + -0.7415311855993945) },
   { 0.2797053914892766, 0.5 * (1.0 + 0.7415311855993945) },
   { 0.1294849661688697, 0.5 * (1.0 + -0.9491079123427585) },
   { 0.1294849661688697, 0.5 * (1.0 + 0.9491079123427585) }
};

void Spline::calculateLength()
{
    // 7-point Gauss-Quadrature for solving length integral
    length = 0.0;

    for (auto &coeff : GAUSS_LENGENDRE_COEFFICIENTS)
    {
        length += velocity(coeff[1]).norm<double>() * coeff[0];
    }

    length *= 0.5;
}

const Spline& SplinePath::select(double ss, double &t) const
{
    assert(ss >= 0.0 && ss <= length);

    double _ss = ss;

    for (const Spline &sp : children)
    {
        if (_ss - sp.length < 0.0)
        {
            t = ss / sp.length;
            return sp;
        }

        _ss -= sp.length;
    }

    throw std::invalid_argument("ss too big.");
}

Point2 SplinePath::position(double ss) const
{
    double t;
    return select(ss, t).position(t);
}

Point2 SplinePath::velocity(double ss) const
{
    double t;
    return select(ss, t).velocity(t);
}

Point2 SplinePath::acceleration(double ss) const
{
    double t;
    return select(ss, t).acceleration(t);
}

SplinePath *SplinePath::fitCardinal(double c, std::vector<Point2> pts, Point2 dstart, Point2 dend, double lower_limit_len)
{
    assert(c >= 0.0 && c <= 1.0);
    assert(pts.size() >= 2);

    const bool has_lower_limit_len = !std::isnan(lower_limit_len);
    const Point2 dd_const(0, 0);
    const double mult = ((1 - c) / 2);

    Point2 i, f;
    Point2 di = dstart; // tangents
    Point2 df;

    // we generate N-1 splines
    SplinePath *out = new SplinePath();

    for (int j = 1; j < pts.size(); ++j)
    {
        if (j < pts.size() - 1)
        {
            Point2 pm1 = pts[j-1]; // p_{j-1}
            Point2 pp1 = pts[j+1]; // p_{j+1}

            df = (pp1 - pm1) * mult;
        }
        else
        {
            df = dend;
        }

        i = pts[j-1];
        f = pts[j];

        Spline s = Spline::calculate(i, f, di, df, dd_const, dd_const);

        out->length += s.length;
        out->children.push_back(s);

        if (has_lower_limit_len && out->length > lower_limit_len)
        {
            break ;
        }

        di = df;
    }

    return out;
}