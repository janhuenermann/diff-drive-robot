#include <drone_ekf/math.hpp>

using namespace drone_ekf;

const Mat<3,3> I3 = Mat<3,3>::Identity();

void drone_ekf::getSkewSymmetricMatrix(Ref<Mat<3, 3>> mat, const Vec<3> &v)
{
    mat <<   0.0, -v(2),   v(1),
            v(2),   0.0,  -v(0),
           -v(1),  v(0),    0.0;
}

// d(p*q)/dq
void drone_ekf::diffQuaternionProductWrtQ(Ref<Mat<4, 4>> dpq, const Quaternion &p)
{
    const Vec<4> &coeffs = p.coeffs();

    dpq.row(0) = coeffs * (-1.0);
    dpq.col(0) = coeffs;

    Ref<Mat<3,3>> ll3x3 = dpq.block<3, 3>(1, 1);
    getSkewSymmetricMatrix(ll3x3, p.vec());
    ll3x3 = ll3x3 + p.w() * I3;
}


// d(p*q)/dq
void drone_ekf::diffQuaternionProductWrtP(Ref<Mat<4, 4>> dpq, const Quaternion &q)
{
    const Vec<4> &coeffs = q.coeffs();

    dpq.row(0) = coeffs * (-1.0);
    dpq.col(0) = coeffs;

    Ref<Mat<3,3>> ll3x3 = dpq.block<3, 3>(1, 1);
    getSkewSymmetricMatrix(ll3x3, q.vec());
    ll3x3 = ll3x3 - q.w() * I3;
}