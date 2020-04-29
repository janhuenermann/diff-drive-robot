#include <drone_ekf/math.hpp>

using namespace drone_ekf;

const Mat<4,4> I4 = Mat<4,4>::Identity();

Mat<3,3> drone_ekf::getSkewSymmetric(const Vec<3> &v)
{
    Mat<3,3> m;
    getSkewSymmetric(m, v);
    return m;
}

void drone_ekf::getSkewSymmetric(Ref<Mat<3, 3>> mat, const Vec<3> &v)
{
    mat <<   0.0, -v(2),   v(1),
            v(2),   0.0,  -v(0),
           -v(1),  v(0),    0.0;
}

// d(p*q)/dq
void drone_ekf::diffQuaternionProductWrtQ(Ref<Mat<4, 4>> dpq, const Quaternion &p)
{    
    dpq.block<3, 3>(0, 0) = I3 * p.w() + getSkewSymmetric(p.vec());
    dpq.block<1, 3>(3, 0) = -p.vec().transpose();
    dpq.block<3, 1>(0, 3) = p.vec();
    dpq(3, 3) = p.w();
}


// d(p*q)/dq
void drone_ekf::diffQuaternionProductWrtP(Ref<Mat<4, 4>> dpq, const Quaternion &q)
{
    dpq.block<3, 3>(0, 0) = I3 * q.w() - getSkewSymmetric(q.vec());
    dpq.block<1, 3>(3, 0) = -q.vec().transpose();
    dpq.block<3, 1>(0, 3) = q.vec();
    dpq(3, 3) = q.w();
}

// d(q' v q)/dq
void drone_ekf::diffQstarVQ_Q(Ref<Mat<3, 4>> d, const Quaternion &q, const Vec<3> &v)
{
    const double q1 = q.x(), q2 = q.y(), q3 = q.z(), q4 = q.w();
    const double v1 = v.x(), v2 = v.y(), v3 = v.z();
    
    d << 2*q1*v1 + 2*q2*v2 + 2*q3*v3, 2*q1*v2 - 2*q2*v1 - 2*q4*v3, 2*q1*v3 - 2*q3*v1 + 2*q4*v2, 2*q3*v2 - 2*q2*v3 + 2*q4*v1,
         2*q2*v1 - 2*q1*v2 + 2*q4*v3, 2*q1*v1 + 2*q2*v2 + 2*q3*v3, 2*q2*v3 - 2*q3*v2 - 2*q4*v1, 2*q1*v3 - 2*q3*v1 + 2*q4*v2,
         2*q3*v1 - 2*q1*v3 - 2*q4*v2, 2*q3*v2 - 2*q2*v3 + 2*q4*v1, 2*q1*v1 + 2*q2*v2 + 2*q3*v3, 2*q2*v1 - 2*q1*v2 + 2*q4*v3;
    // d.block<3, 3>(0, 0) = 2.0 * (2.0 * v.dot(q.vec()) * I3 - v * q.vec().transpose() + q.w() * getSkewSymmetric(v));
    // d.col(3) = 2.0 * (q.w() * v - q.vec().cross(v));
}

// d(q v q')/dq
// void drone_ekf::diffQVQstar_Q(Ref<Mat<3, 4>> d, const Quaternion &q, const Vec<3> v)
// {
//     Mat<3, 3> m;

//     getSkewSymmetric(m, v);
//     d.block<3,3>(0, 0).noalias() = 2.0 * (-v * q.vec().transpose() + v.dot(q.vec()) * Mat<3,3>::Identity() - q.w() * m);

//     getSkewSymmetric(m, q.vec());
//     d.col(3).noalias() = 2.0 * (q.w() * v + m * v);
// }

// d(q' v q)/dv
// void diffQstarVQ_V(Ref<Mat<3, 4>> d, const Quaternion &q, const Vec<3> v)
// {

// }

// d(q v q')/dv
// void drone_ekf::diffQVQstar_V(Ref<Mat<3, 3>> d, const Quaternion &q, const Vec<3> v)
// {
//     const Vec<4> &coeffs = q.coeffs();
//     Mat<3, 3> m;

//     getSkewSymmetric(m, q.vec());
//     d.noalias() = (q.w() * q.w() - q.vec().squaredNorm()) * Mat<3,3>::Identity() 
//                   + 2.0 * (q.vec() * q.vec().transpose() + q.w() * m);

// }