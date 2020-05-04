#include <ekf/math.hpp>

using namespace ekf;

const Mat<4,4> I4 = Mat<4,4>::Identity();

Mat<3,3> ekf::getSkewSymmetric(const Vec<3> &v)
{
    Mat<3,3> out;
    out <<   0.0, -v(2),   v(1),
            v(2),   0.0,  -v(0),
           -v(1),  v(0),    0.0;

    return out;
}


/**
 * Quaternion functions
 * Reference for these derivations: http://graphics.stanford.edu/courses/cs348a-17-winter/Papers/quaternion.pdf
 */

// d(p*q)/dq
Mat<4,4> ekf::dpq_q(const Quaternion &p)
{
    Mat<4,4> out;
    out.block<3, 3>(0, 0).noalias() = I3 * p.w() + getSkewSymmetric(p.vec());
    out.block<1, 3>(3, 0).noalias() = -p.vec().transpose();
    out.block<3, 1>(0, 3).noalias() = p.vec();
    out(3, 3) = p.w();

    return out;
}


// d(p*q)/dq
Mat<4,4> ekf::dpq_p(const Quaternion &q)
{
    Mat<4,4> out;
    out.block<3, 3>(0, 0).noalias() = I3 * q.w() - getSkewSymmetric(q.vec());
    out.block<1, 3>(3, 0).noalias() = -q.vec().transpose();
    out.block<3, 1>(0, 3).noalias() = q.vec();
    out(3, 3) = q.w();

    return out;
}

// d(q' v q)/dq
Mat<3,4> ekf::dqstarvq_q(const Quaternion &q, const Vec<3> &v)
{
    Mat<3,4> out;
    double q0 = q.w();
    Vec<3> qv = q.vec();

    // wrt. w
    out.col(3).noalias()
        = 2*(q0*v - getSkewSymmetric(qv)*v);

    // wrt. xyz
    out.block<3, 3>(0, 0).noalias()
        = 2*(-v*qv.transpose() + v.dot(qv)*Mat<3,3>::Identity() + q0*getSkewSymmetric(v));

    return out;
}

// d(q v q')/dq
Mat<3,4> ekf::dqvqstar_q(const Quaternion &q, const Vec<3> &v)
{

    Mat<3,4> out;
    double q0 = q.w();
    Vec<3> qv = q.vec();

    // wrt. w
    out.col(3).noalias()
        = 2*(q0*v + getSkewSymmetric(qv)*v);

    // wrt. xyz
    out.block<3, 3>(0, 0).noalias()
        = 2*(-v*qv.transpose() + v.dot(qv)*Mat<3,3>::Identity() - q0*getSkewSymmetric(v));

    return out;
}

// d(q' v q)/dv
Mat<3,3> ekf::dqstarvq_v(const Quaternion &q)
{
    Mat<3,3> out;
    double q0 = q.w();
    Vec<3> qv = q.vec();
    out.noalias() = (q0*q0 - qv.dot(qv))*Mat<3,3>::Identity() + 2*qv*qv.transpose() - 2*q0*getSkewSymmetric(qv);
    return out;
}

// d(q v q')/dv
Mat<3,3> ekf::dqvqstar_v(const Quaternion &q)
{
    Mat<3,3> out;
    double q0 = q.w();
    Vec<3> qv = q.vec();
    out.noalias() = (q0*q0 - qv.dot(qv))*Mat<3,3>::Identity() + 2*qv*qv.transpose() + 2*q0*getSkewSymmetric(qv);
    return out;
}