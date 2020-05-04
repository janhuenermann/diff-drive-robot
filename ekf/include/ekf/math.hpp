#ifndef ekf_MATH
#define ekf_MATH

#include <ekf/eigen/Addons.h>
#include <ekf/types.hpp>

#include <Eigen/Dense>


namespace ekf
{

    static const Mat<3,3> I3 = Mat<3,3>::Identity();
    static const Mat<4,4> I4 = Mat<4,4>::Identity();

    // d(p*q)/dq
    Mat<4,4> dpq_q(const Quaternion &p);

    // d(p*q)/dp
    Mat<4,4> dpq_p(const Quaternion &q);

    // d(q' v q)/dq
    Mat<3, 4> dqstarvq_q(const Quaternion &q, const Vec<3> &v);

    // d(q v q')/dq
    Mat<3, 4> dqvqstar_q(const Quaternion &q, const Vec<3> &v);

    // d(q' v q)/dv
    Mat<3, 3> dqstarvq_v(const Quaternion &q);

    // d(q v q')/dv
    Mat<3, 3> dqvqstar_v(const Quaternion &q);

    Mat<3, 3> getSkewSymmetric(const Vec<3> &v);

};

#endif
