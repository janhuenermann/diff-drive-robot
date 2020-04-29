#ifndef DRONE_EKF_MATH
#define DRONE_EKF_MATH

#include <drone_ekf/eigen/Addons.h>
#include <drone_ekf/types.hpp>

#include <Eigen/Dense>


namespace drone_ekf
{

    static const Mat<3,3> I3 = Mat<3,3>::Identity();
    static const Mat<4,4> I4 = Mat<4,4>::Identity();

    // d(p*q)/dq
    void diffQuaternionProductWrtQ(Ref<Mat<4,4>> dpq, const Quaternion &p);

    // d(p*q)/dq
    void diffQuaternionProductWrtP(Ref<Mat<4,4>> dpq, const Quaternion &q);

    // d(q' v q)/dq
    void diffQstarVQ_Q(Ref<Mat<3, 4>> d, const Quaternion &q, const Vec<3> &v);
    // d(q v q')/dq
    void diffQVQstar_Q(Ref<Mat<3, 4>> d, const Quaternion &q, const Vec<3> &v);
    // d(q' v q)/dv
    // void diffQstarVQ_V(Ref<Mat<4, 4>> d, const Quaternion &q, const Vec<3> v);
    // d(q v q')/dv
    void diffQVQstar_V(Ref<Mat<3, 3>> d, const Quaternion &q, const Vec<3> &v);

    void getSkewSymmetric(Ref<Mat<3, 3>> mat, const Vec<3> &v);
    Mat<3, 3> getSkewSymmetric(const Vec<3> &v);

};

#endif
