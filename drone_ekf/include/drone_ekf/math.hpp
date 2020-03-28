#ifndef DRONE_EKF_MATH
#define DRONE_EKF_MATH

#include <drone_ekf/types.hpp>

#include <Eigen/Dense>


namespace drone_ekf
{

    extern const Mat<3,3> I3;

    // d(p*q)/dq
    void diffQuaternionProductWrtQ(Ref<Mat<4,4>> dpq, const Quaternion &p);

    // d(p*q)/dq
    void diffQuaternionProductWrtP(Ref<Mat<4,4>> dpq, const Quaternion &q);

    void getSkewSymmetricMatrix(Ref<Mat<3, 3>> mat, const Vec<3> &v);

};

#endif
