#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP

#include <Eigen/Dense>

using Quaternion = Eigen::Quaternion<double>;

template <int Rows> 
using Vec = Eigen::Matrix<double, Rows, 1>;

template <int Rows, int Cols> 
using Mat = Eigen::Matrix<double, Rows, Cols>;

struct EulerAngles
{
    double roll, pitch, yaw;
};

EulerAngles ToEulerAngles(Quaternion q);
Vec<4> ToMatrix(Quaternion q);
Mat<3,3> GetDCM(EulerAngles angles);
Mat<3,3> GetDCM(Quaternion q);
Mat<3,4> GetDCMDerivative(Quaternion q, Vec<3> v);
Mat<3,3> GetDCMWithoutYaw(Quaternion q);
Mat<4,4> GetHamiltonDerivativeWrtRHS(Quaternion lhs);

#endif