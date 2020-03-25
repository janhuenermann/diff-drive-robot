#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP

#include <Eigen/Dense>

template <int Rows> 
using Vec = Eigen::Matrix<double, Rows, 1>;

template <int Rows, int Cols> 
using Mat = Eigen::Matrix<double, Rows, Cols>;



struct Quaternion
{
    double w, x, y, z;
};

struct EulerAngles
{
    double roll, pitch, yaw;
};

EulerAngles ToEulerAngles(Quaternion q);
Mat<3,3> GetDCM(EulerAngles angles);
Mat<3,3> GetDCM(Quaternion q);

#endif