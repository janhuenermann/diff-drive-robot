#include <drone/geometry.hpp>

EulerAngles ToEulerAngles(Quaternion q) 
{
    EulerAngles angles;
    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());

    if (std::abs(sinp) >= 1)
    {
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    }
    else
    {
        angles.pitch = std::asin(sinp);
    }

    angles.roll = std::atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + q.y() * q.y()));
    angles.yaw = std::atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));

    return angles;
}

// roll (X = phi), pitch(Y = theta), yaw(Z = psi)
Mat<3,3> GetDCM(EulerAngles angles)
{
    Mat<3, 3> DCM;

    double cosr = cos(angles.roll), sinr = sin(angles.roll);
    double cosp = cos(angles.pitch), sinp = sin(angles.pitch);
    double cosy = cos(angles.yaw), siny = sin(angles.yaw);

    DCM << cosp*cosy, -cosr*siny + sinr*sinp*cosy,  sinr*siny + cosr*sinp*cosy,
           cosp*siny,  cosr*cosy + sinr*sinp*siny, -sinr*cosy + cosr*sinp*siny,
               -sinp,                   sinr*cosp,                   cosr*cosp;

    return DCM;
}

Mat<3,3> GetDCM(Quaternion q)
{
    return q.toRotationMatrix();
    // Mat<3, 3> DCM;

    // double qw2 = q.w()*q.w(), qx2 = q.x()*q.x(), qy2 = q.y()*q.y(), qz2 = q.z()*q.z();

    // DCM <<          qw2 + qx2 - qy2 - qz2, 2*(q.x()*q.y() + q.w()*q.z()), 2*(q.x()*q.z() - q.w()*q.y()),
    //         2*(q.x()*q.y() - q.w()*q.z()),         qw2 - qx2 + qy2 - qz2, 2*(q.y()*q.z() + q.w()*q.x()),
    //         2*(q.x()*q.z() + q.w()*q.y()), 2*(q.y()*q.z() - q.w()*q.x()),         qw2 - qx2 - qy2 + qz2;

    // return DCM;
}

Mat<3,4> GetDCMDerivative(Quaternion q, Vec<3> v)
{
    Mat<3, 4> dDCM;

    dDCM << v.x()*q.w() - v.y()*q.z() + v.z()*q.y(), v.x()*q.x() + v.y()*q.y() + v.z()*q.z(), v.y()*q.x() - v.x()*q.y() + v.z()*q.w(), v.z()*q.x() - v.y()*q.w() - v.x()*q.z(),
            v.x()*q.z() + v.y()*q.w() - v.z()*q.x(), v.x()*q.y() - v.y()*q.x() - v.z()*q.w(), v.x()*q.x() + v.y()*q.y() + v.z()*q.z(), v.x()*q.w() - v.y()*q.z() + v.z()*q.y(),
            v.y()*q.x() - v.x()*q.y() + v.z()*q.w(), v.x()*q.z() + v.y()*q.w() - v.z()*q.x(), v.y()*q.z() - v.x()*q.w() - v.z()*q.y(), v.x()*q.x() + v.y()*q.y() + v.z()*q.z();


    return dDCM;
}

Mat<3,3> GetDCMWithoutYaw(Quaternion q)
{
    Mat<3, 3> DCM;

    double c1 = 2.0*(q.w()*q.y() - q.z()*q.x()); // sin(theta)
    double c2 = sqrt(1.0 - c1*c1);       // cos(theta)
    double c3 = 2.0*(q.w()*q.x() + q.y()*q.z());
    double c4 = 1.0 - 2.0*(q.x()*q.x() + q.y()*q.y());
    double c5 = sqrt(c3*c3 + c4*c4);
    double c6 = c3/c5;  // sin(psi)
    double c7 = c4/c5;  // cos(psi)

    DCM <<  c2, c6*c1, c7*c1,
           0.0,    c7,   -c6,
           -c1, c6*c2, c7*c2;

    return DCM;
}

Mat<4,4> GetHamiltonDerivativeWrtRHS(Quaternion lhs)
{
    Mat<4,4> d;
 
    d << lhs.w(), -lhs.x(), -lhs.y(), -lhs.z(),
         lhs.x(),  lhs.w(), -lhs.z(),  lhs.y(),
         lhs.y(),  lhs.z(),  lhs.w(), -lhs.x(),
         lhs.z(), -lhs.y(),  lhs.x(),  lhs.w();

    return d;
}

Vec<4> ToMatrix(Quaternion q)
{
    return Vec<4>(q.w(), q.x(), q.y(), q.z());
}