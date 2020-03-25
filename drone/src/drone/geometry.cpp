#include <drone/geometry.hpp>

EulerAngles ToEulerAngles(Quaternion q) 
{
    EulerAngles angles;
    double sinp = 2 * (q.w * q.y - q.z * q.x);

    if (std::abs(sinp) >= 1)
    {
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    }
    else
    {
        angles.pitch = std::asin(sinp);
    }

    angles.roll = std::atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
    angles.yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));

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
    Mat<3, 3> DCM;

    double qw2 = q.w*q.w, qx2 = q.x*q.x, qy2 = q.y*q.y, qz2 = q.z*q.z;

    DCM <<  qw2 + qx2 - qy2 - qz2, 2*(q.x*q.y + q.w*q.z), 2*(q.x*q.z - q.w*q.y),
            2*(q.x*q.y - q.w*q.z), qw2 - qx2 + qy2 - qz2, 2*(q.y*q.z + q.w*q.x),
            2*(q.x*q.z + q.w*q.y), 2*(q.y*q.z - q.w*q.x), qw2 - qx2 - qy2 + qz2;

    return DCM;
}