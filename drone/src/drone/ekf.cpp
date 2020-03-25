#include <drone/ekf.hpp>

using namespace std;
using namespace Eigen;

template<unsigned int Dims>
void BaseEKF<Dims>::update()
{
    Mat<Dims, Dims> F;
    Mat<Dims, Dims> Q;

    mu_ = processStep(F, Q);
    cov_ = F * cov_ * F.transpose() + Q;
}

template<unsigned int Dims>
template<unsigned int ZDims>
void BaseEKF<Dims>::correct(Vec<ZDims> z, Mat<ZDims, ZDims> R, Vec<ZDims> h_x, Mat<ZDims, Dims> H)
{
    Vec<ZDims> y = z - h_x; // y = z - h(x)
    Mat<Dims, ZDims> K = cov_ * H.transpose() * (H * cov_ * H.transpose() + R).inverse(); // K = P * H' * (H * P * H' + R)^(-1)
    mu_ += K * y;
    cov_ -= K * H * cov_;
}

void ToEulerAngles(double q0, double q1, double q2, double q3, double& roll, double& pitch, double& yaw) 
{
    double sinp = 2 * (q0 * q2 - q3 * q1);

    // roll (x-axis rotation)
    roll = std::atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));

    // pitch (y-axis rotation)
    if (std::abs(sinp) >= 1)
    {
        pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    }
    else
    {
        pitch = std::asin(sinp);
    }

    // yaw (z-axis rotation)
    yaw = std::atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));
}

// roll (X = phi), pitch(Y = theta), yaw(Z = psi)
Mat<3,3> GetDCM(double roll, double pitch, double yaw)
{
    Mat<3, 3> DCM;

    double cosr = cos(roll), sinr = sin(roll);
    double cosp = cos(pitch), sinp = sin(pitch);
    double cosy = cos(yaw), siny = sin(yaw);

    DCM << cosp*cosy, -cosr*siny + sinr*sinp*cosy,  sinr*siny + cosr*sinp*cosy,
           cosp*siny,  cosr*cosy + sinr*sinp*siny, -sinr*cosy + cosr*sinp*siny,
               -sinp,                   sinr*cosp,                   cosr*cosp;

    return DCM;
}

Mat<3,3> GetDCM(double q0, double q1, double q2, double q3)
{
    Mat<3, 3> DCM;
    DCM <<  q0*q0 + q1*q1 - q2*q2 - q3*q3,               2*(q1*q2 + q0*q3),              2*(q1*q3 - q0*q2),
                          2*(q1*q2-q0*q3),   q0*q0 - q1*q1 + q2*q2 - q3*q3,              2*(q2*q3 + q0*q1),
                          2*(q1*q3+q0*q2),               2*(q2*q3 - q0*q1),  q0*q0 - q1*q1 - q2*q2 + q3*q3;

    return DCM;
}

// [x_1, x_2, x_3, v_1, v_2, v_3, a_1, a_2, a_3, q_0, q_1, q_2, q_3, w_1, w_2, w_3]

SensorEKF::SensorEKF()
{
    // reset
    mu_.array() = 0;
    cov_.array() = 0;
    q0() = 1;
    started_ = false;
}

#define sub(v, i0, n) v.segment(i0, n)
#define pos(v)  sub(v, 0, 3)
#define vel(v)  sub(v, 3, 3)
#define acc(v)  sub(v, 6, 3)
#define q(v)    sub(v, 9, 4)
#define qi(i, v) v(9+i)
#define w(v)    sub(v, 13, 3)
#define wi(i, v) v(13 + i)

#define slice_row(A, row, i0, n) A.block(row, i0, 1, n).array()
#define slice_col(A, col, i0, n) A.block(i0, col, n, 1).array()
#define slice_diag(A, i,j,n,m) A.block(i,j,n,m).diagonal().array()


Vec<_DIMS> SensorEKF::processStep(Mat<_DIMS, _DIMS> &F, Mat<_DIMS, _DIMS> &Q)
{
    ros::Time t = ros::Time::now();

    if (!started_)
    {
        started_ = true;
        last_update_time_ = t;
    }

    ros::Duration dur = t - last_update_time_;

    Vec<_DIMS> _x = state();
    Vec<_DIMS> x;

    const double dt = clamp(dur.toSec(), 1e-2, 1.0);
    const double dtsq = dt*dt;
    const double w1 = wi(0, _x), w2 = wi(1, _x), w3 = wi(2, _x);
    const double q0 = qi(0, _x), q1 = qi(1, _x), q2 = qi(2, _x), q3 = qi(3, _x);

    // position
    pos(x) = pos(_x) + dt * vel(_x) + dtsq/2.0 * acc(_x);
    vel(x) = vel(_x) + dt * acc(_x);
    acc(x) = acc(_x);

    // quaternion

    Mat<4, 4> omega;
    omega <<  0,  -w1, -w2, -w3,
             w1,    0,  w3, -w2,
             w2,  -w3,   0,  w1,
             w3,   w2, -w1,   0;

    // q = exp(omega * dT) * q {approx.} = (I + dT/2 * omega) * q

    Mat<4, 4> dqdq = Mat<4, 4>::Identity() + 0.5 * omega * dt;

    Vec<4> q_ = dqdq * q(_x);
    q(x) = q_ / q_.norm(); // normalise
    w(x) = w(_x);

    // calculate jacobi
    F.setZero();

    // dx
    slice_diag(F, 0,0,3,3) = 1;        // dx/dx = 1
    slice_diag(F, 0,3,3,3) = dt;       // dx/dv = dt
    slice_diag(F, 0,6,3,3) = dtsq/2;   // dx/da = dt^2/2

    // dv
    slice_diag(F, 3,3,3,3) = 1;        // dv/dv = 1
    slice_diag(F, 3,6,3,3) = dt;       // dv/da = dt

    // da
    slice_diag(F, 6,6,3,3) = 1;        // da/da = 1

    // dq/dq
    F.block(9,9,4,4) = dqdq;

    // dq/dw
    Mat<4, 3> dqdw;
    dqdw << -0.5*dt*q1, -0.5*dt*q2, -0.5*dt*q3,
             0.5*dt*q0, -0.5*dt*q3,  0.5*dt*q2,
             0.5*dt*q3,  0.5*dt*q0, -0.5*dt*q1,
            -0.5*dt*q2,  0.5*dt*q1,  0.5*dt*q0;

    F.block(9,13,4,3) = dqdw;

    // dw/dw
    slice_diag(F, 13,13,3,3) = 1.0;


    // Process Covariance
    Q.setZero();

    Mat<4, 3> G;
    G <<  q1,  q2,  q3,
         -q0,  q3, -q2,
         -q3, -q0,  q1,
          q2, -q1, -q0;

    const double var_a = 0.66, var_w = 0.33;

    const Mat<3, 3> cov_a = var_a * Mat<3,3>::Identity();
    const Mat<3, 3> cov_w = var_w * Mat<3,3>::Identity();

    Q.block(6,6,3,3)    = cov_a * dt;
    Q.block(9,9,4,4)    = dtsq/4.0 * G * cov_w * G.transpose();
    Q.block(13,13,3,3)  = cov_w * dt;

    last_update_time_ = t;

    return x;
}


void SensorEKF::update()
{
    BaseEKF::update();
}

Vector3d mmag;

void SensorEKF::correctByMagnetometer(const geometry_msgs::Vector3Stamped &msg)
{
    Vec<1> z, hx;
    Mat<1,1> R;
    Mat<1, 16> H;

    Vector3d mag(msg.vector.x, msg.vector.y, msg.vector.z);

    // Project magnetic field into world frame, without rotating around yaw.
    double pitch, roll, yaw;
    ToEulerAngles(q0(), q1(), q2(), q3(), roll, pitch, yaw);
    mag = GetDCM(roll, pitch, 0.0) * mag;

    z << -atan2(mag.y(), mag.x());
    R << 1.00;

    const double q0 = this->q0(), q1 = this->q1(), q2 = this->q2(), q3 = this->q3();
    const double c1 = 2.0*(q0*q3 + q1*q2), c1sq = c1*c1,
                 c2 = q0*q0 + q1*q1 - q2*q2 - q3*q3,
                 c3 = c1*c1 + c2*c2;

    hx << atan2(c1, c2);

    H(0, 9) = 2.0*(q3*c2 - q0*c1)/c3;
    H(0,10) = 2.0*(q2*c2 - q1*c1)/c3;
    H(0,11) = 2.0*(q1*c2 + q2*c1)/c3;
    H(0,12) = 2.0*(q0*c2 + q3*c1)/c3;

    correct<1>(z, R, hx, H);
}

Mat<3, 3> SensorEKF::getDCM(bool invert)
{
    double q0_ = q0();

    if (invert)
        q0_ *= -1;

    Mat<3, 3> DCM;
    DCM <<  q0_*q0_ + q1()*q1() - q2()*q2() - q3()*q3(),                      2*(q1()*q2() + q0_*q3()),                     2*(q1()*q3() - q0_*q2()),
                                 2*(q1()*q2()-q0_*q3()),   q0_*q0_ - q1()*q1() + q2()*q2() - q3()*q3(),                     2*(q2()*q3() + q0_*q1()),
                                 2*(q1()*q3()+q0_*q2()),                      2*(q2()*q3() - q0_*q1()),  q0_*q0_ - q1()*q1() - q2()*q2() + q3()*q3();

    return DCM;
}


void SensorEKF::correctByIMU(const sensor_msgs::Imu& msg)
{
    Vec<6> z, hx;
    Mat<6,  6> R;
    Mat<6, 16> H;

    R.setZero();
    H.setZero();

    // Accel

    Vec<3> lin_acc = Vector3d(msg.linear_acceleration.x,
                              msg.linear_acceleration.y,
                              msg.linear_acceleration.z);

    lin_acc -= accel_bias_;

    // Gyro

    Vec<3> ang_vel = Vector3d(msg.angular_velocity.x,
                              msg.angular_velocity.y,
                              msg.angular_velocity.z);

    ang_vel -= gyro_bias_;

    // set correction params

    z.segment(0,3) = lin_acc; // accel z
    z.segment(3,3) = ang_vel;  // gyro z

    R.diagonal().segment(0,3).array() = 2.00; // accel cov
    R.diagonal().segment(3,3).array() = 0.33; // gyro cov

    hx.segment(3,3) = mu_.segment(13, 3); // accel z'
    hx.segment(0,3) = mu_.segment( 6, 3); // gyro z'

    H.block(0, 6,3,3).diagonal().array() = 1.0; // jacobi accel
    H.block(3,13,3,3).diagonal().array() = 1.0; // jacobi gyro

    // do correction

    correct<6>(z, R, hx, H);
}

void SensorEKF::correctByQuaternion(const geometry_msgs::Quaternion& msg)
{
    Vec<4> z, hx;
    Mat<4,  4> R;
    Mat<4, 16> H;
}

void SensorEKF::correctByGPS(const sensor_msgs::NavSatFix& msg)
{
}

template class BaseEKF<16>;
