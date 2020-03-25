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


    // -- Calculate Next Time-Step

    // Position
    pos(x) = pos(_x) + dt * vel(_x) + dtsq/2.0 * acc(_x);
    vel(x) = vel(_x) + dt * acc(_x);
    acc(x) = acc(_x);

    // Quaternion
    Mat<4, 4> omega;
    omega <<  0,  -w1, -w2, -w3,
             w1,    0,  w3, -w2,
             w2,  -w3,   0,  w1,
             w3,   w2, -w1,   0;

    Mat<4, 3> G; // needed for dq/dw
    G << -q1, -q2, -q3,
          q0, -q3,  q2,
          q3,  q0, -q1,
         -q2,  q1,  q0;

    // q = exp(omega * dT) * q  =approx.=  (I + dt/2 * omega) * q
    Mat<4, 4> dqdq = Mat<4, 4>::Identity() + 0.5 * omega * dt;

    Vec<4> q_ = dqdq * q(_x);
    q(x) = q_ / q_.norm(); // normalise
    w(x) = w(_x);


    // -- Calculate Jacobi

    F.setZero();

    slice_diag(F, 0,0,3,3) = 1;        // dx/dx = 1
    slice_diag(F, 0,3,3,3) = dt;       // dx/dv = dt
    slice_diag(F, 0,6,3,3) = dtsq/2;   // dx/da = dt^2/2
    slice_diag(F, 3,3,3,3) = 1;        // dv/dv = 1
    slice_diag(F, 3,6,3,3) = dt;       // dv/da = dt
    slice_diag(F, 6,6,3,3) = 1;        // da/da = 1

    F.block(9,9,4,4)    = dqdq;        // dq/dq
    F.block(9,13,4,3)   = 0.5*dt*G;    // dq/dw

    slice_diag(F, 13,13,3,3) = 1.0;    // dw/dw


    // -- Calculate Process Covariance

    Q.setZero();

    const double var_a = 0.66, var_w = 0.33;

    const Mat<3, 3> cov_a = var_a * Mat<3,3>::Identity();
    const Mat<3, 3> cov_w = var_w * Mat<3,3>::Identity();

    // TODO: variance on velocity by accel noise
    // TODO: variance on position by accel noise + vel noise
    Q.block( 6, 6,3,3) = cov_a * dt;                           // variance of acceleration
    Q.block( 9, 9,4,4) = dtsq/4.0 * G * cov_w * G.transpose(); // variance on quaternion introduced by noise of angular vel
    Q.block(13,13,3,3) = cov_w * dt;                           // variance of angular vel


    // -- Update Time
    
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
    EulerAngles euler = ToEulerAngles(rotation());
    mag = GetDCM((EulerAngles){ euler.roll, euler.pitch, 0.0 }) * mag;

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
