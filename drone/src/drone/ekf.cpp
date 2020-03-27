#include <drone/ekf.hpp>

using namespace std;

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
    // using euler integration
    
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

    const double var_a = 1.00, var_w = 1.00;

    const Mat<3, 3> cov_a = var_a * Mat<3,3>::Identity();
    const Mat<3, 3> cov_w = var_w * Mat<3,3>::Identity();

    Q.block( 6, 6,3,3) = cov_a * dt;                           // variance of acceleration
    Q.block(13,13,3,3) = cov_w * dt;                           // variance of angular vel

    // -- Update Time
    
    last_update_time_ = t;

    return x;
}


void SensorEKF::update()
{
    BaseEKF::update();
}

void SensorEKF::correctByMagnetometer(const geometry_msgs::Vector3Stamped &msg)
{
    last_mag_.x() = msg.vector.x;
    last_mag_.y() = msg.vector.y;
    last_mag_.z() = msg.vector.z;
}

void SensorEKF::correctByIMU(const sensor_msgs::Imu& msg)
{
    Vec<7> z, hx; // observation and predicted observation
    Mat<7, 16> H;
    Mat<7,7> R;

    z.setZero();
    hx.setZero();
    H.setZero();
    R.setZero();

    Vec<3> w = Vec<3>(msg.angular_velocity.x,
                      msg.angular_velocity.y,
                      msg.angular_velocity.z);

    w -= gyro_bias_;

    Vec<3> a = Vec<3>(msg.linear_acceleration.x,
                      msg.linear_acceleration.y,
                      msg.linear_acceleration.z);

    a -= accel_bias_;
    a *= -1;

    Vec<3> m = last_mag_;

    Quaternion q_0 = rotation();
    Mat<3,3> dcm = GetDCM(q_0);

    Vec<3> grav(0, 0, -1.0), mag(1.0, 0.0, 0.0);
    Vec<3> v_g = a / a.norm(), v_m = m / m.norm();
    Vec<3> v_g_hat = dcm * grav, v_m_hat = dcm * mag;
    Vec<3> n_a = v_g.cross(v_g_hat);

    double dtheta_a = acos(v_g.dot(v_g_hat));
    double mu_a = 1.0;
    double cos_a = cos(mu_a * dtheta_a / 2.0),
           sin_a = sin(mu_a * dtheta_a / 2.0);

    Quaternion q_ae(cos_a, n_a.x() * sin_a, n_a.y() * sin_a, n_a.z() * sin_a);
    Quaternion q_est = q_ae * q_0;

    cout << "Est: " << ToMatrix(q_est) << endl;

    z.segment(0, 4) = ToMatrix(q_est);
    z.segment(4, 3) = w;

    hx.segment(0, 4) = ToMatrix(q_0);
    hx.segment(4, 3) = mu_.segment(13, 3);

    H.block(0,9,4,4) = GetHamiltonDerivativeWrtRHS(q_ae);
    // H.block(0, 9,4,4).diagonal().array() = 1;
    H.block(4,13,3,3).diagonal().array() = 1;

    R.diagonal().segment(0,4).array() = 0.01;
    R.diagonal().segment(4,3).array() = 0.03;

    correct<7>(z, R, hx, H);
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
