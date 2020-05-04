#include <ekf/model.hpp>
#include <ekf/math.hpp>

#include <ros/ros.h>

using namespace std;
using namespace ekf_model;

using ekf::Vec;
using ekf::Mat;
using ekf::Ref;

using ekf::I3;
using ekf::I4;

using Quaternion = Eigen::Quaternion<double>;

using ekf::dpq_p;
using ekf::dpq_q;
using ekf::dqstarvq_q;
using ekf::dqvqstar_q;
using ekf::dqstarvq_v;
using ekf::dqvqstar_v;

/** ---- System ---- */

void System::reset()
{
    state.mean.setZero();
    state.cov = 0.01 * System::State::StateCov::Identity();

    // set unit quaternion
    _quaternion.mean = Quaternion::Identity().coeffs();
}

void System::predict(const Input &u, const double dt)
{
    const double dtsq = dt*dt;
    const Vec<3> a_b = u.mean.segment<3>(0) - _bias_accel.mean;  // acceleration
    const Vec<3> w_b = u.mean.segment<3>(3) - _bias_gyro.mean;   // angular velocity

    const Quaternion qw_b = Quaternion(0.0, w_b(0), w_b(1), w_b(2));  
    const Quaternion qa_b = Quaternion(0.0, a_b(0), a_b(1), a_b(2));

    Vec<3> a_w;

    // Forward step the state
    Prediction &pred = prediction;
    Quaternion qdot, q = getOrientation(_quaternion);

    // Rotate input from body to world frame
    qdot = q * qw_b;
    a_w = (q * qa_b * q.inverse()).vec() - GRAVITY;

    // Derivatives wrt q and a
    Mat<4, 4> dqdot_dqw = dpq_q(q);
    Mat<4, 4> dqdot_dq = dpq_p(qw_b);
    Mat<3, 4> a_w_dq = dqvqstar_q(q, qa_b.vec());
    Mat<3, 3> a_w_da = dqvqstar_v(q);

    // ---

    pred.reset();

    // F(X, U)
    pred.mean<XQuat>() = (_quaternion.mean + 0.5 * dt * qdot.coeffs()).normalized();
    pred.mean<XPos>()  = _pos.mean + dt * _vel.mean + 0.5 * a_w * dtsq;
    pred.mean<XVel>()  = _vel.mean + dt * a_w;
    pred.mean<XBiasAccel>() = _bias_accel.mean;
    pred.mean<XBiasGyro>()  = _bias_gyro.mean;

    // DF(X, U)/DX
    pred.jacobi<XQuat, XQuat>() = I4 + 0.5 * dt * dqdot_dq;
    pred.jacobi<XQuat, XBiasGyro>() = -0.5 * dt * dqdot_dqw.block<4,3>(0,0);
    pred.jacobi<XPos, XPos>() = I3;
    pred.jacobi<XPos, XVel>() = dt * I3;
    pred.jacobi<XPos, XQuat>() = 0.5 * dtsq * a_w_dq; // commented out because doesnt work great
    pred.jacobi<XPos, XBiasAccel>() = -0.5 * dtsq * a_w_da;
    pred.jacobi<XVel, XVel>() = I3;
    pred.jacobi<XVel, XQuat>() = dt * a_w_dq;
    pred.jacobi<XVel, XBiasAccel>() = -dt * a_w_da;
    pred.jacobi<XBiasAccel, XBiasAccel>() = I3;
    pred.jacobi<XBiasGyro, XBiasGyro>() = I3;

    // DF(X, U)/DU
    pred.inputJacobi<XPos, 0, 3>()  = 0.5 * dtsq * a_w_da;
    pred.inputJacobi<XVel, 0, 3>()  = dt * a_w_da;
    pred.inputJacobi<XQuat, 3, 3>() = 0.5 * dt * dqdot_dqw.block<4,3>(0,0);
}

/** ---- Accelerometer ---- */

void AccelerometerGravityMeasurement::update()
{
    XQuat quat = state->getSubState<XQuat>();
    Quaternion g_w(0.0, 0.0, 0.0, 1.0), b_w = getOrientation(quat); // body in world 

    // update gravity in body
    g_b = (b_w.inverse() * g_w * b_w).vec();
    dg_b = dqstarvq_q(b_w, g_w.vec());

    Measurement::update();
}

void AccelerometerGravityMeasurement::transform()
{
    Vec<3> g_f = mean - bias;

    measured.mean<0,3>() = g_f / g_f.norm();
    measured.jacobi<0,3, 0,3>() = I3; // 1.0 / g_f.norm() * I3; // no identity but good enough approximation for gradient
}

void AccelerometerGravityMeasurement::predict()
{
    PredictionTransform &pred = predicted;

    pred.mean<0, 3>() = g_b;
    pred.jacobi<0, 3, XQuat>().noalias() = dg_b;
}

/** ---- Magnetometer ---- */

void MagnetometerMeasurement::predict()
{
    XQuat quat = state->getSubState<XQuat>();
    PredictionTransform &pred = predicted;
    Quaternion b_w = getOrientation(quat); // body in world 

    pred.mean<0, 3>() = (b_w.inverse() * reference_field_ * b_w).vec();
    pred.jacobi<0, 3, XQuat>() = dqstarvq_q(b_w, reference_field_.vec());

    // Mat<3,3> dref = dqstarvq_v(b_w);
    // pred.Q = dref * ref_cov_ * dref.transpose();
}

/** ---- Position ---- */

void PositionMeasurement::predict()
{
    XPos pos = state->getSubState<XPos>();
    PredictionTransform &pred = predicted;

    pred.mean<0, 3>() = pos.mean;
    pred.jacobi<0, 3, XPos>() = I3;

    ROS_INFO_STREAM("measured pos :: \n" << measured.y);
}

/** ---- Velocity ---- */

void VelocityMeasurement::predict()
{
    XVel vel = state->getSubState<XVel>();
    PredictionTransform &pred = predicted;

    pred.mean<0, 3>() = vel.mean;
    pred.jacobi<0, 3, XVel>() = I3;
}

/** ---- Sonar ---- */

void SonarMeasurement::predict()
{
    const Mat<1,3> D = Vec<3>(0.0,0.0,1.0).transpose();
    XPos pos = state->getSubState<XPos>();
    PredictionTransform &pred = predicted;

    pred.mean<0, 1>()(0) = std::max(std::min(3.0, pos.mean.z()), 0.0);
    pred.jacobi<0, 1, XPos>() = D;
}

/** ---- Altimeter ---- */

void AltimeterMeasurement::predict()
{
    const Mat<1,3> D = Vec<3>(0.0,0.0,1.0).transpose();
    XPos pos = state->getSubState<XPos>();
    PredictionTransform &pred = predicted;

    pred.mean<0, 1>()(0) = pos.mean.z(); // std::max(std::min(3.0, pos.mean.z()), 0.0);
    pred.jacobi<0, 1, XPos>() = D;
}

/** ---- Bias ---- */

void BiasMeasurement::predict()
{
    XBias b = state->getSubState<XBias>();
    PredictionTransform &pred = predicted;

    pred.mean<0, 6>() = b.mean;
    pred.jacobi<0, 6, XBias>().setIdentity();
}

/** ---- Orientation ---- */

void OrientationMeasurement::predict()
{
    XQuat quat = state->getSubState<XQuat>();
    PredictionTransform &pred = predicted;

    pred.mean<0, 4>() = quat.mean;
    pred.jacobi<0, 4, XQuat>().setIdentity();
}
