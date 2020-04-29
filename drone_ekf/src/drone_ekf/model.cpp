#include <drone_ekf/model.hpp>
#include <drone_ekf/math.hpp>

#include <ros/ros.h>

using namespace std;
using namespace drone_ekf_model;

using drone_ekf::Vec;
using drone_ekf::Mat;
using drone_ekf::Ref;

using drone_ekf::I3;
using drone_ekf::I4;

using Quaternion = Eigen::Quaternion<double>;

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

    // this function needs to calculate
    // - prediction.x
    // - the Jacobi matrix prediction.F
    
    Prediction &pred = prediction;

    Quaternion q(_quaternion.mean);
    Quaternion qv, qdot, qa; // tmp variable
    Mat<4, 4> dpq;
    Vec<4> predq;

    qv.w() = 0.0;
    qv.vec() = _gyro.mean; // bias already subtracted in measurement

    qdot = q*qv;
    predq = _quaternion.mean + 0.5 * dt * qdot.coeffs(); // integration by euler

    pred.reset();

    pred.mean<QuatState>()      = predq.normalized();
    pred.mean<GyroState>()      = _gyro.mean;
    pred.mean<PositionState>()  = _pos.mean + dt * _vel.mean + 0.5 * dtsq * _accel.mean;
    pred.mean<VelocityState>()  = _vel.mean + dt * _accel.mean;
    pred.mean<AccelState>()       = _accel.mean;

    // dependencies:
    // 
    // - dq/dq, dq/dw
    // - dw/dw
    // - dx/dx, dx/dv, dx/da
    // - dv/dv, dv/da
    // - da/da

    // dq/dq
    drone_ekf::diffQuaternionProductWrtP(dpq, qv);
    pred.jacobi<QuatState, QuatState>() = I4 + 0.5 * dt * dpq;

    // dq/dw
    drone_ekf::diffQuaternionProductWrtQ(dpq, q);
    pred.jacobi<QuatState, GyroState>() = 0.5 * dt * dpq.block<4, 3>(0, 0);

    // dw/dw
    pred.jacobi<GyroState, GyroState>() = I3;

    // dx/dx
    pred.jacobi<PositionState, PositionState>() = I3;
    // dx/dv
    pred.jacobi<PositionState, VelocityState>() = dt * I3;
    // dx/da
    pred.jacobi<PositionState, AccelState>() = 0.5 * dtsq * I3;

    // dv/dv
    pred.jacobi<VelocityState, VelocityState>() = I3;
    // dv/da
    pred.jacobi<VelocityState, AccelState>() = dt * I3;

    // da/da
    pred.jacobi<AccelState, AccelState>() = I3;

    pred.noise<GyroState>() = 0.33 * dt * I3;
}

void GyroMeasurement::predict()
{
    Transform &zhat = predicted;

    zhat.mean<0,3>() = state->mean;
    zhat.jacobi<0,3, GyroState>() = I3;
}

void AccelerometerMeasurement::attach(ModelState::SRef *st)
{
    Measurement::attach(st);

    // create sub-states used later
    _accel = st->getSubStateRef<AccelState>();
    _quaternion = st->getSubStateRef<QuatState>();
}

void AccelerometerMeasurement::update()
{
    Quaternion g_w(0.0, 0.0, 0.0, 1.0), b_w = getOrientation(*_quaternion); // body in world 

    // update gravity in body
    g_b = (b_w.inverse() * g_w * b_w).vec();
    drone_ekf::diffQstarVQ_Q(dg_b, b_w, g_w.vec());

    Measurement::update();
}

void AccelerometerMeasurement::transform()
{
    Vec<3> g_f = mean - bias;
    double g_f_magn = g_f.norm();

    measured.reset();

    valid = abs(g_f_magn - GRAVITY_MAGNITUDE) < 3.0;

    measured.mean<0,3>() = g_f / g_f_magn;
    measured.jacobi<0,3, 0,3>() = I3; // 1.0 / g_f_magn * I3; // no identity but good enough approximation for gradient
}

void AccelerometerMeasurement::predict()
{
    PredictionTransform &pred = predicted;

    pred.reset();

    pred.mean<0, 3>() = g_b;
    pred.jacobi<0, 3, QuatState>().noalias() = dg_b;
}

void MagnetometerMeasurement::predict()
{
    PredictionTransform &pred = predicted;

    Quaternion b_w = getOrientation(*state); // body in world 

    // update gravity in body
    pred.mean<0, 3>() = (b_w.inverse() * reference_field_ * b_w).vec();

    drone_ekf::diffQstarVQ_Q(pred.jacobi<0,3, QuatState>(),
                             b_w, reference_field_.vec());

    auto v = pred.mean<0,3>();
    ROS_WARN_STREAM( "magneto pred : \n " << v );
    ROS_WARN_STREAM( "magneto act  : \n " << mean );
}

