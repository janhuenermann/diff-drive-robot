#include <drone_ekf/model.hpp>
#include <drone_ekf/math.hpp>


using namespace drone_ekf_model;

using drone_ekf::Vec;
using drone_ekf::Mat;
using drone_ekf::I3;

using Quaternion = Eigen::Quaternion<double>;

void System::reset()
{
    state.mean.setZero();

    // set unit quaternion
    quaternion.mean = Quaternion::Identity().coeffs();
}

void System::predict(const Input &u, const double dt)
{
    const double dtsq = dt*dt;

    // this function needs to calculate
    // - prediction.x
    // - the Jacobi matrix prediction.F
    
    Prediction &pred = prediction;

    Quaternion q(quaternion.mean);
    Quaternion qv, qdot, qa; // tmp variable
    Mat<4, 4> dpq;
    Vec<4> predq;

    qv.w() = 0.0;
    qv.vec() = gyro.mean; // bias already subtracted in measurement

    qdot = q*qv;
    predq = quaternion.mean + 0.5 * dt * qdot.coeffs(); // integration by euler

    pred.reset();

    pred.mean<QuaternionState>()    = predq.normalized();
    pred.mean<GyroState>()          = gyro.mean;
    pred.mean<PositionState>()      = pos.mean + dt * vel.mean + 0.5 * dtsq * accel.mean;
    pred.mean<VelocityState>()      = vel.mean + dt * accel.mean;
    pred.mean<AccelerationState>()  = accel.mean;

    // d(q*qv)/dq
    drone_ekf::diffQuaternionProductWrtP(dpq, qv);
    pred.jacobi<QuaternionState, QuaternionState>()     = 0.5 * dt * dpq;

    // d(q*qv)/dqv
    drone_ekf::diffQuaternionProductWrtQ(dpq, q);
    pred.jacobi<QuaternionState, GyroState>()           = 0.5 * dt * dpq.block<4, 3>(0, 1);

    pred.jacobi<GyroState, GyroState>()                 = I3;

    pred.jacobi<PositionState, PositionState>()         = I3;
    pred.jacobi<PositionState, VelocityState>()         = dt * I3;
    pred.jacobi<PositionState, AccelerationState>()     = 0.5 * dtsq * I3;

    pred.jacobi<VelocityState, VelocityState>()         = I3;
    pred.jacobi<VelocityState, AccelerationState>()     = dt * I3;

    pred.jacobi<AccelerationState, AccelerationState>() = I3;
}

void GyroMeasurement::predict(const GyroState &state)
{
    Transform &pred = prediction;

    pred.mean<0, 3>() = state.mean;
    pred.jacobi<0, 3, GyroState>() = I3;
}

void AccelerometerMeasurement::predict(const ModelState &state)
{
    Transform &pred = prediction;

    // pred.mean<0, 3>() = state.mean;
    // pred.jacobi<0, 3, GyroState>() = I3;
}
