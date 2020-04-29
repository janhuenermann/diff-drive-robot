#ifndef EKF_MODEL_HPP
#define EKF_MODEL_HPP


/**
 * File contains implementations of state and measurement class 
 * for the specific drone model.
 */

#include <drone_ekf/eigen/Addons.h>
#include <drone_ekf/types.hpp>
#include <drone_ekf/ekf.hpp>
#include <drone_ekf/measurement.hpp>
#include <drone_ekf/system.hpp>

#include <Eigen/Dense>

using drone_ekf::Measurement;
using drone_ekf::StateMeasurement;

using drone_ekf::Vec;
using drone_ekf::Mat;
using drone_ekf::Quaternion;

using drone_ekf::FullState;
using drone_ekf::SubState;

namespace drone_ekf_model
{

    static const unsigned int StateSize = 17;
    static const unsigned int InputSize = 0;

    using ModelState = FullState<StateSize>;

    // sub-states
    using PositionState = SubState<ModelState, 0, 3>;
    using VelocityState = SubState<ModelState, 3, 3>;
    using AccelState = SubState<ModelState, 6, 3>;
    using QuatState = SubState<ModelState, 9, 4>;
    using GyroState = SubState<ModelState, 13, 3>;

    inline Quaternion getOrientation(QuatState _q) { return Quaternion(_q.mean); }

    // measurements

    struct GyroMeasurement : public StateMeasurement<3, GyroState>
    {

        // System measurement
        // => angular velocity directly measured

        using StateMeasurement::StateMeasurement;
        using StateMeasurement::Transform;

        using Measurement::state;
        using Measurement::predicted;

        void predict();

    };

    struct AccelerometerMeasurement : public Measurement<3, ModelState, 3>
    {

        // we correct two independent states
        // - 1. quaternion := z[0:3]
        //      z = normed(accelerometer - state acceleration)
        //      zhat = rotated gravity vector
        //      
        // - 2. acceleration := z[3:6]
        //      z = accelerometer - rotated gravity vector
        //      zhat = acceleration
        
        static constexpr const double GRAVITY_MAGNITUDE = 9.81;
        
        using AccelVec = Vec<3>;

        using GaussianVector<3>::mean;
        using GaussianVector<3>::cov;

        using Measurement<3, ModelState, 3>::PredictionTransform;

        using Measurement<3, ModelState, 3>::predicted;
        using Measurement<3, ModelState, 3>::measured;

        AccelerometerMeasurement() : 
            Measurement<3,ModelState,3>(), 
            bias(AccelVec::Zero()),
            _accel(nullptr),
            _quaternion(nullptr)
        {}

        void attach(ModelState::SRef *st);
        void update();
        void transform(); // z
        void predict(); // zhat

        void applied() { _quaternion->mean.normalize(); }

        AccelVec bias;

    private:

        AccelState *_accel;
        QuatState *_quaternion;

        Vec<3> g_b; // gravity in body frame
        Mat<3,4> dg_b;  // derivative

    };

    struct MagnetometerMeasurement : public StateMeasurement<3, QuatState>
    {

        // predict magnetometer readings from state
        using StateMeasurement::PredictionTransform;

        using StateMeasurement::StateMeasurement;
        using StateMeasurement::Transform;

        using Measurement::state;
        using Measurement::predicted;

        void predict(); // zhat

        void setReference(Vec<3> ref)
        {
            reference_field_ = Quaternion(0.0, ref.x(), ref.y(), ref.z());
        }

        void applied() { state->mean.normalize(); }

        Quaternion reference_field_;

    };

    // struct GPSMeasurement : Measurement<3, PositionSubState>
    // {
    //     using Measurement::Measurement;
    // };

    // system

    struct System : public drone_ekf::System<StateSize, InputSize>
    {

        // Types
        using drone_ekf::System<StateSize, InputSize>::Input;
        using Prediction = drone_ekf::System<StateSize, InputSize>::Transform;

        // Properties
        using drone_ekf::System<StateSize, InputSize>::state;
        using drone_ekf::System<StateSize, InputSize>::prediction;

        // Members
        
        System(State &st) : drone_ekf::System<StateSize, InputSize>(st),
            _quaternion(st.getSubState<QuatState>()),
            _gyro(st.getSubState<GyroState>()),
            _pos(st.getSubState<PositionState>()),
            _vel(st.getSubState<VelocityState>()),
            _accel(st.getSubState<AccelState>())
        {}

        void reset();
        void predict(const Input &u, const double dt);

        inline drone_ekf::Quaternion getQuaternion() { return drone_ekf::Quaternion(_quaternion.mean); }

    private:

        QuatState _quaternion;
        GyroState _gyro;

        PositionState _pos;
        VelocityState _vel;
        AccelState _accel;
        
    };


    // filter

    class EKF : public drone_ekf::EKF<System>
    {

    public:

        EKF() : 
            drone_ekf::EKF<System>()
        {
            ROS_INFO("BOOTING EKF");
        }

        using drone_ekf::EKF<System>::system_;

        inline drone_ekf::Quaternion orientation() { return system_.getQuaternion(); };

    };

};

#endif