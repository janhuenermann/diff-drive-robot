#ifndef EKF_MODEL_HPP
#define EKF_MODEL_HPP


/**
 * File contains implementations of state and measurement class 
 * for the specific drone model.
 */

#include <drone_ekf/measurement.hpp>
#include <drone_ekf/system.hpp>
#include <drone_ekf/ekf.hpp>

#include <Eigen/Dense>

using drone_ekf::Measurement;
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
    using AccelerationState = SubState<ModelState, 6, 3>;
    using QuaternionState = SubState<ModelState, 9, 4>;
    using OrientationState = SubState<ModelState, 9, 7>;
    using GyroState = SubState<ModelState, 13, 3>;

    struct GyroMeasurement : Measurement<3, GyroState>
    {
        using Measurement::Measurement;
        using Measurement::Transform;

        using Measurement::prediction;

        void predict(const GyroState &state);
    };

    struct AccelerometerMeasurement : Measurement<3, ModelState>
    {
        using Measurement::Measurement;
        using Measurement::Transform;

        using Measurement::prediction;

        void predict(const ModelState &state);
    };

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
            orientation(st.getSubState<OrientationState>()),
            quaternion(st.getSubState<QuaternionState>()),
            gyro(st.getSubState<GyroState>()),
            pos(st.getSubState<PositionState>()),
            vel(st.getSubState<VelocityState>()),
            accel(st.getSubState<AccelerationState>())
        {}

        void reset();
        void predict(const Input &u, const double dt);

        OrientationState orientation;
        QuaternionState quaternion;
        GyroState gyro;

        PositionState pos;
        VelocityState vel;
        AccelerationState accel;
        
    };

    // struct MagnetometerMeasurement : Measurement<3, OrientationSubState>
    // {
    //     using Measurement::Measurement;
    // };

    // struct GPSMeasurement : Measurement<3, PositionSubState>
    // {
    //     using Measurement::Measurement;
    // };

    class EKF : public drone_ekf::EKF<System>
    {};

};

#endif