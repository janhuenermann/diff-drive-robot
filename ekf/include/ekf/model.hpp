#ifndef EKF_MODEL_HPP
#define EKF_MODEL_HPP


/**
 * File contains implementations of state and measurement class 
 * for the specific drone model.
 */

#include <ekf/eigen/Addons.h>
#include <ekf/types.hpp>
#include <ekf/ekf.hpp>
#include <ekf/measurement.hpp>
#include <ekf/system.hpp>

#include <Eigen/Dense>

using ekf::Measurement;
using ekf::StateMeasurement;

using ekf::Vec;
using ekf::Mat;
using ekf::Quaternion;

using ekf::FullState;
using ekf::SubState;

namespace ekf_model
{

    const Vec<3> GRAVITY = Vec<3>(0.0, 0.0, 9.81);

    static const unsigned int StateSize = 16; 
    static const unsigned int InputSize = 6; // Acceleration and Gyro

    using X = FullState<StateSize>;

    // sub-states
    using XPos = SubState<X, 0, 3>;
    using XVel = SubState<X, 3, 3>;
    using XQuat = SubState<X, 6, 4>;

    using XBias = SubState<X, 10, 6>;
    using XBiasAccel = SubState<X, 10, 3>;
    using XBiasGyro = SubState<X, 13, 3>;

    inline Quaternion getOrientation(XQuat _q) { return Quaternion(_q.mean); }

    // measurements

    struct AccelerometerGravityMeasurement : public Measurement<3, X, 3>
    {

        // z = normed(accelerometer - state acceleration)
        // zhat = rotated gravity vector
                
        using AccelVec = Vec<3>;

        using GaussianVector<3>::mean;
        using GaussianVector<3>::cov;

        using Measurement<3, X, 3>::PredictionTransform;
    
        using Measurement<3, X, 3>::predicted;
        using Measurement<3, X, 3>::measured;

        AccelerometerGravityMeasurement() : 
            Measurement<3,X,3>(), 
            bias(AccelVec::Zero())
        {}

        void update();
        void transform(); // z
        void predict(); // zhat

        void applied() { state->getSubState<XQuat>().mean.normalize(); }

        AccelVec bias;

    private:

        Vec<3> g_b; // gravity in body frame
        Mat<3,4> dg_b;  // derivative

    };

    struct MagnetometerMeasurement : public StateMeasurement<3, X>
    {

        using Measurement::PredictionTransform;

        using Measurement::state;
        using Measurement::predicted;

        void predict(); // zhat
        void applied() { state->getSubState<XQuat>().mean.normalize(); }

        void setReference(Vec<3> ref, Mat<3,3> cov) 
        {
            reference_field_ = Quaternion(0.0, ref.x(), ref.y(), ref.z());
            ref_cov_ = cov;
        }


        Quaternion reference_field_;
        Mat<3,3> ref_cov_;

    };

    struct BiasMeasurement : public StateMeasurement<6, X>
    {

        using Measurement::PredictionTransform;

        using Measurement::state;
        using Measurement::predicted;

        void predict();

    };

    struct PositionMeasurement : public StateMeasurement<3, X>
    {

        using Measurement::PredictionTransform;

        using Measurement::state;
        using Measurement::predicted;

        void predict();

    };

    struct VelocityMeasurement : public StateMeasurement<3, X>
    {

        using Measurement::PredictionTransform;

        using Measurement::state;
        using Measurement::predicted;

        void predict(); // zhat

    };

    struct CmdVelMeasurement : public StateMeasurement<3, X>
    {

        using Measurement::PredictionTransform;

        using Measurement::state;
        using Measurement::predicted;

        void predict(); // zhat

    };

    struct SonarMeasurement : public StateMeasurement<1, X>
    {

        using Measurement::PredictionTransform;

        using Measurement::state;
        using Measurement::predicted;

        void predict(); // zhat

    };

    struct AltimeterMeasurement : public StateMeasurement<1, X>
    {

        using Measurement::PredictionTransform;

        using Measurement::state;
        using Measurement::predicted;

        void predict(); // zhat

    };

    struct OrientationMeasurement : public StateMeasurement<4, X>
    {
        
        using Measurement::PredictionTransform;

        using Measurement::state;
        using Measurement::predicted;

        void predict(); // zhat

    };

    // system

    struct System : public ekf::System<StateSize, InputSize>
    {

        // Types
        using ekf::System<StateSize, InputSize>::Input;
        using Prediction = ekf::System<StateSize, InputSize>::Transform;

        // Properties
        using ekf::System<StateSize, InputSize>::state;
        using ekf::System<StateSize, InputSize>::prediction;

        // Members
        
        System(State &st) : ekf::System<StateSize, InputSize>(st),
            _quaternion(st.getSubState<XQuat>()),
            _pos(st.getSubState<XPos>()),
            _vel(st.getSubState<XVel>()),
            _bias_accel(st.getSubState<XBiasAccel>()),
            _bias_gyro(st.getSubState<XBiasGyro>())
        {}

        void reset();
        void predict(const Input &u, const double dt);

        inline ekf::Quaternion getQuaternion() { return ekf::Quaternion(_quaternion.mean); }
        inline ekf::Vec<3> getPosition() { return _pos.mean; }

    private:

        XQuat _quaternion;
        XPos _pos;
        XVel _vel;
        XBiasAccel _bias_accel;
        XBiasGyro _bias_gyro;
        
    };


    // filter

    class EKF : public ekf::EKF<System>
    {

    public:

        EKF() : 
            ekf::EKF<System>()
        {
            ROS_INFO("BOOTING EKF");
        }

        using ekf::EKF<System>::system_;

        inline ekf::Quaternion orientation() { return system_.getQuaternion(); };
        inline ekf::Vec<3> position() { return system_.getPosition(); }

    };

};

#endif