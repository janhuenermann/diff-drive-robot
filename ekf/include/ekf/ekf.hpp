#ifndef EKF_HPP
#define EKF_HPP

#include <ekf/eigen/Addons.h>

#include <ekf/types.hpp>
#include <ekf/measurement.hpp>
#include <ekf/system.hpp>
#include <ekf/state.hpp>

#include <ros/ros.h>
#include <vector>
#include <type_traits>
#include <iostream>

#include <Eigen/Dense>


namespace ekf
{

    using std::is_base_of;
    using std::vector;
    using std::cout;
    using std::endl;

    /** N = state dims */
    /** M = input dims */

    template<typename System>
    class EKF
    {

    public:

        static_assert(is_base_of_any<ekf::System, System>{});

        static const unsigned int StateDims = System::StateDims;
        static const unsigned int InputDims = System::InputDims;

        using State = typename System::State;
        using Input = typename System::Input;

        using StateVec = typename State::StateVec;
        using StateCov = typename State::StateCov;

        static_assert(is_base_of<ekf::FullState<16>, State>{});

        EKF() : 
            state_(), 
            system_(state_) 
        {
            ROS_INFO("BOOTING EKF WHAETEVER");
            reset();
        }

        void reset()
        {
            running_ = false;
            last_step_time_ = ros::Time(0);
            system_.reset();
        }

        /**
         * Inits a measurement for use with this filter
         * @param m Measurement
         */
        template<typename Measure>
        void use(Measure &m)
        {
            m.attach(Measure::StateType::createRefFromFullState(state_));
        }

        /**
         * Next step
         */
        void step(const Input &u)
        {
            const ros::Time now = ros::Time::now();
            typename System::Transform &pred = system_.prediction;

            double dt = 0.0;

            if (running_)
            {
                dt = (now - last_step_time_).toSec();
                assert(dt > 0.0);
            }

            // runs f(x, u) = system_.transform.x, gives us F and G (derivatives) in system_.transform
            system_.predict(u, dt);

            // project cov
            pred.projectCov(state_.cov, u.cov);

            // sanity checks
            assert(u.cov.isApproxSymmetric());
            assert(pred.Q.isApproxSymmetric());

            // update state by copying over data and calculating new cov
            state_.mean.noalias() = pred.y;
            state_.cov.noalias()  = pred.cov;

            // sanity checks (important!)
            assert(state_.cov.isApproxSymmetric());

            running_ = true;
            last_step_time_ = now;

            // ROS_INFO_STREAM("state mean --- \n" << state_.mean);
            // ROS_INFO_STREAM("state cov ---- \n" << state_.cov);
        }

        /**
         * Corrects the state of the filter based on measurements
         */
        template<typename Measure>
        void correct(Measure &m)
        {
            // static_assert(is_measurement_class<ekf::Measurement, Measurement>{});
            const typename Measure::StateRef::StateCov I = Measure::StateRef::StateCov::Identity();
            
            // intermediate repr z
            typename Measure::MeasurementTransform &z = m.measured;
            typename Measure::PredictionTransform &zhat = m.predicted;

            typename Measure::Correction &corr = m.correction;
            typename Measure::StateRef *&st = m.state;

            // transform state to predicted measurement
            m.update();

            if (!m.valid) return ;

            // sanity check
            assert(z.cov.isApproxSymmetric());
            assert(zhat.cov.isApproxSymmetric());

            corr.y.noalias() = z.y - zhat.y;
            corr.S.noalias() = zhat.cov + z.cov;
            corr.K.noalias() = st->cov * zhat.dy.transpose() * corr.S.inverse();

            #ifndef NDEBUG
            if (abs(corr.S.determinant()) < 1e-8)
            {
                ROS_WARN("det S very small.");
            }
            #endif

            if (corr.S.diagonal().array().abs().maxCoeff() < 1e-9)
            {
                ROS_DEBUG("S matrix is looking not good. Bailing out. ");
                ROS_DEBUG_STREAM("Correction type : " << typeid(Measure).name());

                ROS_DEBUG_STREAM("y = \n " << corr.y);
                ROS_DEBUG_STREAM("z cov = \n " << z.cov);
                ROS_DEBUG_STREAM("z hat cov = \n " << zhat.cov);
                ROS_DEBUG_STREAM("S = \n " << corr.S);
                ROS_DEBUG_STREAM("K = \n " << corr.K);

                return ;
            }

            // apply correction
            st->mean = st->mean + corr.K * corr.y;
            st->cov  = (I - corr.K * zhat.dy) * st->cov;

            m.applied();

            // sanity checks (important!)
            assert(st->cov.isApproxSymmetric());
        }


        State state_;
        System system_;

    protected:

        ros::Time last_step_time_;
        bool running_;

    };

};


#endif