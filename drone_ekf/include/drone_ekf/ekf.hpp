#ifndef EKF_HPP
#define EKF_HPP

#include <drone_ekf/types.hpp>
#include <drone_ekf/measurement.hpp>
#include <drone_ekf/system.hpp>
#include <drone_ekf/state.hpp>

#include <ros/ros.h>


namespace drone_ekf
{

    /** N = state dims */
    /** M = input dims */

    template<typename System>
    class EKF
    {

        static_assert(is_base_of_any<drone_ekf::System, System>{});

        static const unsigned int StateDims = System::StateDims;
        static const unsigned int InputDims = System::InputDims;

        using State = typename System::State;
        using Input = typename System::Input;

        using StateVec = typename State::StateVec;
        using StateCov = typename State::StateCov;

        EKF() : state_(), system_(state_) 
        {
            reset();
        }

        void reset();

        /**
         * Next step
         */
        void step(const Input &u);

        /**
         * Corrects the state of the filter based on measurements
         */
        template<typename Measurement>
        void correct(const Measurement &Z);

        State state_;
        System system_;

    protected:

        ros::Time last_step_time_;
        bool running_;

        StateVec __mean; // tmp
        StateCov __cov; // tmp

    };

};


#endif