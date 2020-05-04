#ifndef EKF_MEASUREMENT_HPP
#define EKF_MEASUREMENT_HPP

#include <ros/ros.h>
#include <iostream>

#include <ekf/eigen/Addons.h>
#include <ekf/state.hpp>


namespace ekf
{

    using namespace std;

    /**
     *  Transforms input := Vector<Dims> to an intermediate representation
     *  z := Vector<IntermediateReprDims>.
     *
     *  Equally, transforms a state := State (could be a substate) into
     *  the intermediate representation zhat := Vector<IntermediateReprDims>.
     */

    template<unsigned int Dims, typename State, unsigned int IntermediateReprDims>
    struct Measurement : public GaussianVector<Dims>
    {
        static_assert(IntermediateReprDims > 0);
        static_assert(is_base_of_any<ekf::State, State>{});

        static const int IntermediateDims = IntermediateReprDims;

        typedef State StateType;
        typedef typename State::SRef StateRef;

        using GaussianVector<Dims>::cov;

        using MeasurementVec = Vec<Dims>;
        using MeasurementCov = Mat<Dims, Dims>;

        using PredictionTransform = StateTransform<State, IntermediateReprDims>;
        using MeasurementTransform = Transform<Dims, IntermediateReprDims>;

        struct Correction
        {
            // some tmp params
            Vec<IntermediateReprDims> y;
            Mat<IntermediateReprDims, IntermediateReprDims> S;
            Mat<State::Dims, IntermediateReprDims> K;
        };

        Measurement() :
            GaussianVector<Dims>(),
            state(nullptr)
        {}

        virtual void update()
        {
            assert(state != nullptr);

            // run
            transform();
            predict();

            // update covariances of intermediate representation
            measured.projectCov(cov);
            predicted.projectCov(state->cov);
        }

        virtual void applied() {}

        /**
         * Transforms the state into the intermediate representation.
         * @param state State
         */
        virtual void predict() {}

        /**
         * Transforms the measured value into
         * the intermediate representation.
         */
        virtual void transform() {}

        /** Sets measurement input */
        virtual void set(MeasurementVec mu, MeasurementCov var, ros::Time time)
        {
            GaussianVector<Dims>::set(mu, var);
            stamp = time;
        }

        virtual void attach(StateRef *st)
        {
            assert(state == nullptr && st != nullptr);
            state = st;
        }

        ~Measurement()
        {
            delete state;
        }


        // attached filter state
        StateRef *state;

        // time-stamp of measurement
        ros::Time stamp;

        // intermediate-representation
        PredictionTransform predicted;
        MeasurementTransform measured;

        // correction for state, used as storage
        Correction correction;

        bool valid = true;

    };


    template<unsigned int Dims, typename State>
    struct StateMeasurement : public Measurement<Dims, State, Dims>
    {

        using MeasurementVec = Vec<Dims>;
        using MeasurementCov = Mat<Dims, Dims>;
        using typename Measurement<Dims, State, Dims>::StateRef;

        using GaussianVector<Dims>::mean;
        using GaussianVector<Dims>::cov;

        using Measurement<Dims, State, Dims>::stamp;
        using Measurement<Dims, State, Dims>::predicted;
        using Measurement<Dims, State, Dims>::measured;

        using Transform = typename Measurement<Dims, State, Dims>::PredictionTransform;

        StateMeasurement() : Measurement<Dims, State, Dims>()
        {
            bias.setZero();

            // state measurement => input equal to measured
            measured.dy = Mat<Dims, Dims>::Identity(); 
            measured.y.setZero();
        };

        void transform()
        {
            measured.y = mean - bias;
        }

        MeasurementVec bias;

    };

};

#endif