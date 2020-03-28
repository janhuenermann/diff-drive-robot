#ifndef EKF_MEASUREMENT_HPP
#define EKF_MEASUREMENT_HPP

#include <drone_ekf/state.hpp>


namespace drone_ekf
{

    /** N = dims of measurement */
    /** S = State class i.e. SubState, State, etc. */

    template<unsigned int N, typename State>
    struct Measurement : public GaussianVariable<N>
    {
        
        static_assert(N > 0);
        static_assert(is_base_of_any<drone_ekf::State, State>{});

        using GaussianVariable<N>::Dims;
        using MeasurementVec = Vec<Dims>;
        using MeasurementCov = Mat<Dims, Dims>;

        /** Transform state -> measurement */
        struct Transform
        {
            using StateJacobi = Mat<Dims, State::Dims>;

            MeasurementVec zhat;
            StateJacobi H;

            /**
             * Returns reference to part of the transformed mean. 
             */
            template <unsigned int K, unsigned int M> 
            inline Ref< Vec<M> > mean()
            {
                // make sure values are in bounds
                static_assert(K+M <= N && K >= 0, "Measurement out of bounds");

                return zhat.template segment<M>(K);
            }

            /**
             * Returns reference to part of the Jacobian.
             * Specifically, zhat(K:M) wrt. state.
             */
            template <unsigned int K, unsigned int M, typename WrtStateType = State>
            inline Ref< Mat<M, WrtStateType::Dims> > jacobi()
            {
                // make sure the state is in bounds
                static_assert(WrtStateType::SegmentIndex >= State::SegmentIndex, "Jacobi out of bounds");
                static_assert(WrtStateType::SegmentIndex + WrtStateType::Dims <= State::SegmentIndex + State::Dims, "Jacobi out of bounds");
                static_assert(K+M <= N && K >= 0, "Measurement out of bounds");

                return H.template block<M, WrtStateType::Dims>(K, WrtStateType::SegmentIndex - State::SegmentIndex);
            }
        };

        struct Correction
        {
            // some tmp params
            Vec<Dims> y;
            Mat<Dims, Dims> S;
            Mat<State::Dims, Dims> K;
        };

        Measurement()
        {
            mean.setZero();
            cov.setZero();            
        };

        /**
         * Updates the prediction
         * @param state State
         */
        virtual void predict(const State &state);


        inline void update(MeasurementVec mu, MeasurementCov var)
        {
            mean = mu;
            cov = var;
        }

        inline Ref<MeasurementVec> getMean() { return mean; };
        inline Ref<MeasurementCov> getCov() { return cov; };

        MeasurementVec mean;
        MeasurementCov cov;

        Transform prediction;
        Correction correction;

    };


};

#endif