#ifndef EKF_STATE_HPP
#define EKF_STATE_HPP

/**
 * File contains state classes that manage the
 * EKF's state.
 */

#include <drone_ekf/types.hpp>

#include <Eigen/Dense>


namespace drone_ekf
{

    /** Base state. Can either be a sub-, or full-state. */
    template<unsigned int N>
    struct State : public GaussianVariable<N>
    {

    public:

        static_assert(N > 0);

        using Me = State<N>;
        using StateVec = typename GaussianVariable<N>::MeanVec;
        using StateCov = typename GaussianVariable<N>::CovMat;

        State(Ref<StateVec> mu, Ref<StateCov> var) : mean(mu), cov(var) {}

        Ref<StateVec> getMean() { return mean; }
        Ref<StateCov> getCov()  { return cov; }

        Ref<StateVec> mean;
        Ref<StateCov> cov;

    };

    template<typename OwnerState, unsigned int Index, unsigned int N>
    struct SubState;

    /** Contains all data necessary to describe a system state. */
    template<unsigned int N>
    struct FullState : public State<N>
    {

    public:

        static const unsigned int SegmentIndex = 0;

        using Me = FullState<N>;

        using typename State<N>::StateVec;
        using typename State<N>::StateCov;

        using State<N>::mean;
        using State<N>::cov;

        struct Data
        {
            StateVec mean;
            StateCov cov;
        };

        FullState() : State<N>(storage.mean, storage.cov)
        {
            mean.setZero();
            cov.setZero();
        };

        /** Members */
        template<unsigned int Index, unsigned int M>
        SubState<Me, Index, M> getSubState();

        template<typename S>
        S getSubState();

        /** State constructor */
        inline static Me construct(Me &state) { return state; }

        /** Properties */
        Data storage;

    };

    /** Refers to a part of the full state, e.g. just the quaternion sub-state. */
    template<typename OwnerState, unsigned int Index, unsigned int N>
    struct SubState : public State<N>
    {

    public:

        static_assert(is_base_of_any<FullState, OwnerState>{}, "OwnerState must be of FullState type");
        static_assert(OwnerState::Dims > Index + N, "SubState range is out of bounds");

        static const unsigned int SegmentIndex = Index;

        using Me = SubState<OwnerState, Index, N>;
        using State<N>::StateVec;
        using State<N>::StateCov;

        SubState(OwnerState &owner) :
            State<N>(owner.mean.segment<N>(Index),
                     owner.cov.block<N,N>(Index, Index)) {};

        /** State constructor */
        inline static Me construct(OwnerState &state) { return state.template getSubState<Index, N>(); };

    };

};


#endif