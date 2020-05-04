#ifndef EKF_STATE_HPP
#define EKF_STATE_HPP

/**
 * File contains state classes that manage the
 * EKF's state.
 */

#include <ekf/eigen/Addons.h>
#include <ekf/types.hpp>

#include <Eigen/Dense>


namespace ekf
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

        typedef SubState<Me, 0, N> SRef; // StateRef


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
        template<typename S>
        inline S getSubState()
        {
            return S(*this);
        }


        /** Properties */
        Data storage;


        static SRef *createRefFromFullState(Me &st)
        {
            return new SRef(st);
        }

    };

    /** Refers to a part of the full state, e.g. just the quaternion sub-state. */
    template<typename OwnerState, unsigned int Index, unsigned int N>
    struct SubState : public State<N>
    {

    public:

        using Me = SubState<OwnerState, Index, N>;

        static const int SegmentIndex = Index;

        typedef OwnerState Owner;
        typedef Me SRef; // StateRef

        SubState(OwnerState &st) :
            State<N>(st.mean.template segment<N>(Index),
                     st.cov.template block<N, N>(Index,Index)),
            owner(st)
        {}

        template<typename S>
        inline S getSubState()
        {
            return owner.template getSubState<S>();
        }

        template<typename S>
        inline S *getSubStateRef()
        {
            return S::createRefFromFullState(owner); // owner.template getSubState<S>();
        }

        OwnerState &owner;

        static SRef *createRefFromFullState(OwnerState &st)
        {
            return new Me(st);
        }
        
    };

};


#endif