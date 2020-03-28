#ifndef EKF_SYSTEM_HPP
#define EKF_SYSTEM_HPP

#include <drone_ekf/state.hpp>


namespace drone_ekf
{

    /** N := StateDims */
    /** M := InputDims */

    template<unsigned int N, unsigned int M>
    struct System
    {
        static_assert(N > 0);

        static const int StateDims = N;
        static const int InputDims = M;

        using State = typename drone_ekf::FullState<N>;
        using Input = typename drone_ekf::Input<M>;

        struct Transform
        {
            using NoiseCov = Mat<N, N>;
            using StateJacobi = Mat<N, N>;
            using InputJacobi = Mat<N, M>;

            StateJacobi F; // dstate_{t}/dstate_{t-1}
            InputJacobi G; // dstate_{t}/dinput
            NoiseCov Q;

            typename State::StateVec x; // next mean of state

            inline void reset()
            {
                x.setZero();
                F.setZero();
                G.setZero();
                Q.setZero();
            }

            template <typename StateType = State> 
            inline Ref<typename StateType::StateVec> mean()
            {
                return x.template segment<StateType::Dims>(StateType::SegmentIndex);
            }

            template <typename VarStateType = State, typename WrtStateType = State>
            inline Ref< Mat<VarStateType::Dims, WrtStateType::Dims> > jacobi()
            {
                return F.template block<VarStateType::Dims, WrtStateType::Dims>(VarStateType::SegmentIndex, WrtStateType::SegmentIndex);
            }
        };

        System(State &state) : state(state) {}

        virtual void reset();
        virtual void predict(const Input &u, const double dt);

        State &state;
        Transform prediction;

    };

};

#endif