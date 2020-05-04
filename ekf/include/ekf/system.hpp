#ifndef EKF_SYSTEM_HPP
#define EKF_SYSTEM_HPP

#include <ekf/types.hpp>
#include <ekf/state.hpp>


namespace ekf
{

    /** N := StateDims */
    /** M := InputDims */

    template<unsigned int N, unsigned int M>
    struct System
    {
        static_assert(N > 0);

        static const int StateDims = N;
        static const int InputDims = M;

        typedef typename ekf::FullState<N> State;
        typedef typename ekf::Input<M> Input;

        typedef SystemTransform<State, M> Transform;


        System(State &state) : state(state) {}

        virtual void reset() {}
        virtual void predict(const Input &u, const double dt) {}


        State &state;
        Transform prediction;

    };

};

#endif