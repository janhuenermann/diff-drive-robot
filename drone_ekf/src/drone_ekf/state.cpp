#include <drone_ekf/state.hpp>

using namespace drone_ekf;

template<unsigned int N>
template<unsigned int Index, unsigned int M>
SubState<FullState<N>, Index, M> FullState<N>::getSubState()
{
    return SubState<Me, Index, M>(*this);
}

template<unsigned int N>
template<typename S>
S FullState<N>::getSubState()
{
    return S(*this);
}