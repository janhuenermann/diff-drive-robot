#ifndef EKF_TYPES_HPP
#define EKF_TYPES_HPP

/**
 * File contains basic typedefs.
 */

#include <type_traits>
#include <Eigen/Dense>


namespace ekf
{

    /**
     * Table of Contents:
     * 
     * 1. GaussianVariable
     *    Abstract type for variable with mean and cov
     * 
     * 2. GaussianVector : GaussianVariable
     *    Instanstiation that stores mean and cov
     *
     * 3. Transform
     * 4. StateTransform
     * 5. SystemTransform
     * 
     */

    template<int N, int M>
    using Mat = Eigen::Matrix<double, N, M>;
    
    template<int N>
    using Vec = Mat<N, 1>;

    using Quaternion = Eigen::Quaternion<double>;

    template<typename T>
    using Ref = Eigen::Ref<T>;

    /**
     * Abstract data type for a variable with mean and cov.
     */
    template<unsigned int N>
    struct GaussianVariable
    {
        static const int Dims = N;

        typedef Vec<N> MeanVec;
        typedef Mat<N,N> CovMat;

        inline double& operator[](int k) { return getMean()(k); }
        inline double& operator()(int i, int j) { return getCov()(i, j); }

        virtual Ref<MeanVec> getMean() { throw std::logic_error("abstract class"); }
        virtual Ref<CovMat> getCov()   { throw std::logic_error("abstract class"); }
    };

    /**
     * Vector with mean and covariance.
     */
    template<unsigned int N>
    struct GaussianVector : public GaussianVariable<N>
    {

        using typename GaussianVariable<N>::MeanVec;
        using typename GaussianVariable<N>::CovMat;

        GaussianVector() : mean(MeanVec::Zero()), cov(CovMat::Zero())
        {};

        GaussianVector(MeanVec mu, CovMat var) : mean(mu), cov(var) 
        {};

        void set(MeanVec mu, CovMat var)
        {
            mean = mu;
            cov = var;
        }

        virtual Ref<MeanVec> getMean() { return mean; };
        virtual Ref<CovMat> getCov()   { return cov; };

        MeanVec mean;
        CovMat cov;

    };

    /** Variable to variable */
    template<unsigned int InDims, unsigned int OutDims>
    struct Transform
    {

        using InCov = Mat<InDims, InDims>;
        using NoiseCov = Mat<OutDims, OutDims>;
        using Jacobi = Mat<OutDims, InDims>;
        
        Vec<OutDims> y;
        Jacobi dy;

        NoiseCov Q;
        Mat<OutDims, OutDims> cov;

        Transform() { reset(); }

        virtual void reset()
        {
            y.setZero();
            dy.setZero();
            Q.setZero();
        }

        /**
         * Returns reference to part of the transformed mean. 
         */
        template <unsigned int Index, unsigned int Count> 
        inline Ref< Vec<Count> > mean()
        {
            // make sure values are in bounds
            static_assert(Index+Count <= OutDims && Index >= 0, "Slice out of bounds");

            return y.template segment<Count>(Index);
        }

        virtual void projectCov(const InCov &xcov)
        {
            cov = dy * xcov * dy.transpose() + Q;
        }

        template <unsigned int I, unsigned int N, unsigned int J, unsigned M>
        inline Ref< Mat<N, M> > jacobi()
        {
            static_assert(I+N <= OutDims && I >= 0);
            static_assert(J+M <= InDims && J >= 0);

            return dy.template block<N, M>(I, J);
        }

    };

    /** State to variable */
    template<typename InState, unsigned int OutDims>
    struct StateTransform : public Transform<InState::Dims, OutDims>
    {

        using Transform<InState::Dims, OutDims>::dy;

        /**
         * Returns reference to part of the Jacobian.
         * Specifically, zhat(K:M) wrt. state.
         */
        template <unsigned int Index, unsigned int Count, typename WrtStateType = InState>
        inline Ref< Mat<Count, WrtStateType::Dims> > jacobi()
        {
            // make sure the state is in bounds
            static_assert(WrtStateType::SegmentIndex >= InState::SegmentIndex, "Jacobi out of bounds");
            static_assert(WrtStateType::SegmentIndex + WrtStateType::Dims <= InState::SegmentIndex + InState::Dims, "Jacobi out of bounds");
            static_assert(Index+Count <= OutDims && Index >= 0, "Jacobi out of bounds");

            return dy.template block<Count, WrtStateType::Dims>(Index, WrtStateType::SegmentIndex - InState::SegmentIndex);
        }

    };

    /** State (+input) to state */
    template<typename State, unsigned int InputSize>
    struct SystemTransform : public StateTransform<State, State::Dims>
    {
        using NoiseCov = Mat<State::Dims, State::Dims>;
        using InputJacobi = Mat<State::Dims, InputSize>;

        using Transform<State::Dims, State::Dims>::y;
        using Transform<State::Dims, State::Dims>::dy;
        using Transform<State::Dims, State::Dims>::cov;

        using StateCov = Mat<State::Dims, State::Dims>;
        using InputCov = Mat<InputSize,InputSize>;

        InputJacobi dyi; // dstate_{t}/dinput
        NoiseCov Q;

        InputCov inputCov;

        inline void reset()
        {
            Transform<State::Dims,State::Dims>::reset();
            dyi.setZero();
            Q.setZero();
        }

        void projectCov(const StateCov &xcov, const InputCov &incov)
        {
            cov = dy * xcov * dy.transpose() + dyi * incov * dyi.transpose() + Q;
        }

        template <typename StateType = State> 
        inline Ref<typename StateType::StateVec> mean()
        {
            return Transform<State::Dims,State::Dims>::template mean<StateType::SegmentIndex, StateType::Dims>();
        }

        template <typename VarStateType = State, typename WrtStateType = State>
        inline Ref< Mat<VarStateType::Dims, WrtStateType::Dims> > jacobi()
        {
            return StateTransform<State,State::Dims>::template jacobi<VarStateType::SegmentIndex, VarStateType::Dims, WrtStateType>();
        }

        template <typename VarStateType = State, unsigned int Index, unsigned int Count>
        inline Ref< Mat<VarStateType::Dims, Count> > inputJacobi()
        {
            return dyi.template block<VarStateType::Dims, Count>(VarStateType::SegmentIndex, Index);
        }

        template <typename VarStateType = State, typename WrtStateType = VarStateType>
        inline Ref< Mat<VarStateType::Dims, WrtStateType::Dims> > noise()
        {
            return Q.template block<VarStateType::Dims, WrtStateType::Dims>(VarStateType::SegmentIndex, WrtStateType::SegmentIndex);
        } 
    };


    template<unsigned int N>
    using Input = GaussianVector<N>;

};


/** some generic helper methods, ignore */

namespace generic
{
    template <template <auto... > typename C>
    struct is_base_of_any_helper
    {

        template <auto... T>
        std::true_type operator ()(const C<T...>*) const;

        std::false_type operator ()(...) const;

    };
    
}

template <template <auto... > typename C , typename T>
using is_base_of_any = decltype(generic::is_base_of_any_helper<C>{}(std::declval<const T*>()));

#endif
