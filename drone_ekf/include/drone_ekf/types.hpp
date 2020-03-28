#ifndef EKF_TYPES_HPP
#define EKF_TYPES_HPP

/**
 * File contains basic typedefs.
 */

#include <type_traits>
#include <Eigen/Dense>


namespace drone_ekf
{

    template<unsigned int N, unsigned int M>
    using Mat = Eigen::Matrix<double, N, M>;
    
    template<unsigned int N>
    using Vec = Mat<N, 1>;

    using Quaternion = Eigen::Quaternion<double>;

    template<typename T>
    using Ref = Eigen::Ref<T>;

    /**
     * Basic data type for a variable with mean and cov.
     */
    template<unsigned int N>
    struct GaussianVariable
    {
        static const int Dims = N;

        using MeanVec = Vec<N>;
        using CovMat = Mat<N,N>;

        inline double& operator[](int k) { return getMean()(k); }
        inline const double& operator[](int k) const { return getMean()(k); }

        inline double& operator()(int i, int j) { return getCov()(i, j); }
        inline const double& operator()(int i, int j) const { return getCov()(i, j); }

        virtual Ref<MeanVec> getMean();
        virtual Ref<CovMat> getCov();
    };

    /** Helper gaussian variable to store input */
    template<unsigned int N>
    struct Input : public GaussianVariable<N>
    {
        using InputVec = typename GaussianVariable<N>::MeanVec;
        using InputCov = typename GaussianVariable<N>::CovMat;

        inline virtual Ref<InputVec>& getMean() { return mean; };
        inline virtual Ref<InputCov>& getCov() { return cov; };

        InputVec mean;
        InputCov cov;

    };

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
