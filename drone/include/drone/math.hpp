#ifndef DRONE_MATH_HPP
#define DRONE_MATH_HPP


#include <Eigen/Dense>
#include <vector>


template<int N, int M>
using Mat = Eigen::Matrix<double, N, M>;
using Mat33 = Mat<3,3>;

template<int N>
using Vec = Mat<N, 1>;
using Vec3 = Vec<3>;
using Vec1 = Vec<1>;


template<int N>
struct Stats
{
    Vec<N> mean;
    Mat<N, N> cov;
};


/**
 * Calculates the mean and covariance of given samples
 */
template<int N>
Stats<N> getStats(const std::vector<Vec<N>> &m)
{
    Stats<N> out;
    out.mean.setZero();

    for (int k = 0; k < N; ++k)
    {
        for (int j = 0; j < m.size(); ++j)
        {
            out.mean[k] += m[j](k);
        }
    }

    out.mean /= (double)m.size();
    out.cov.setZero();

    for (int k = 0; k < N; ++k)
    {
        for (int l = 0; l <= k; ++l)
        {

            // Cov[k, l] = E[k*l] - E[k]*E[l]
            for (int j = 0; j < m.size(); ++j)
            {
                out.cov(k, l) += m[j](k) * m[j](l);
            }

            out.cov(k, l) /= (double)m.size();
            out.cov(k, l) -= out.mean[k] * out.mean[l];

            // symmetry
            out.cov(l, k) = out.cov(k, l);

        }
    }

    return out;
}


#endif