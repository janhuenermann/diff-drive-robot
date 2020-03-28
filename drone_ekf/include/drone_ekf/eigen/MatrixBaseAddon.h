#ifndef DRONE_EKF_MATRIXBASE_PLUGIN
#define DRONE_EKF_MATRIXBASE_PLUGIN

#ifndef EIGEN_MATRIXBASE_PLUGIN
#define EIGEN_MATRIXBASE_PLUGIN <drone_ekf/eigen/MatrixBaseAddons.h>
#endif

#else  // DRONE_EKF_MATRIXBASE_PLUGIN

bool isApproxSymmetric()
{
    return this->isApprox(this->transpose(), 1e-5);
}

#endif