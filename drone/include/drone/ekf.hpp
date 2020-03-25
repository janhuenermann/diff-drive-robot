#ifndef EKF_HPP
#define EKF_HPP

#include <ros/ros.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Quaternion.h>

#include <Eigen/Dense>

template <int Rows> 
using Vec = Eigen::Matrix<double, Rows, 1>;

template <int Rows, int Cols> 
using Mat = Eigen::Matrix<double, Rows, Cols>;

template<unsigned int Dims>
class BaseEKF
{

public:

    /**
     * Do a process step.
     * @param F 
     */
    void update();

    /**
     * Correct part of the state.
     * @param z The sensor observation.
     */
    template<unsigned int ZDims>
    void correct(Vec<ZDims> z, Mat<ZDims, ZDims> R, Vec<ZDims> hx, Mat<ZDims, Dims> H);

    /**
     * Processes the next step.
     * @param  F Reference to the Jacobi Mat.
     * @return   Next state f(x_t) = x_{t+1}
     */
    virtual Vec<Dims> processStep(Mat<Dims, Dims> &F, Mat<Dims, Dims> &Q) { throw std::invalid_argument("unimpl"); }

    /** Returns the mean of the state at the current step. */
    inline Vec<Dims>& state()
    {
        return mu_;
    }

    /** Returns the covariance at the current step */
    inline Mat<Dims, Dims>& cov()
    {
        return cov_;
    }

protected:

    Vec<Dims> mu_;
    Mat<Dims, Dims> cov_;

};

#define _DIMS 16
#define _STATE_VAR(name, index) inline double& name() { return mu_(index); }

class SensorEKF : protected BaseEKF<_DIMS>
{
public:

    SensorEKF();

    void update();

    void correctByMagnetometer(const geometry_msgs::Vector3Stamped &msg);
    void correctByIMU(const sensor_msgs::Imu& msg);
    void correctByQuaternion(const geometry_msgs::Quaternion& msg);
    void correctByGPS(const sensor_msgs::NavSatFix& msg);

    inline Vec<4> rotation()
    {
        return state().segment(9, 4);
    }

    // define state variables
    _STATE_VAR(x1, 0);
    _STATE_VAR(x2, 1);
    _STATE_VAR(x3, 2);
    _STATE_VAR(v1, 3);
    _STATE_VAR(v2, 4);
    _STATE_VAR(v3, 5);
    _STATE_VAR(a1, 6);
    _STATE_VAR(a2, 7);
    _STATE_VAR(a3, 8);
    _STATE_VAR(q0, 9);
    _STATE_VAR(q1, 10);
    _STATE_VAR(q2, 11);
    _STATE_VAR(q3, 12);
    _STATE_VAR(w1, 13);
    _STATE_VAR(w2, 14);
    _STATE_VAR(w3, 15);

    inline void setGyroBias(Vec<3> bias)
    {
        gyro_bias_ = bias;
    }

    inline void setAccelerometerBias(Vec<3> bias)
    {
        accel_bias_ = bias;
    }

    inline Vec<16> state()
    {
        return BaseEKF::state();
    }
    inline Mat<16, 16> cov()
    {
        return BaseEKF::cov();
    }

    // direction cosine matrix
    Mat<3, 3> getDCM(bool invert = false);

    ros::Time last_update_time_;
    bool started_;

    Vec<_DIMS> processStep(Mat<_DIMS, _DIMS> &F, Mat<_DIMS, _DIMS> &Q);

protected:

    Vec<3> gyro_bias_;
    Vec<3> accel_bias_;

};

#endif