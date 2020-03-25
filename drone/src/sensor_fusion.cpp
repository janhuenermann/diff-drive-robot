#include <ros/ros.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>

#include <drone/ekf.hpp>

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

const double Frequency = 50.0;

class SensorFusion
{
public:

    SensorFusion() : 
        nh_() //, ekf_()
    {
        sub_imu_ = nh_.subscribe("raw_imu", 1, &SensorFusion::imuCallback,this);
        sub_imu_bias_ = nh_.subscribe("raw_imu/bias", 1, &SensorFusion::imuBiasCallback,this);

        sub_magnetic_ = nh_.subscribe("magnetic", 1, &SensorFusion::magnetometerCallback, this);

        tick_timer_ = nh_.createTimer(ros::Duration(1.0 / Frequency), &SensorFusion::tickCallback, this);
    }

    void tickCallback(const ros::TimerEvent& evt)
    {
        // ekf_.update();

        // cout << ekf_.state() << endl;
        // cout << ekf_.cov() << endl;
        // cout << ekf_.state() << endl;

        // Eigen::Vector4d q = ekf_.rotation();
        // ROS_INFO("%.3f %.3f %.3f %.3f", q(0), q(1), q(2), q(3));
    }

    void imuCallback(const sensor_msgs::Imu& msg)
    {
        ekf_.update();
        ekf_.correctByIMU(msg);

        ROS_INFO("imu: %.3lf, %.3lf, %.3lf, %.3lf", msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
        ROS_INFO("ekf: %.3lf, %.3lf, %.3lf, %.3lf", ekf_.q0(), ekf_.q1(), ekf_.q2(), ekf_.q3());

    }

    void imuBiasCallback(const sensor_msgs::Imu& msg)
    {
        ekf_.setGyroBias(Vector3d(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z));
        ekf_.setAccelerometerBias(Vector3d(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z));
    }

    void magnetometerCallback(const geometry_msgs::Vector3Stamped& msg)
    {
        // ekf_.update();
        ekf_.correctByMagnetometer(msg);

        // ROS_INFO("%.3lf, %.3lf, %.3lf", ekf_.w1(), ekf_.w2(), ekf_.w3());
    }

protected:

    SensorEKF ekf_;

    ros::Subscriber sub_imu_;
    ros::Subscriber sub_imu_bias_;
    ros::Subscriber sub_magnetic_;

    ros::NodeHandle nh_;
    ros::Timer tick_timer_;

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_fusion");

    SensorFusion node;
    ros::spin();
}