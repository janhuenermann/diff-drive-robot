#include <ros/ros.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>

#include <drone/ekf.hpp>
#include <drone/geometry.hpp>

#include <Eigen/Dense>

using namespace std;
// using namespace Eigen;

const double Frequency = 50.0;

class SensorFusion
{
public:

    SensorFusion() : 
        nh_() //, ekf_()
    {
        ros::Duration(6.0).sleep();
        sub_imu_ = nh_.subscribe("raw_imu", 1, &SensorFusion::imuCallback,this);
        sub_imu_bias_ = nh_.subscribe("raw_imu/bias", 1, &SensorFusion::imuBiasCallback,this);

        sub_magnetic_ = nh_.subscribe("magnetic", 1, &SensorFusion::magnetometerCallback, this);

        pub_marker_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);

        tick_timer_ = nh_.createTimer(ros::Duration(1.0 / Frequency), &SensorFusion::tickCallback, this);
    }

    void publishMarker(Quaternion estimate)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/drone_tf/world";
        marker.header.stamp = ros::Time::now();

        marker.ns = "base";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::ARROW;

        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;

        marker.pose.orientation.x = estimate.x();
        marker.pose.orientation.y = estimate.y();
        marker.pose.orientation.z = estimate.z();
        marker.pose.orientation.w = estimate.w();

        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0f;

        marker.lifetime = ros::Duration();

        pub_marker_.publish(marker);
    }

    void tickCallback(const ros::TimerEvent& evt)
    {
        publishMarker(ekf_.rotation());
        

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

        Quaternion imu = Quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
        Quaternion ekf = ekf_.rotation();

        EulerAngles ekf_euler = ToEulerAngles(ekf);

        ROS_INFO("imu: %.3lf, ekf: %.3lf %.3lf %.3lf", ToEulerAngles(imu).yaw, ekf_euler.roll, ekf_euler.pitch, ekf_euler.yaw);

    }

    void imuBiasCallback(const sensor_msgs::Imu& msg)
    {
        ekf_.setGyroBias(Eigen::Vector3d(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z));
        ekf_.setAccelerometerBias(Eigen::Vector3d(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z));
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

    ros::Publisher pub_marker_;

    ros::NodeHandle nh_;
    ros::Timer tick_timer_;

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_fusion");

    SensorFusion node;
    ros::spin();
}