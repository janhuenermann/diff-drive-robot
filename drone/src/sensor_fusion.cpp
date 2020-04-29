#include <drone_ekf/model.hpp>
#include <drone_ekf/math.hpp>
#include <drone_ekf/types.hpp>

#include <ros/ros.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>

using namespace std;
using namespace drone_ekf_model;
using Vec3 = Eigen::Vector3d;
using Mat33 = Eigen::Matrix<double, 3, 3>;
using drone_ekf::I3;

const double Frequency = 100.0;
const drone_ekf::Input<0> NoInput = drone_ekf::Input<0>();

struct SensorCalibration
{

    Vec<3> magneto_ = {};
    int magneto_measurements_ = 0;

    Vec<3> getMagnetoReference() 
    {
        assert(magneto_measurements_ > 2);
        return magneto_ / (double)magneto_measurements_;
    }

    bool isFine() { return magneto_measurements_ > 10; }

};

class SensorFusion
{
public:

    SensorFusion() : 
        nh_(),
        ekf_(),
        imu_counter_(0)
    {
        ROS_INFO("Booting sensor fusion");

        ekf_.use(gyro_);
        ekf_.use(accel_);
        ekf_.use(magneto_);

        ros::Duration(6.0).sleep(); // weired imu values at start

        sub_imu_ = nh_.subscribe("raw_imu", 1, &SensorFusion::imuCallback,this);
        sub_imu_bias_ = nh_.subscribe("raw_imu/bias", 1, &SensorFusion::imuBiasCallback,this);
        sub_magnetic_ = nh_.subscribe("magnetic", 1, &SensorFusion::magnetometerCallback, this);

        pub_marker_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1, true);

        calibrate();

        tick_timer_ = nh_.createTimer(ros::Duration(1.0 / Frequency), &SensorFusion::tickCallback, this);
    }

    void calibrate()
    {

        SensorCalibration _calib;
        calibration_ = &_calib;

        ROS_INFO("Starting calibration.");

        do
        {

            ros::spinOnce();

            if (calibration_->isFine())
            {
                applyCalibration();
                calibration_ = nullptr;
            }


        }
        while (calibration_ != nullptr);
    }

    void applyCalibration()
    {
        ROS_DEBUG_STREAM("Magneto calibration : \n" << calibration_->getMagnetoReference());
        magneto_.setReference(calibration_->getMagnetoReference());

        calibrated_ = true;

        ROS_INFO("Calibration done.");
    }

    void tickCallback(const ros::TimerEvent& evt)
    {
        ekf_.step(NoInput);
        // ekf_.system_.gyro.cov = 0.10 * I3;

        publishMarker(ekf_.orientation());
    }

    void imuCallback(const sensor_msgs::Imu& msg)
    {
        if (!calibrated_)
        {
            return ;
        }

        Vec3 linear_acceleration = Vec3(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
        Vec3 angular_velocity = Vec3(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
        
        Mat33 linear_acceleration_cov = I3 * 12.00;
        Mat33 angular_velocity_cov = I3 * 0.02;

        gyro_.set(
            angular_velocity,
            angular_velocity_cov,
            msg.header.stamp
        );

        accel_.set(
            linear_acceleration,
            linear_acceleration_cov,
            msg.header.stamp
        );

        ekf_.correct(gyro_);

        // only correct every 5th step
        if (imu_counter_ % 5 == 0)
        {

            ekf_.correct(accel_);
            
        }

        imu_counter_ += 1;
    }

    void imuBiasCallback(const sensor_msgs::Imu& msg)
    {
        accel_.bias = Vec3(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
        gyro_.bias = Vec3(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
    }

    void magnetometerCallback(const geometry_msgs::Vector3Stamped& msg)
    {
        Vec3 field(msg.vector.x, msg.vector.y, msg.vector.z);
        Mat33 field_cov = 1.00 * I3;

        if (!calibrated_)
        {
            calibration_->magneto_ += field;
            calibration_->magneto_measurements_ += 1;
        }

        magneto_.set(field, field_cov, msg.header.stamp);
        ekf_.correct(magneto_);
    }

protected:

    GyroMeasurement gyro_;
    AccelerometerMeasurement accel_;
    MagnetometerMeasurement magneto_;

    EKF ekf_;

    ros::Subscriber sub_imu_;
    ros::Subscriber sub_imu_bias_;
    ros::Subscriber sub_magnetic_;

    ros::Publisher pub_marker_;

    ros::NodeHandle nh_;
    ros::Timer tick_timer_;

    int imu_counter_;

    SensorCalibration *calibration_;
    bool calibrated_ = false;

private:

    void publishMarker(Quaternion estimate)
    {
        visualization_msgs::MarkerArray markers;
        visualization_msgs::Marker marker;

        // marker blueprint
        marker.header.frame_id = "/drone_tf/world";
        marker.header.stamp = ros::Time::now();

        marker.ns = "base";
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;

        marker.color.a = 0.9f;

        vector<Quaternion> qs;
        qs.push_back(estimate * Quaternion(1.0,0,0,0));                    // x
        qs.push_back(estimate * Quaternion(1.0/M_SQRT2,0,0,1.0/M_SQRT2));  // y
        qs.push_back(estimate * Quaternion(1.0/M_SQRT2,0,-1.0/M_SQRT2,0)); // z

        int id = 0;
        // add orientation arrows
        for (const Quaternion &q : qs)
        {
            marker.id = ++id;
            marker.type = visualization_msgs::Marker::ARROW;

            marker.pose.orientation.x = q.x();
            marker.pose.orientation.y = q.y();
            marker.pose.orientation.z = q.z();
            marker.pose.orientation.w = q.w();

            marker.scale.x = 1.0;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;

            float r = 0.0f, g = 0.0f, b = 0.0f;
            if (id == 1) r = 1.0f;
            if (id == 2) g = 1.0f;
            if (id == 3) b = 1.0f;

            marker.color.r = r;
            marker.color.g = g;
            marker.color.b = b;

            marker.lifetime = ros::Duration();
            markers.markers.push_back(marker);
        }

        // add position sphere
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.id = ++id;
        marker.pose.orientation.w = 1.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.color.r = 70.0/255.0;
        marker.color.g = 130.0/255.0;
        marker.color.b = 180.0/255.0;
        marker.color.a = 1.0f;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;

        markers.markers.push_back(marker);

        pub_marker_.publish(markers);
    }

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_fusion");

    SensorFusion node;
    ros::spin();
}