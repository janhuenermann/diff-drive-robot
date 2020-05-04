#include <ekf/model.hpp>
#include <ekf/math.hpp>
#include <ekf/types.hpp>

#include <ros/ros.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/NavSatFix.h>
#include <hector_uav_msgs/Altimeter.h>

#include <drone/gps_utils.hpp>
#include <drone/math.hpp>

using namespace std;
using namespace ekf_model;

using ekf::I3;
using ekf::Input;


struct SensorCalibration
{

    vector<Vec3> magneto_ = {};
    vector<Vec3> ecef_ = {};
    vector<Vec3> vel_ = {};
    vector<Vec1> sonar_ = {};
    vector<Vec1> altitude_ = {};

    // we can only use these messages once we have an initial ecef estimate
    vector<sensor_msgs::NavSatFix> pos_msgs_ = {};

    Stats<3> magneto_initial_;
    Stats<3> ecef_initial_;
    Stats<3> vel_initial_;
    Stats<3> pos_initial_;
    Stats<1> sonar_initial_;
    Stats<1> altitude_initial_;

    Vec3 magneto_ref_mu_;
    Mat33 magneto_ref_cov_;

    Vec3 world_offset_;

    void finalize()
    {
        magneto_initial_ = getStats(magneto_);
        ecef_initial_ = getStats(ecef_);
        vel_initial_ = getStats(vel_);
        sonar_initial_ = getStats(sonar_);
        altitude_initial_ = getStats(altitude_);

        // world origin is offset by initial
        world_offset_.setZero();
        world_offset_(2) = -sonar_initial_.mean(0);

        vector<Vec3> pos = {};
        for (const sensor_msgs::NavSatFix& fix : pos_msgs_)
        {
            pos.push_back(GPS::convertMeasurement(fix, ecef_initial_.mean, world_offset_));
        }

        pos_initial_ = getStats(pos);

        magneto_ref_mu_(0) = magneto_initial_.mean.head(2).norm();
        magneto_ref_mu_(1) = 0;
        magneto_ref_mu_(2) = magneto_initial_.mean(2);

        magneto_ref_cov_.setZero();
        magneto_ref_cov_(0,0) = magneto_initial_.cov(0,0) + magneto_initial_.cov(1,1);
        magneto_ref_cov_(2,2) = magneto_initial_.cov(2,2);
    }

    bool isDone() 
    { 
        return magneto_.size() >= 10 
            && ecef_.size() >= 20 
            && vel_.size() >= 5
            && sonar_.size() >= 8
            && altitude_.size() >= 8;
    }

    Vec3 getReferenceECEF() { assert(isDone()); return ecef_initial_.mean; }

    Vec3 getMagnetoReference() { assert(isDone());  return magneto_ref_mu_; }
    Mat33 getMagnetoCov() { assert(isDone()); return magneto_ref_cov_; }

    Vec3 getPositionReference() { assert(isDone()); return pos_initial_.mean; }
    Mat33 getPositionCov() { assert(isDone()); return pos_initial_.cov; }

    Vec3 getInitialVelocity() { assert(isDone()); return vel_initial_.mean; }
    Mat33 getInitialVelocityCov() { assert(isDone()); return vel_initial_.cov; }

    double getAltimeterReference() { assert(isDone()); return altitude_initial_.mean(0) - world_offset_.z(); }

    Vec3 getWorldOffset() { return world_offset_; }

};

class SensorFusion
{
public:

    SensorFusion() : 
        nh_(),
        ekf_(),
        imu_counter_(0),
        calibration_(new SensorCalibration())
    {
        ROS_INFO("Booting sensor fusion");

        ekf_.use(accel_gravity_);
        ekf_.use(magneto_);
        ekf_.use(position_);
        ekf_.use(velocity_);
        ekf_.use(sonar_);
        ekf_.use(altimeter_);
        ekf_.use(bias_);

        ros::Duration(6.0).sleep(); // weired imu values at start

        sub_imu_ = nh_.subscribe("raw_imu", 1, &SensorFusion::imuCallback,this);
        sub_imu_bias_ = nh_.subscribe("raw_imu/bias", 1, &SensorFusion::imuBiasCallback,this);
        sub_magnetic_ = nh_.subscribe("magnetic", 1, &SensorFusion::magnetometerCallback, this);
        sub_gps_pos_ = nh_.subscribe("fix", 1, &SensorFusion::positionCallback, this);
        sub_gps_vel_ = nh_.subscribe("fix_velocity", 1, &SensorFusion::velocityCallback, this);
        sub_sonar_ = nh_.subscribe("sonar_height", 1, &SensorFusion::sonarCallback, this);
        sub_altimeter_ = nh_.subscribe("altimeter", 1, &SensorFusion::altimeterCallback, this);

        pub_marker_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1, true);

        calibrate();

        tick_timer_ = nh_.createTimer(ros::Duration(1.0 / 20.0), &SensorFusion::tickCallback, this);
    }

    ~SensorFusion()
    {
        delete calibration_;
    }

    void calibrate()
    {
        ROS_INFO("Calibrating...");

        do
        {
            ros::spinOnce();

            if (calibration_->isDone())
            {
                applyCalibration();
            }
        }
        while (!isCalibrated());
    }

    void applyCalibration()
    {
        calibration_->finalize();  // Calculate statistics

        // Set magnetometer reference
        magneto_.setReference(calibration_->getMagnetoReference(), calibration_->getMagnetoCov());

        // Update initial ekf state
        XPos x_pos = ekf_.state_.getSubState<XPos>();
        x_pos.mean = calibration_->getPositionReference();
        x_pos.cov = calibration_->getPositionCov();

        XVel x_vel = ekf_.state_.getSubState<XVel>();
        x_vel.mean = Vec3(0.0,0.0,0.0);
        x_vel.cov = calibration_->getInitialVelocityCov();

        ROS_INFO_STREAM("Initial position: \n " << calibration_->pos_initial_.mean << "\n with cov: \n" << calibration_->pos_initial_.cov);
        ROS_INFO_STREAM("Initial velocity: \n " << calibration_->vel_initial_.mean << "\n with cov: \n" << calibration_->vel_initial_.cov);

        ROS_INFO("Calibration done.");

        calibrated_ = true;
    }

    void tickCallback(const ros::TimerEvent& evt)
    {
        publishMarker(ekf_.position(), ekf_.orientation());
    }

    void imuCallback(const sensor_msgs::Imu& msg)
    {
        if (!isCalibrated())
        {
            return ;
        }

        // Get sensor data
        Vec3 linear_acceleration = Vec3(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
        Vec3 angular_velocity = Vec3(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
        
        Mat33 linear_acceleration_cov = I3 * 0.125;
        Mat33 angular_velocity_cov = I3 * 0.01;

        // Update input
        input.mean.setZero();
        input.cov.setZero();

        input.mean.segment<3>(0) = linear_acceleration;
        input.cov.block<3,3>(0,0) = linear_acceleration_cov;

        input.mean.segment<3>(3) = angular_velocity;
        input.cov.block<3,3>(3,3) = angular_velocity_cov;

        // Correct gravity every 6th step
        if (imu_counter_ % 6 == 0)
        {
            accel_gravity_.set(
                linear_acceleration,
                2.0 * linear_acceleration_cov,
                msg.header.stamp);

            ekf_.correct(accel_gravity_);
        }
        
        ekf_.step(input);

        imu_counter_ += 1;
    }

    void imuBiasCallback(const sensor_msgs::Imu& msg)
    {
        accel_bias_ = Vec3(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
        gyro_bias_ = Vec3(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);

        accel_gravity_.bias = accel_bias_;

        bias_.mean.segment<3>(0) = accel_bias_;
        bias_.cov.block<3,3>(0,0) = 0.10 * I3;

        bias_.mean.segment<3>(3) = gyro_bias_;
        bias_.cov.block<3,3>(3,3) = 0.01 * I3;

        ekf_.correct(bias_);
    }

    void magnetometerCallback(const geometry_msgs::Vector3Stamped& msg)
    {
        Vec3 field(msg.vector.x, msg.vector.y, msg.vector.z);
        Mat33 field_cov = 0.05 * I3;

        if (!isCalibrated())
        {
            calibration_->magneto_.push_back(field);
            return ;
        }

        magneto_.set(field, field_cov, msg.header.stamp);
        ekf_.correct(magneto_);
    }

    void positionCallback(const sensor_msgs::NavSatFix& msg)
    {
        if (!isCalibrated())
        {
            Vec3 ecef = GPS::GPS2ECEF(msg);

            calibration_->ecef_.push_back(ecef);
            calibration_->pos_msgs_.push_back(msg);

            return ;
        }

        Vec3 xyz = GPS::convertMeasurement(msg, 
            calibration_->getReferenceECEF(), 
            calibration_->getWorldOffset());

        Mat33 xyz_cov = 0.04 * I3;

        position_.set(xyz, xyz_cov, msg.header.stamp);
        ekf_.correct(position_);
    }

    void velocityCallback(const geometry_msgs::Vector3Stamped& msg)
    {
        Vec3 vel(msg.vector.x, msg.vector.y, msg.vector.z);
        Mat33 vel_cov = 0.02 * I3;

        if (!isCalibrated())
        {
            calibration_->vel_.push_back(vel);
            return ;
        }

        velocity_.set(vel, vel_cov, msg.header.stamp);
        ekf_.correct(velocity_);
    }

    void sonarCallback(const sensor_msgs::Range &msg)
    {
        const double range_cov = 0.0001;
        double range = (double)msg.range;

        if (!isCalibrated())
        {
            calibration_->sonar_.push_back(Vec1(range));
            return ;
        }

        // If out of range, do not use sonar
        if (range >= 2.95)
        {
            return ;
        }

        sonar_.set(Vec1(range), Vec1(range_cov), msg.header.stamp);
        ekf_.correct(sonar_);
    }

    void altimeterCallback(const hector_uav_msgs::Altimeter &msg)
    {
        const double altitude_cov = 0.15;
        double altitude = (double)msg.altitude;

        if (!isCalibrated())
        {
            calibration_->altitude_.push_back(Vec1(altitude));
            return ;
        }

        altitude -= calibration_->getAltimeterReference();
        altimeter_.set(Vec1(altitude), Vec1(altitude_cov), msg.header.stamp);
        ekf_.correct(altimeter_);
    }

    bool isCalibrated() { return calibrated_; }

protected:

    /** EKF */
    EKF ekf_;
    Input<6> input;

    /** EKF correctors */
    AccelerometerGravityMeasurement accel_gravity_;
    MagnetometerMeasurement magneto_;
    PositionMeasurement position_;
    VelocityMeasurement velocity_;
    SonarMeasurement sonar_;
    AltimeterMeasurement altimeter_;
    BiasMeasurement bias_;

    /** Subscribers */
    ros::Subscriber sub_imu_;
    ros::Subscriber sub_imu_bias_;
    ros::Subscriber sub_magnetic_;
    ros::Subscriber sub_gps_pos_;
    ros::Subscriber sub_gps_vel_;
    ros::Subscriber sub_sonar_;
    ros::Subscriber sub_altimeter_;

    /** Debug publishers */
    ros::Publisher pub_marker_;

    /** Misc */
    ros::NodeHandle nh_;
    ros::Timer tick_timer_;
    int imu_counter_;
    int bias_counter_ = 0;

    /** Calibration */
    SensorCalibration *calibration_;
    bool calibrated_ = false;

    /** Sensor bias */
    Vec3 accel_bias_;
    Vec3 gyro_bias_;

private:

    void publishMarker(Vec<3> pos, Quaternion estimate)
    {
        visualization_msgs::MarkerArray markers;
        visualization_msgs::Marker marker;

        // marker blueprint
        marker.header.frame_id = "/drone_tf/world";
        marker.header.stamp = ros::Time::now();

        marker.ns = "base";
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = pos.x();
        marker.pose.position.y = pos.y();
        marker.pose.position.z = pos.z();

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