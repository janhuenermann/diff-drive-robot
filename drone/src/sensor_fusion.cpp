#include <ekf/model.hpp>
#include <ekf/math.hpp>
#include <ekf/types.hpp>

#include <ros/ros.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>

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
    vector<Vec<6>> bias_ = {};
    vector<Vec3> start_pose_ = {};

    // we can only use these messages once we have an initial ecef estimate
    vector<sensor_msgs::NavSatFix> pos_msgs_ = {};

    Stats<3> magneto_initial_;
    Stats<3> ecef_initial_;
    Stats<3> vel_initial_;
    Stats<3> pos_initial_;
    Stats<1> sonar_initial_;
    Stats<1> altitude_initial_;
    Stats<6> bias_initial_;
    Stats<3> pose_initial_;

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
        pose_initial_ = getStats(start_pose_);

        if (bias_.size() > 0)
        {
            bias_initial_ = getStats(bias_);
        }
        else
        {
            bias_initial_.mean.setZero();
            bias_initial_.cov = Mat<6,6>::Identity() * 0.20;
        }

        // world origin is offset by initial
        world_offset_.x() = pose_initial_.mean.x();
        world_offset_.y() = pose_initial_.mean.y();
        world_offset_.z() = -sonar_initial_.mean(0);

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
            && altitude_.size() >= 8
            && bias_.size() >= 0
            && start_pose_.size() >= 4;
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

    Vec<6> getBiasInitial() { return bias_initial_.mean; }
    Mat<6,6> getBiasInitialCov() { return bias_initial_.cov; }

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
        ekf_.use(orientation_);
        ekf_.use(bias_);

        ros::Duration(2.0).sleep(); // weired imu values at start

        sub_imu_ = nh_.subscribe("raw_imu", 1, &SensorFusion::imuCallback,this);
        sub_imu_bias_ = nh_.subscribe("raw_imu/bias", 1, &SensorFusion::imuBiasCallback,this);
        sub_magnetic_ = nh_.subscribe("magnetic", 1, &SensorFusion::magnetometerCallback, this);
        sub_gps_pos_ = nh_.subscribe("fix", 1, &SensorFusion::positionCallback, this);
        sub_gps_vel_ = nh_.subscribe("fix_velocity", 1, &SensorFusion::velocityCallback, this);
        sub_sonar_ = nh_.subscribe("sonar_height", 1, &SensorFusion::sonarCallback, this);
        sub_altimeter_ = nh_.subscribe("altimeter", 1, &SensorFusion::altimeterCallback, this);
        sub_initial_pose_ = nh_.subscribe("ground_truth_to_tf/pose", 1, &SensorFusion::initialPoseCallback, this);

        pub_marker_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1, true);
        pub_state_ = nh_.advertise<nav_msgs::Odometry>("state", 1, true);

        calibrate();
    }

    ~SensorFusion()
    {
        delete calibration_;
    }

    void calibrate()
    {
        ros::Time start_time = ros::Time::now();
        ROS_INFO("Calibrating...");

        do
        {

            ros::spinOnce();
            ros::Duration delta = ros::Time::now() - start_time;

            if (calibration_->isDone())
            {
                applyCalibration();
            }

            if (delta.toSec() > 5.0)
            {
                ROS_WARN("still calibrating... ");

                if (calibration_->magneto_.size() < 10)
                {
                    ROS_WARN("have not received magneto reading");
                }

                if (calibration_->ecef_.size() < 20)
                {
                    ROS_WARN("have not received ecef reading");
                }

                if (calibration_->vel_.size() < 5)
                {
                    ROS_WARN("have not received velocity reading");
                }

                if (calibration_->sonar_.size() < 8)
                {
                    ROS_WARN("have not received sonar reading");
                }

                if (calibration_->altitude_.size() < 8)
                {
                    ROS_WARN("have not received altitude reading");
                }

                if (calibration_->bias_.size() < 1)
                {
                    ROS_WARN("have not received bias reading");
                }

                if (calibration_->start_pose_.size() < 4)
                {
                    ROS_WARN("have not received initial pose reading");
                }

                start_time = ros::Time::now();
            } 
        }
        while (ros::ok() && !isCalibrated());
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

        XBias x_bias = ekf_.state_.getSubState<XBias>();
        x_bias.mean = calibration_->getBiasInitial();
        x_bias.cov = calibration_->getBiasInitialCov();

        ROS_INFO_STREAM("Initial position: \n " << calibration_->pos_initial_.mean << "\n with cov: \n" << calibration_->pos_initial_.cov);
        ROS_INFO_STREAM("Initial velocity: \n " << calibration_->vel_initial_.mean << "\n with cov: \n" << calibration_->vel_initial_.cov);

        ROS_INFO("Calibration done.");

        sum_accel_.setZero();
        calibrated_ = true;

    }

    void spin()
    {
        ros::Rate process_rate(100.0);
        while (ros::ok())
        {
            ros::spinOnce();

            process();

            publishState();
            publishMarker();

            process_rate.sleep();
        }
    }

    void process()
    {

        vector<double> s_t(8, std::numeric_limits<double>::infinity());

        // Order:
        // &accel_gravity_, &magneto_, &position_, &velocity_, 
        // &sonar_, &altimeter_, &orientation_, &bias_
        if (accel_gravity_.new_data) s_t[0] = accel_gravity_.stamp.toSec();
        if (magneto_.new_data) s_t[1] = magneto_.stamp.toSec();
        if (position_.new_data) s_t[2] = position_.stamp.toSec();
        if (velocity_.new_data) s_t[3] = velocity_.stamp.toSec();
        if (sonar_.new_data) s_t[4] = sonar_.stamp.toSec();
        if (altimeter_.new_data) s_t[5] = altimeter_.stamp.toSec();
        if (orientation_.new_data) s_t[6] = orientation_.stamp.toSec();
        if (bias_.new_data) s_t[7] = bias_.stamp.toSec();

        // forward EKF
        if (!ekf_.step(input))
        {
            ROS_INFO("skipping step");
            return ;
        }

        // get oldest data first
        int cur = distance(s_t.begin(), min_element(s_t.begin(), s_t.end()));

        while (s_t[cur] != std::numeric_limits<double>::infinity())
        {
            // correct ekf
            switch (cur)
            {
                case 0: ekf_.correct(accel_gravity_); break ; 
                case 1: ekf_.correct(magneto_); break ; 
                case 2: ekf_.correct(position_); break ; 
                case 3: ekf_.correct(velocity_); break ; 
                case 4: ekf_.correct(sonar_); break ; 
                case 5: ekf_.correct(altimeter_); break ; 
                case 6: ekf_.correct(orientation_); break ; 
                case 7: ekf_.correct(bias_); break ; 
            }

            // remove from data list
            s_t[cur] = std::numeric_limits<double>::infinity();

            // get next sensor reading
            cur = distance(s_t.begin(), min_element(s_t.begin(), s_t.end()));
        }

        ekf_.normalize();

    }

    Vec3 sum_accel_;

    void imuCallback(const sensor_msgs::Imu& msg)
    {
        // Get sensor data
        Vec3 linear_acceleration = Vec3(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
        Vec3 angular_velocity = Vec3(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
        
        Mat33 linear_acceleration_cov = I3 * 0.200;
        Mat33 angular_velocity_cov = I3 * 0.01;

        // Update input
        input.mean.setZero();
        input.cov.setZero();

        input.mean.segment<3>(0) = linear_acceleration;
        input.cov.block<3,3>(0,0) = linear_acceleration_cov;

        input.mean.segment<3>(3) = angular_velocity;
        input.cov.block<3,3>(3,3) = angular_velocity_cov;

        sum_accel_ += linear_acceleration;
        ++imu_counter_;

        // Correct gravity every 5th step
        if (imu_counter_ % 5 == 0)
        {
            accel_gravity_.update(
                sum_accel_ / (double)imu_counter_,
                4.0 * I3,
                msg.header.stamp);

            sum_accel_.setZero();
            imu_counter_ = 0;

            // Vec<4> imu_quat(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w); // x y z w
            // orientation_.update(imu_quat, Mat<4,4>::Identity() * 0.100, msg.header.stamp);
        }


    }

    void imuBiasCallback(const sensor_msgs::Imu& msg)
    {
        accel_bias_ = Vec3(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
        gyro_bias_ = Vec3(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);

        accel_gravity_.bias = accel_bias_;

        Vec<6> b;
        Mat<6,6> bcov;
        bcov.setZero();

        b.segment<3>(0) = accel_bias_;
        bcov.block<3,3>(0,0) = 0.20 * I3;

        b.segment<3>(3) = gyro_bias_;
        bcov.block<3,3>(3,3) = 0.05 * I3;

        if (!isCalibrated())
        {
            calibration_->bias_.push_back(b);
            return ;
        }

        // if (++bias_counter_ % 10 == 0)
        // {
        //     bias_.update(b, bcov, msg.header.stamp);
        // }
    }

    void magnetometerCallback(const geometry_msgs::Vector3Stamped& msg)
    {
        Vec3 field(msg.vector.x, msg.vector.y, msg.vector.z);
        Mat33 field_cov = 0.005 * I3;

        if (!isCalibrated())
        {
            calibration_->magneto_.push_back(field);
            return ;
        }

        magneto_.update(field, field_cov, msg.header.stamp);
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

        position_.update(xyz, xyz_cov, msg.header.stamp);
        // ROS_INFO_STREAM("xyz :: \n" << xyz);
    }

    void velocityCallback(const geometry_msgs::Vector3Stamped& msg)
    {
        Vec3 vel(msg.vector.x, msg.vector.y, msg.vector.z);
        Mat33 vel_cov = 0.05 * I3;

        if (!isCalibrated())
        {
            calibration_->vel_.push_back(vel);
            return ;
        }

        velocity_.update(vel, vel_cov, msg.header.stamp);
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

        sonar_.update(Vec1(range), Vec1(range_cov), msg.header.stamp);
    }

    void altimeterCallback(const hector_uav_msgs::Altimeter &msg)
    {
        const double altitude_cov = 5.0;
        double altitude = (double)msg.altitude;

        if (!isCalibrated())
        {
            calibration_->altitude_.push_back(Vec1(altitude));
            return ;
        }

        altitude -= calibration_->getAltimeterReference();
        altimeter_.update(Vec1(altitude), Vec1(altitude_cov), msg.header.stamp);
    }

    void initialPoseCallback(const geometry_msgs::PoseStamped &msg)
    {
        if (!isCalibrated())
        {
            calibration_->start_pose_.push_back(Vec3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z));
            return ;
        }

        sub_initial_pose_.shutdown();
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
    OrientationMeasurement orientation_;
    BiasMeasurement bias_;

    /** Subscribers */
    ros::Subscriber sub_imu_;
    ros::Subscriber sub_imu_bias_;
    ros::Subscriber sub_magnetic_;
    ros::Subscriber sub_gps_pos_;
    ros::Subscriber sub_gps_vel_;
    ros::Subscriber sub_sonar_;
    ros::Subscriber sub_altimeter_;
    ros::Subscriber sub_initial_pose_;

    /** Debug publishers */
    ros::Publisher pub_marker_;
    ros::Publisher pub_state_;

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

    void publishMarker()
    {
        const Vec3 pos = ekf_.position();
        const Quaternion quat = ekf_.orientation();

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
        qs.push_back(quat * Quaternion(1.0,0,0,0));                    // x
        qs.push_back(quat * Quaternion(1.0/M_SQRT2,0,0,1.0/M_SQRT2));  // y
        qs.push_back(quat * Quaternion(1.0/M_SQRT2,0,-1.0/M_SQRT2,0)); // z

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

    void publishState()
    {
        const Vec3 pos = ekf_.position();
        const Mat33 pos_cov = ekf_.positionCov();
        const Vec3 vel = ekf_.velocity();
        const Quaternion quat = ekf_.orientation();

        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "/drone_tf/world";

        odom.pose.pose.position.x = pos.x();
        odom.pose.pose.position.y = pos.y();
        odom.pose.pose.position.z = pos.z();

        Eigen::Map<Eigen::Matrix<double,6,6>> msg_cov(odom.pose.covariance.data());
        msg_cov.block<3,3>(0,0) = pos_cov;

        odom.pose.pose.orientation.x = quat.x();
        odom.pose.pose.orientation.y = quat.y();
        odom.pose.pose.orientation.z = quat.z();
        odom.pose.pose.orientation.w = quat.w();

        odom.twist.twist.linear.x = vel.x();
        odom.twist.twist.linear.y = vel.y();
        odom.twist.twist.linear.z = vel.z();

        pub_state_.publish(odom);
    }

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_fusion");

    SensorFusion node;
    node.spin();
}