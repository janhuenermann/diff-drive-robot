#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Imu.h"
#include "ros/ros.h"
#include "ros/time.h"
#include <tf/transform_broadcaster.h>
#include <math/util.hpp>


class OdometryMM{
private:
  const double WEIGHT_ODOMETRY = 1.0;
  const double WEIGHT_COMPASS = 0.0;

  geometry_msgs::Pose2D pos;    // position in world frame
  ros::Subscriber wheel_sub;    // subscribe to wheel positions
  ros::Subscriber imu_sub;    // subscribe to imu
  ros::Publisher pos_pub;       // position publisher
  ros::Publisher vel_pub;
  double wl;                     // angle of left wheel in rad
  double wr;                     // angle of right wheel in rad
  double L = 0.16;               // Axle axle track
  double WR = 0.066/2;           // Wheel radius
  ros::Time t;                      // current ros time in seconds
  double a;                         // current acceleration
  double v;                         // current speed
  double compass;                 // angle output of compas
  double w ;                      // angular velocity
  Profiler profiler;

public:
  OdometryMM(geometry_msgs::Pose2D pos_init, double wl_init, double wr_init, ros::NodeHandle *nh);
  void callback_wheels(const sensor_msgs::JointState& msg);
  void callback_imu(const sensor_msgs::Imu& msg);
  void publish_pos();
  void publish_vel(double wz, double v);
};
