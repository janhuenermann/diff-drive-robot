#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Imu.h"
#include "ros/ros.h"
#include "ros/time.h"
#include <tf/transform_broadcaster.h>


class OdometryMM{
private:
  const float WEIGHT_IMU = 0.05f;       // weights for weighted sensor fusion
  const float WEIGHT_ODOMETRY = 0.95f;
  const float WEIGHT_COMPASS = 0.3f;
  geometry_msgs::Pose2D pos;    // position in world frame
  ros::Subscriber wheel_sub;    // subscribe to wheel positions
  ros::Subscriber imu_sub;    // subscribe to imu
  ros::Publisher pos_pub;       // position publisher
  ros::Publisher vel_pub;
  float wl;                     // angle of left wheel in rad
  float wr;                     // angle of right wheel in rad
  float L = 0.16;               // Axle axle track
  float WR = 0.066/2;           // Wheel radius
  double t;                      // current ros time in seconds
  double a;                         // current acceleration
  double v;                         // current speed
  double compass;                 // angle output of compas
  double w ;                      // angular velocity

public:
  OdometryMM(geometry_msgs::Pose2D pos_init,float wl_init ,float wr_init, ros::NodeHandle *nh);
  void callback_wheels(const sensor_msgs::JointState& msg);
  void callback_imu(const sensor_msgs::Imu& msg);
  void publish_pos();
  void publish_vel(float wz,float vx,float vy);
};
