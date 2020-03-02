#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Imu.h"
#include "ros/ros.h"
#include "ros/time.h"
#include <tf/transform_broadcaster.h>

#define WEIGHT_COMPAS 0.5;
#define WEIGHT_IMU 0.2;
#define WEIGHT_ODOMETRY 0.8;

class OdometryMM{
private:
  geometry_msgs::Pose2D pos;    // position in world frame
  geometry_msgs::Pose2D pos_imu;    // position in world frame
  geometry_msgs::Pose2D pos_od;    // position in world frame
  ros::Subscriber wheel_sub;    // subscribe to wheel positions
  ros::Publisher pos_pub;       // position publisher
  float wl;                     // angle of left wheel in rad
  float wr;                     // angle of right wheel in rad
  float L = 0.16;               // Axle axle track
  float WR = 0.066/2;           // Wheel radius
  double t_od;                      // current ros time in seconds updated for odometry
  double t_imu;                     // current ros time in seconds updated for imu
  double v;                         // current speed

public:
  OdometryMM(geometry_msgs::Pose2D pos_init,float wl_init ,float wr_init, ros::NodeHandle *nh);
  void callback_wheels(const sensor_msgs::JointState& msg);
  void callback_imu(const sensor_msgs::Imu& msg);
  void publish_pos();
};
