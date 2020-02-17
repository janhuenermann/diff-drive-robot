#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/JointState.h"
#include "ros/ros.h"
#include "ros/time.h"

class OdometryMM{
private:
  geometry_msgs::Pose2D pos;    // position in world frame
  ros::Subscriber wheel_sub;    // subscribe to wheel positions
  ros::Publisher pos_pub;       // position publisher
  float wl;                     // angle of left wheel in rad
  float wr;                     // angle of right wheel in rad
  float L = 0.16;               // Axle axle track
  float WR = 0.066/2;           // Wheel radius
  double t;                     // current ros time in seconds

public:
  OdometryMM(geometry_msgs::Pose2D pos_init,float wl_init ,float wr_init, ros::NodeHandle *nh);
  void callback_wheels(const sensor_msgs::JointState& msg);
  void publish_pos();
};
