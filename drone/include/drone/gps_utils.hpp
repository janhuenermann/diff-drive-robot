#include "ros/ros.h"
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <Eigen/Dense>
#include <vector>

class GPSUtils{
public:
  GPSUtils(geometry_msgs::Point initial_pos,double initial_heading);

  geometry_msgs::Point convert_measurement(sensor_msgs::NavSatFix msg);
  void posCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
  void velCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);

  geometry_msgs::Point get_pos();
  geometry_msgs::Vector3 get_vel();
  geometry_msgs::Vector3 get_var_pos();
  geometry_msgs::Vector3 get_var_vel();

  bool initialized();
private:
  Eigen::Vector3d gps2ecef(sensor_msgs::NavSatFix msg);

  Eigen::Vector3d initial_map;
  Eigen::Vector3d initial_ecef;
  std::vector<sensor_msgs::NavSatFix> init_pos_collection;
  std::vector<geometry_msgs::Vector3> init_vel_collection;
  geometry_msgs::Vector3 var_pos;
  geometry_msgs::Vector3 var_vel;
  geometry_msgs::Point current_pos;
  geometry_msgs::Vector3 current_vel;
  int init_counter_pos = 0;
  int init_counter_vel = 0;
  const int NR_INIT_MEASURMENTS = 20;
  double initial_compass;

  ros::Subscriber sub_gps_pos;
  ros::Subscriber sub_gps_vel;
};
