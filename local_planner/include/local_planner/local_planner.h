#include "geometry_msgs/Pose2D.h"
#include "ros/ros.h"
#include "ros/time.h"

struct Spline{
  double a[4];
  double b[4];
  const int TF = 1;
  void calculate(geometry_msgs::Pose2D start,geometry_msgs::Pose2D dstart,geometry_msgs::Pose2D end,geometry_msgs::Pose2D dend);
  geometry_msgs::Pose2D get_pos(double t);
  geometry_msgs::Pose2D get_derivative(double t);
};

class LocalPlanner{
  
};
