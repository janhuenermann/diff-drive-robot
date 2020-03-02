#include "ros/ros.h"
#include <local_planner/spline.hpp>

class LocalPlannerNode
{

};

// Main function of the slam package
int main(int argc, char **argv){
  // Init Ros
  ros::init(argc,argv, "local_planner_node");
  ros::NodeHandle n;

  ros::spin();
  return 0;
}
