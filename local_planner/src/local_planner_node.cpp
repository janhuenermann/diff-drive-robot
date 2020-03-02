#include <local_planner/local_planner.h>


// Main function of the slam package
int main(int argc, char **argv){
  // Init Ros
  ros::init(argc,argv,"slam");
  ros::NodeHandle n;

  ros::spin();
  return 0;
}
