#include <mission_planner/mission_planner.h>


// Main function of the slam package
int main(int argc, char **argv){
  // Init Ros
  ros::init(argc,argv,"mission_planner");
  ros::NodeHandle n;
  MissionPlanner planner(&n);
  ros::spin();
  return 0;
}
