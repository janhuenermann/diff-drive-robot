#include <slam/odometry_mm.h>


int main(int argc, char **argv){
  ros::init(argc,argv,"slam");
  ros::NodeHandle n;
  geometry_msgs::Pose2D pos_init;
  pos_init.x = 0;
  pos_init.y = 0;
  pos_init.theta = 0;
  OdometryMM motion_model(pos_init,0,0,&n);
  ROS_INFO("Slam Started");
  ros::spin();
  return 0;
}
