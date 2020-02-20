#include <slam/odometry_mm.h>
#include <slam/make_map.h>



int main(int argc, char **argv){
  ros::init(argc,argv,"slam");
  ros::NodeHandle n;
  geometry_msgs::Pose2D pos_init;
  pos_init.x = 0;
  pos_init.y = 0;
  pos_init.theta = 0;
  OdometryMM motion_model(pos_init,0,0,&n);
  geometry_msgs::Pose2D min_pos,max_pos;
  min_pos.x = -2;
  min_pos.y = -3;
  max_pos.x = 5;
  max_pos.y = 4;
  LidarMap map(&n,min_pos,max_pos,0.1,0.2);
  ros::spin();
  return 0;
}
