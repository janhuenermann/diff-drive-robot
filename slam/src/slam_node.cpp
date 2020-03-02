#include <slam/odometry_mm.h>
#include <slam/make_map.h>


// Main function of the slam package
int main(int argc, char **argv){
  // Init Ros
  ros::init(argc,argv,"slam");
  ros::NodeHandle n;
  // Set initial position to (0,0) and initialize motion model for position estimation
  geometry_msgs::Pose2D pos_init;
  pos_init.x = 0;
  pos_init.y = 0;
  pos_init.theta = 0;
  OdometryMM motion_model(pos_init,0,0,&n);
  // Declare the minimal and maximal position to consider for the map and create object for generating map and read LIDAR
  geometry_msgs::Pose2D min_pos,max_pos;
  min_pos.x = -2;
  min_pos.y = -3;
  max_pos.x = 5;
  max_pos.y = 4;
  float cell_size = 0.1;
  float inflation_rad = 0.2;
  LidarMap map(&n,min_pos,max_pos,cell_size,inflation_rad);
  //Let objects take care of received messages
  ros::spin();
  return 0;
}
