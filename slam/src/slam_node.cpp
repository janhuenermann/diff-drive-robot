#include <slam/odometry_mm.h>
#include <slam/make_map.h>
#include <string>


// Main function of the slam package
int main(int argc, char **argv){
  // Init Ros
  ros::init(argc,argv,"slam");
  ros::NodeHandle n("~");
  //Parse arguments given at startup
  std::string start_pos_s;
  std::string min_pos_s;
  std::string max_pos_s;
  std::string cell_size_s;
  while(!n.getParam ("start_pos", start_pos_s)){
    ROS_INFO("waiting for start pos");
  }
  while(!n.getParam ("min_pos", min_pos_s)){
    ROS_INFO("waiting for min pos");
  }
  while(!n.getParam ("max_pos", max_pos_s)){
    ROS_INFO("waiting for max pos");
  }
  while(!n.getParam ("cell_size", cell_size_s)){
    ROS_INFO("waiting for cell size");
  }
  // Set initial position to (0,0) and initialize motion model for position estimation
  geometry_msgs::Pose2D pos_init;
  int pos = start_pos_s.find(',');
  pos_init.x = std::stof(start_pos_s.substr(0, pos));
  start_pos_s.erase(0, pos + 1);
  pos_init.y =  std::stof(start_pos_s);
  // start motion model
  OdometryMM motion_model(pos_init,0,0,&n);
  // Declare the minimal and maximal position to consider for the map and create object for generating map and read LIDAR
  geometry_msgs::Pose2D min_pos,max_pos;
  pos = min_pos_s.find(',');
  min_pos.x = std::stof(min_pos_s.substr(0, pos));
  min_pos_s.erase(0, pos + 1);
  min_pos.y =  std::stof(min_pos_s);

  pos = max_pos_s.find(',');
  max_pos.x = std::stof(max_pos_s.substr(0, pos));
  max_pos_s.erase(0, pos + 1);
  max_pos.y = std::stof(max_pos_s);


  float cell_size = std::stof(cell_size_s);
  float inflation_rad = 0.2;
  // start map
  LidarMap map(&n,min_pos,max_pos,cell_size,inflation_rad);
  ROS_INFO("Booted slam node");
  //Let objects take care of received messages
  ros::spin();
  return 0;
}
