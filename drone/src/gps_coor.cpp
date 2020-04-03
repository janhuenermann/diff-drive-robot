#include "drone/gps_utils.hpp"
#include <tf/transform_broadcaster.h>


int main(int argc, char **argv){
  // Init Ros
  ros::init(argc,argv, "gps_test_node");
  ROS_INFO("gps test node started");
  geometry_msgs::Point initial_map;
  initial_map.x = 0;
  initial_map.y = 5;
  initial_map.z = 0;
  double initial_compass = 0;
  GPSUtils mygps(initial_map,initial_compass);
  ros::Rate r(100);
  while (ros::ok()){
    ros::spinOnce();                   // Handle ROS events
    if(mygps.initialized()) break;
    r.sleep();
  }
  ROS_INFO("gps inited");
  geometry_msgs::Vector3 vpos,vvel;
  vpos = mygps.get_var_pos();
  vvel = mygps.get_var_vel();
  ROS_INFO("v_pos x=%f, y=%f, z=%f",vpos.x,vpos.y,vpos.z);
  ROS_INFO("v_vel x=%f, y=%f, z=%f",vvel.x,vvel.y,vvel.z);
  ros::spin();
  return 0;
}
