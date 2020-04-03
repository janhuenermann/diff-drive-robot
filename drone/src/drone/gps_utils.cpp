#include "drone/gps_utils.hpp"

GPSUtils::GPSUtils(geometry_msgs::Point initial_pos,double initial_heading){
  //Save initial position and heading to translate/rotate the system accordingly
  initial_map << initial_pos.x,initial_pos.y,initial_pos.z;
  initial_compass = initial_heading;
  ros::NodeHandle nh;
  // Subscribe to gps velocity and position
  sub_gps_pos = nh.subscribe("/drone/fix",1,&GPSUtils::posCallback,this);
  sub_gps_vel = nh.subscribe("/drone/fix_velocity",1,&GPSUtils::velCallback,this);
}

geometry_msgs::Vector3 GPSUtils::get_var_pos(){
  return var_pos;
}
geometry_msgs::Vector3 GPSUtils::get_var_vel(){
  return var_vel;
}

void GPSUtils::posCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
  // First NR_INIT_MEASURMENTS steps are used for initializing
  if(init_counter_pos==0) initial_ecef << 0,0,0;
  // Find average ecef position to set origin on earth surface
  if(init_counter_pos<NR_INIT_MEASURMENTS){
    init_counter_pos++;
    Eigen::Vector3d ecef_curr = gps2ecef(*msg);
    initial_ecef += (1.0/NR_INIT_MEASURMENTS)*ecef_curr;
    init_pos_collection.push_back(*msg);
    // Calculate variance after gathering the data
    geometry_msgs::Point curr_point;
    if(init_counter_pos == NR_INIT_MEASURMENTS){
      init_counter_pos++;
      Eigen::ArrayXf currx(init_pos_collection.size());
      Eigen::ArrayXf curry(init_pos_collection.size());
      Eigen::ArrayXf currz(init_pos_collection.size());
      for(int i=0;i<init_pos_collection.size();i++){
        curr_point = convert_measurement(init_pos_collection[i]);
        currx(i) = curr_point.x;
        curry(i) = curr_point.y;
        currz(i) = curr_point.z;
        ROS_INFO("x:%f,y:%f,z:%f",curr_point.x,curr_point.y,curr_point.z);
      }
      var_pos.x = (currx - currx.mean()).square().sum()/(NR_INIT_MEASURMENTS);
      var_pos.y = (curry - curry.mean()).square().sum()/(NR_INIT_MEASURMENTS);
      var_pos.z = (currz - currz.mean()).square().sum()/(NR_INIT_MEASURMENTS);
    }
  }
  // If inited calculate the position in map coordinate frame
  if(init_counter_pos>NR_INIT_MEASURMENTS){
    current_pos = convert_measurement(*msg);
  }
}
void GPSUtils::velCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
  // Save velocity
  current_vel = msg->vector;
  if(init_counter_vel<NR_INIT_MEASURMENTS){
    init_counter_vel++;
    init_vel_collection.push_back(msg->vector);
  }
  // Find initial vairance in measurements
  if(init_counter_vel==NR_INIT_MEASURMENTS){
    init_counter_vel++;
    Eigen::ArrayXf currx(init_vel_collection.size());
    Eigen::ArrayXf curry(init_vel_collection.size());
    Eigen::ArrayXf currz(init_vel_collection.size());
    for(int i=0;i<init_vel_collection.size();i++){
      currx(i) = (init_vel_collection[i]).x;
      curry(i) = (init_vel_collection[i]).y;
      currz(i) = (init_vel_collection[i]).z;
    }
    var_vel.x = (currx - currx.mean()).square().sum()/(currx.size()-1);
    var_vel.y = (curry - curry.mean()).square().sum()/(curry.size()-1);
    var_vel.z = (currz - currz.mean()).square().sum()/(currz.size()-1);
  }
}

geometry_msgs::Point GPSUtils::convert_measurement(sensor_msgs::NavSatFix msg){
  // Convert gps message to map coordinate system
  double longitude = msg.longitude*M_PI/180.0;
  double latitude = msg.latitude*M_PI/180.0;
  double altitude = msg.altitude;
  Eigen::Vector3d curr_ecef = gps2ecef(msg);
  Eigen::Vector3d ned;
  Eigen::Matrix3d Ren;
  Ren << -sin(latitude)*cos(longitude), -sin(longitude), -cos(latitude)*cos(longitude),
  -sin(latitude)*sin(longitude), cos(longitude), -cos(latitude)*sin(longitude),
  cos(latitude),0 , -sin(latitude);

  ned = Ren.transpose()*(curr_ecef-initial_ecef);

  Eigen::Matrix3d Rmn;
  Rmn << cos(initial_compass),sin(initial_compass),0,
    sin(initial_compass),-cos(initial_compass),0,
    0,0,-1;

  Eigen::Vector3d map;

  map = Rmn*(ned-initial_map);
  geometry_msgs::Point return_point;
  return_point.x = map(0);
  return_point.y = map(1);
  return_point.z = map(2);
  return return_point;
}

bool GPSUtils::initialized(){
  return init_counter_pos > NR_INIT_MEASURMENTS && init_counter_vel > NR_INIT_MEASURMENTS;
}

geometry_msgs::Point GPSUtils::get_pos(){
  return current_pos;
}

geometry_msgs::Vector3 GPSUtils::get_vel(){
  return current_vel;
}

Eigen::Vector3d GPSUtils::gps2ecef(sensor_msgs::NavSatFix msg){
  // Convert gps messages to ecef system
  double longitude = msg.longitude*M_PI/180.0;
  double latitude = msg.latitude*M_PI/180.0;
  double altitude = msg.altitude;

  double a = 6378137;
  double b = 6356752;
  double e_sq = 1-b*b/(a*a);
  Eigen::Vector3d ecef;
  double N = a/sqrt(1-e_sq*pow(sin(latitude),2));
  ecef << (N+altitude)*cos(latitude)*cos(longitude),
    (N+altitude)*cos(latitude)*sin(longitude),
    (N*b*b/(a*a)+altitude)*sin(latitude);
  return ecef;
}
