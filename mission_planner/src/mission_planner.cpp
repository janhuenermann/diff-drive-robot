#include <mission_planner/mission_planner.h>

double MissionPlanner::calc_dist(geometry_msgs::Pose2D p1,geometry_msgs::Pose2D p2){
  return sqrt(std::pow(p1.x-p2.x,2)+std::pow(p1.y-p2.y,2));
}

MissionPlanner::MissionPlanner(ros::NodeHandle *nh){
  std::vector<double> goals_x;
  std::vector<double> goals_y;
  /*nh->getParam ("/GOALS_X", goals_x);
  nh->getParam ("/GOALS_Y", goals_y);
  for (int i=0;i<goals_x.size();i++){
    goals[i].x = goals_x[i];
    goals[i].y = goals_y[i];
  }*/
  geometry_msgs::Pose2D p1;
  p1.x = 1;
  p1.y = 1;
  geometry_msgs::Pose2D p2;
  p2.x = 3;
  p2.y = 2;
  goals.push_back(p1);
  goals.push_back(p2);
  current_goal = 0;
  pos_sub = nh->subscribe("robot_pose",1,&MissionPlanner::callback_pos,this);
  goal_pub = nh->advertise<geometry_msgs::Pose2D>("/navigation/goal", 5);
}

void MissionPlanner::callback_pos(const geometry_msgs::Pose2D& msg){
  //ROS_INFO("dist: %f",calc_dist(msg,goals[current_goal]));
  if(calc_dist(msg,goals[current_goal])<THRESHOLD_DISTANCE){
    if(current_goal<goals.size()){
      current_goal++;
    }
  }
  publish_goal();
}

void MissionPlanner::publish_goal(){
  goal_pub.publish(goals[current_goal]);
}
