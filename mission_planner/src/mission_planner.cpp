#include <mission_planner/mission_planner.h>

double MissionPlanner::calc_dist(geometry_msgs::Pose2D p1,geometry_msgs::Pose2D p2){
  return sqrt(std::pow(p1.x-p2.x,2)+std::pow(p1.y-p2.y,2));
}

MissionPlanner::MissionPlanner(ros::NodeHandle *nh, std::vector<geometry_msgs::Pose2D> goal_col){
  /*Initializes all parameters needed in mission planner
  Parameters:
      nh            Pointer to node handle
      goal_col      Collection of goal points
  */
  goals = goal_col;
  current_goal = 0;
  // initialize subscribers and publishers
  pos_sub = nh->subscribe("/robot_pose",1,&MissionPlanner::callback_pos,this);
  goal_pub = nh->advertise<geometry_msgs::Pose2D>("/navigation/goal", 5);
}

void MissionPlanner::callback_pos(const geometry_msgs::Pose2D& msg){
  /* Takes new position, determines distance to goal and updates the goal
  Parameters:
      msg     message with robot coordinates
  */
  if(calc_dist(msg,goals[current_goal])<THRESHOLD_DISTANCE){
    if(current_goal<(int)goals.size()-1){
      current_goal++;
    }
  }
  ROS_INFO("idx:%d",current_goal);
  ROS_INFO("size:%d",(int)goals.size());
  publish_goal();
}

void MissionPlanner::publish_goal(){
  /*Publishes the current goal point
  */
  goal_pub.publish(goals[current_goal]);
}
