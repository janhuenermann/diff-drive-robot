#include <mission_planner/mission_planner.h>


// Main function of the slam package
int main(int argc, char **argv){
  // Init Ros
  ros::init(argc,argv,"mission_planner");
  ros::NodeHandle n("~");
  std::string goals;
  while(!n.getParam ("goals", goals)){
    ROS_INFO("waiting for goals");
  }
  // parse given arguments to get goal points
  std::vector<geometry_msgs::Pose2D> goal_col;
  geometry_msgs::Pose2D current_pos;
  char curr_char;
  bool before_dec = true;

  int pos = goals.find('|');
  std::string curr,x,y;

  while (pos != std::string::npos) {
      pos = goals.find('|');
      curr = goals.substr(0, pos);
      int pos_c = curr.find(',');
      x = goals.substr(0, pos_c);
      curr.erase(0, pos_c + 1);
      y = curr;
      current_pos.y = std::stof(y);
      current_pos.x = std::stof(x);
      goal_col.push_back(current_pos);
      goals.erase(0, pos + 1);
  }

  // Start mission planner
  MissionPlanner planner(goal_col);
  ROS_INFO("Booted MissionPlanner node");
  ros::spin();
  return 0;
}
