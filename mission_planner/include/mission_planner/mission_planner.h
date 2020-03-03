#include "geometry_msgs/Pose2D.h"
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "ros/time.h"
#include "math.h"
#include <vector>
#include <string>

class MissionPlanner{
private:
  std::vector<geometry_msgs::Pose2D> goals;     //contains coordinates of all goles
  ros::Subscriber pos_sub;
  ros::Publisher goal_pub;
  int current_goal;                             // index of current goal
  const double THRESHOLD_DISTANCE = 0.1;        // min distance to goal to be considered reached
public:
  MissionPlanner(ros::NodeHandle *nh,std::vector<geometry_msgs::Pose2D> goal_col);
  double calc_dist(geometry_msgs::Pose2D p1,geometry_msgs::Pose2D p2);
  void callback_pos(const geometry_msgs::Pose2D& msg);
  void publish_goal();
};
