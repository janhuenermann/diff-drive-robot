#include "geometry_msgs/Pose2D.h"
#include "ros/ros.h"
#include "ros/time.h"
#include "math.h"
#include <vector>

class MissionPlanner{
private:
  std::vector<geometry_msgs::Pose2D> goals;
  ros::Subscriber pos_sub;
  ros::Publisher goal_pub;
  int current_goal;
  const double THRESHOLD_DISTANCE = 0.1;
public:
  MissionPlanner(ros::NodeHandle *nh);
  double calc_dist(geometry_msgs::Pose2D p1,geometry_msgs::Pose2D p2);
  void callback_pos(const geometry_msgs::Pose2D& msg);
  void publish_goal();
};
