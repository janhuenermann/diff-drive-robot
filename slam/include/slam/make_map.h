#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "ros/ros.h"
#include "ros/time.h"
#include <vector>
#include <set>
#include "math.h"

const int UNKNOWN = -1;
const int FREE = 0;
const int OCCUPIED = 1;
const int INFLATED = 2;

struct MapIdx{
  MapIdx(int x_pos,int y_pos);
  bool operator<(const MapIdx& rhs)const;
  int x;
  int y;
};

class LidarMap{
private:
  ros::Subscriber scan_sub;       // subscribe to wheel positions
  ros::Subscriber pos_sub;
  ros::Publisher grid_pub;        // map publisher
  float max_range = 3.5;
  float inflation_radius;
  float l_occ;
  float l_free;
  float L_THRESH = 5;
  geometry_msgs::Pose2D min_pos;
  float cell_size;
  int width;
  int height;
  geometry_msgs::Pose2D robot_pos;
  nav_msgs::OccupancyGrid occ_grid;
  std::vector<std::vector<float>> log_odds;
  std::vector<MapIdx> mask_inflation;
  std::vector<std::vector<std::set<MapIdx>>> inflation;

  geometry_msgs::Pose2D inverseSensorModel(geometry_msgs::Pose2D pos,float rng,float deg);
public:
  LidarMap(ros::NodeHandle *nh,geometry_msgs::Pose2D mi_pos,geometry_msgs::Pose2D max_pos, float c_size,float inf_radius);
  MapIdx pos2idx(geometry_msgs::Pose2D pos);
  geometry_msgs::Pose2D pos2idx_no_round(geometry_msgs::Pose2D pos);
  geometry_msgs::Pose2D idx2pos(MapIdx idx);
  void callback_scan(const sensor_msgs::LaserScan& msg);
  void callback_pos(const geometry_msgs::Pose2D& msg);
  void publish_grid();
  std::vector<MapIdx> line_of_sight(geometry_msgs::Pose2D end_pos);
  std::vector<MapIdx> gen_mask();
  bool idx_in_map(MapIdx idx);
  void update_at_idx(MapIdx idx, bool occupied);
  void add_inflation(MapIdx center,bool occupied);
};
