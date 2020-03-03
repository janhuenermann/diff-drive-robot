#include <slam/make_map.h>

MapIdx::MapIdx(int x_pos,int y_pos){
  /* Constructor for Map Indices
  Parameters:
      position (two integers)   The two indizes in map.
  */
  x = x_pos;
  y = y_pos;
}

bool MapIdx::operator<(const MapIdx& rhs) const{
  /* Comparator for Map Indices (needed to insert into set)
  Parameters:
      rhs (MapIdx)   The value to compare to
  */
  // Sort by x first and then by y
  if(x<rhs.x) return true;
  if(x==rhs.x && y<rhs.y) return true;
  return false;
}

LidarMap::LidarMap(ros::NodeHandle *nh,geometry_msgs::Pose2D mi_pos,geometry_msgs::Pose2D max_pos,float c_size,float inf_radius):
  /* Constructor for map and lidar
  Parameters:
      *nh               Node handle required to make subscriptions
      mi_pos            Minimal position captured on map
      max_pos           Maximal position captured on map
      c_size            Size of a cell
      inf_radius        Radius added to obsticles, such that robot can pass
  */
  inflation_radius(inf_radius),
  min_pos(mi_pos),
  cell_size(c_size)
  {
    // Init subscribers and publishers
    scan_sub=nh->subscribe("scan",10,&LidarMap::callback_scan,this);
    pos_sub = nh->subscribe("robot_pose",1,&LidarMap::callback_pos,this);
    grid_pub = nh->advertise<nav_msgs::OccupancyGrid>("/map", 5);
    // Initialize occupancy grid metadata
    occ_grid.info.resolution = cell_size;
    occ_grid.info.origin.position.x = min_pos.x;
    occ_grid.info.origin.position.y = min_pos.y;
    width = ceil((max_pos.x-min_pos.x)/cell_size);
    height = ceil((max_pos.y-min_pos.y)/cell_size);
    occ_grid.info.width = width;
    occ_grid.info.height = height;
    occ_grid.data.resize(width*height);
    std::fill(occ_grid.data.begin(),occ_grid.data.end(),UNKNOWN);
    // Generate relative coordinates to inflate
    mask_inflation = gen_mask();
    // Init log odds
    l_occ = 0.1;
    l_free = -0.1;
    log_odds.resize(width, std::vector<float>(height));
    std::fill(log_odds.begin(), log_odds.end(), std::vector<float>(height, 0));
    // Stores all occupied cells causing an inflation
    inflation.resize(width, std::vector<std::set<MapIdx>>(height));
}

void LidarMap::callback_scan(const sensor_msgs::LaserScan& msg){
  /* Gets information from new LIDAR scan
  Parameters:
      msg       Output of laser scan
  */
 std::vector<float> ranges = msg.ranges;
 // Scan is 360 degrees
 for(int i=0;i<360;i++){
   MapIdx end_idx(0,0);
   geometry_msgs::Pose2D end_pos;

   bool obstacle_found = ranges[i] != INFINITY;
   float range = obstacle_found ? ranges[i] : max_range;

   end_pos = inverseSensorModel(robot_pos,range,i);
   end_idx = pos2idx(end_pos);

   // find free cells between robot position and scan endpoint
   std::vector<MapIdx> los_idx = line_of_sight(end_pos);
   for(int i=0;i<los_idx.size();i++){
    if (los_idx[i].x == end_idx.x && los_idx[i].y == end_idx.y)
    {
      break ;
    }

    if(idx_in_map(los_idx[i])){
      update_at_idx(los_idx[i], false);
    }
   }

   if(obstacle_found && idx_in_map(end_idx)){
    update_at_idx(end_idx, true);
   }
 }
 // publish the new grid
 publish_grid();
}

void LidarMap::callback_pos(const geometry_msgs::Pose2D& msg){
  /* Saves the new robot position
  Parameters:
      msg       Estimated position
  */
  robot_pos = msg;
}

geometry_msgs::Pose2D LidarMap::idx2pos(MapIdx idx){
  /*Converts map indices to world coordinates
  Parameters:
      pos     Index in map
  Returns:
      Position of the cell in world coordinates
  */
  geometry_msgs::Pose2D res;
  res.x = idx.x * cell_size + min_pos.x;
  res.y = idx.y * cell_size + min_pos.y;
  return res;
}

MapIdx LidarMap::pos2idx(geometry_msgs::Pose2D pos){
  /*Converts World coordinates to map indices
  Parameters:
      pos (World coordinate in form (x,y))
  Returns:
      Index in map
  */
  int x = round((pos.x - min_pos.x)/cell_size);
  int y = round((pos.y - min_pos.y)/cell_size);
  MapIdx res(x,y);
  return res;
}

geometry_msgs::Pose2D LidarMap::pos2idx_no_round(geometry_msgs::Pose2D pos){
  /*Converts World coordinates to map indices but does not round for integer indices
  Parameters:
      pos (World coordinate in form (x,y))
  Returns:
      Index in map
  */
  geometry_msgs::Pose2D res;
  res.x = (pos.x - min_pos.x)/cell_size;
  res.y = (pos.y - min_pos.y)/cell_size;
  return res;
}

geometry_msgs::Pose2D LidarMap::inverseSensorModel(geometry_msgs::Pose2D pos,float rng,float deg){
    /* Takes output of lidar and gives world position of the end of the laser ray
    Parameters:
        pos       Position of robot in world coordinates
        rng       Distance to obstacle, or max range if no obstacle
        deg       Angle of laser measurement
    Returns:
        Position of the end point of measurement in world coordinates
    */
    float rad = (M_PI/180)*deg;
    geometry_msgs::Pose2D res;
    res.x = pos.x + rng*cos(pos.theta+rad);
    res.y = pos.y + rng*sin(pos.theta+rad);
    return res;
}

std::vector<MapIdx> LidarMap::gen_mask(){
  /* Generates mask of relative map indices, that have to be considered for inflation
  Returns:
      List of Map indices that have to be considered
  */
  std::vector<MapIdx> res;
  MapIdx curr(0,0);
  grid_pub.publish(occ_grid);
  int x_max = round(inflation_radius/cell_size); // max idx
  float r_idx = inflation_radius/cell_size;
  // Only look at first quadrant and then use symmetry for the rest, while not addiding twice
  for(int x=0;x<x_max;x++){
    for(int y=0;y<x_max;y++){
      if(x==0 && y==0) continue;
      if(sqrt(x*x+y*y)<r_idx){
        curr.x = x;
        curr.y = y;
        res.push_back(curr);
        if(x>0){
          curr.x = -x;
          curr.y = y;
          res.push_back(curr);
        }
        if(y>0){
          curr.x = x;
          curr.y = -y;
        res.push_back(curr);
        }
        if(x>0&&y>0){
          curr.x = -x;
          curr.y = -y;
          res.push_back(curr);
        }
      }
    }
  }
  return res;
}

void LidarMap::publish_grid(){
  /* Publishes the calculated Occupancy grid
  */
  grid_pub.publish(occ_grid);
}

std::vector<MapIdx> LidarMap::line_of_sight(geometry_msgs::Pose2D end_pos){
  /* Uses general line algorithm to compute the indices between position of robot and an end position
  Parameters:
      end_pos the end position of the line of sight
  Returns:
      vector of map indices, in line of sight between robot and end_pos
  */
  std::vector<MapIdx> res;
  MapIdx start_idx_rounded = pos2idx(robot_pos);
  MapIdx end_idx_rounded = pos2idx(end_pos);
  geometry_msgs::Pose2D start_idx = pos2idx_no_round(robot_pos);
  geometry_msgs::Pose2D end_idx = pos2idx_no_round(end_pos);
  // append start
  res.push_back(start_idx_rounded);
  float di = end_idx.x-start_idx.x;
  float dj = end_idx.y-start_idx.y;
  float dl,ds;
  bool swap;
  // find long and short side
  if(abs(di)>abs(dj)){
    dl = di;
    ds = dj;
    swap = false;
  }else{
    dl = dj;
    ds = di;
    swap = true;
  }
  // Now all x entries are the long axis and y indices are the short axis
  float l_s,s_s,l_e,s_e;
  int l_sr,s_sr,l_er,s_er;
  if(swap){
    l_s = start_idx.y; s_s = start_idx.x;
    l_e = end_idx.y; s_e = end_idx.x;
    l_sr = start_idx_rounded.y; s_sr = start_idx_rounded.x;
    l_er = end_idx_rounded.y; s_er = end_idx_rounded.x;
  }else{
    l_s = start_idx.x; s_s = start_idx.y;
    l_e = end_idx.x; s_e = end_idx.y;
    l_sr = start_idx_rounded.x; s_sr = start_idx_rounded.y;
    l_er = end_idx_rounded.x; s_er = end_idx_rounded.y;
  }
  // increments
  int sign_ds = copysign(1,ds);
  int sign_dl = copysign(1,dl);
  float phi = ds/abs(dl);
  //error
  float es = s_s-s_sr;
  float l = abs(ds/dl)*(0.5+(l_s-l_sr)*sign_dl)-0.5;
  // calculate the indices in line of sight and add to vector
  while((l_sr!=l_er)||(s_sr!=s_er)){
    l_sr += sign_dl;
    es += phi;
    if((ds>=0 && es>=0.5)||(ds<0 && es<-0.5)){
      es -= sign_ds;
      s_sr += sign_ds;
      float l_dash = es*sign_ds;
      if(l_dash<l){
        MapIdx curr(l_sr,s_sr-sign_ds);
        if(swap){
          curr.x = s_sr-sign_ds; curr.y = l_sr;
        }
        res.push_back(curr);
        if ((l_sr==l_er)&&(s_sr-sign_ds==s_er)) break;
      }
      if(l_dash > l){
        MapIdx curr(l_sr-sign_dl,s_sr);
        if(swap){
          curr.x = s_sr; curr.y = l_sr-sign_dl;
        }
        res.push_back(curr);
        if ((l_sr-sign_dl==l_er)&&(s_sr==s_er)) break;
      }
      if(l_dash==l){
        MapIdx curr1(l_sr-sign_dl,s_sr);
        MapIdx curr2(l_sr,s_sr-sign_ds);
        if(swap){
          curr1.x = s_sr; curr1.y = l_sr-sign_dl;
          curr2.x = s_sr-sign_ds; curr2.y = l_sr;
        }
        res.push_back(curr1);
        if ((l_sr-sign_dl==l_er)&&(s_sr==s_er)) break;
        res.push_back(curr2);
        if ((l_sr==l_er)&&(s_sr-sign_ds==s_er)) break;
      }
    }
    if((l_sr!=l_er)||(s_sr!=s_er)){
      MapIdx curr(l_sr,s_sr);
      if(swap){
        curr.x = s_sr; curr.y = l_sr;
      }
      res.push_back(curr);
    }
  }
  return res;
}

bool LidarMap::idx_in_map(MapIdx idx){
  /* Checks if an index is inside the bounds of the map
  Parameters:
      idx       Map index to be checked
  Returns:
      true if index is in bounds, false otherwise
  */
  if(idx.x<occ_grid.info.width && idx.y<occ_grid.info.height){
    return true;
  }
  return false;
}

void LidarMap::update_at_idx(MapIdx idx, bool occupied){
  /* Update log odds and occupancy grid with new found information
  Parameters:
      idx           index to update
      occupied      true if the cell is beliefed to be occupide, false else
  */
  if(occupied){
    log_odds[idx.x][idx.y] = std::min(l_occ+log_odds[idx.x][idx.y],L_max);
    if(log_odds[idx.x][idx.y]>=L_THRESH){
      occ_grid.data[idx.y*width+idx.x] = OCCUPIED;
      add_inflation(idx,true);
    }
  }else{
    log_odds[idx.x][idx.y] = std::max(l_free+log_odds[idx.x][idx.y],-L_max);
    if(log_odds[idx.x][idx.y]<=-L_THRESH){
      add_inflation(idx,false);
      if(occ_grid.data[idx.y*width+idx.x] != INFLATED){
        occ_grid.data[idx.y*width+idx.x] = FREE;
      }
    }
  }
  if(std::abs(log_odds[idx.x][idx.y])<L_THRESH){
    occ_grid.data[idx.y*width+idx.x] = UNKNOWN;
    add_inflation(idx,false);
  }
}

// occupied is the new occupation status
void LidarMap::add_inflation(MapIdx center,bool occupied){
  /* Marks cells to be inflated, when a change happens at center
  Parameters:
      center        The changed cell
      occupied      true if the cell is newly occupied, false if cell is newly free
  */
  MapIdx idx(0,0);
  for(int i=0;i<mask_inflation.size();i++){
    idx.x = center.x+mask_inflation[i].x;
    idx.y = center.y+mask_inflation[i].y;
    if(idx_in_map(idx)){
      if(occupied){
        inflation[idx.x][idx.y].insert(center);
        if(occ_grid.data[width*idx.y+idx.x]==FREE){
          occ_grid.data[width*idx.y+idx.x] = INFLATED;
        }
      }else{
        inflation[idx.x][idx.y].erase(center);
        if(occ_grid.data[width*idx.y+idx.x]==INFLATED && inflation[idx.x][idx.y].empty()){
          occ_grid.data[width*idx.y+idx.x] = FREE;
        }
      }
    }
  }
}
