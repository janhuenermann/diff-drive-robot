#include <slam/make_map.h>

MapIdx::MapIdx(int x_pos,int y_pos){
  x = x_pos;
  y = y_pos;
}

bool MapIdx::operator<(const MapIdx& rhs) const{
  if(x<rhs.x) return true;
  if(x==rhs.x && y<rhs.y) return true;
  return false;
}

LidarMap::LidarMap(ros::NodeHandle *nh,geometry_msgs::Pose2D mi_pos,geometry_msgs::Pose2D max_pos,float c_size,float inf_radius):
  inflation_radius(inf_radius),
  min_pos(mi_pos),
  cell_size(c_size)
  {
    scan_sub=nh->subscribe("scan",10,&LidarMap::callback_scan,this);
    pos_sub = nh->subscribe("robot_pose",1,&LidarMap::callback_pos,this);
    grid_pub = nh->advertise<nav_msgs::OccupancyGrid>("/map", 5);
    occ_grid.info.resolution = cell_size;
    occ_grid.info.origin.position.x = min_pos.x;
    occ_grid.info.origin.position.y = min_pos.y;
    width = ceil((max_pos.x-min_pos.x)/cell_size);
    height = ceil((max_pos.y-min_pos.y)/cell_size);
    occ_grid.info.width = width;
    occ_grid.info.height = height;
    mask_inflation = gen_mask();
    l_occ = log(0.65/0.35);
    l_free = log(0.35/0.65);
    log_odds.resize(width, std::vector<float>(height));
    std::fill(log_odds.begin(), log_odds.end(), std::vector<float>(height, 0));
    inflation.resize(width, std::vector<std::set<MapIdx>>(height));
    occ_grid.data.resize(width*height);
    std::fill(occ_grid.data.begin(),occ_grid.data.end(),-1);
}

void LidarMap::callback_scan(const sensor_msgs::LaserScan& msg){
 std::vector<float> ranges = msg.ranges;
 for(int i=0;i<360;i++){
   geometry_msgs::Pose2D end_pos;
   if(ranges[i]!=INFINITY){
     end_pos = inverseSensorModel(robot_pos,ranges[i],i);
     MapIdx end_idx = pos2idx(end_pos);
     if(idx_in_map(end_idx)){
       update_at_idx(end_idx,true);
     }
   }else{
     end_pos = inverseSensorModel(robot_pos,max_range,i);
     MapIdx end_idx = pos2idx(end_pos);
     if(idx_in_map(end_idx)){
       update_at_idx(end_idx,false);
     }
   }
   std::vector<MapIdx> los_idx = line_of_sight(end_pos);
   for(int i=0;i<los_idx.size();i++){
     if(idx_in_map(los_idx[i])){
       update_at_idx(los_idx[i],false);
     }
   }
 }
 publish_grid();
}

void LidarMap::callback_pos(const geometry_msgs::Pose2D& msg){
  robot_pos = msg;
}

geometry_msgs::Pose2D LidarMap::idx2pos(MapIdx idx){
  geometry_msgs::Pose2D res;
  res.x = idx.x * cell_size + min_pos.x;
  res.y = idx.y * cell_size + min_pos.y;
  return res;
}

MapIdx LidarMap::pos2idx(geometry_msgs::Pose2D pos){
  int x = round((pos.x - min_pos.x)/cell_size);
  int y = round((pos.y - min_pos.y)/cell_size);
  MapIdx res(x,y);
  return res;
}

geometry_msgs::Pose2D LidarMap::pos2idx_no_round(geometry_msgs::Pose2D pos){
  geometry_msgs::Pose2D res;
  res.x = (pos.x - min_pos.x)/cell_size;
  res.y = (pos.y - min_pos.y)/cell_size;
  return res;
}

geometry_msgs::Pose2D LidarMap::inverseSensorModel(geometry_msgs::Pose2D pos,float rng,float deg){
    float rad = (M_PI/180)*deg;
    geometry_msgs::Pose2D res;
    res.x = pos.x + rng*cos(pos.theta+rad);
    res.y = pos.y + rng*sin(pos.theta+rad);
    return res;
}

std::vector<MapIdx> LidarMap::gen_mask(){
  std::vector<MapIdx> res;
  MapIdx curr(0,0);
  grid_pub.publish(occ_grid);
  int x_max = round(inflation_radius/cell_size); // max idx
  float r_idx = inflation_radius/cell_size;
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
  grid_pub.publish(occ_grid);
}

std::vector<MapIdx> LidarMap::line_of_sight(geometry_msgs::Pose2D end_pos){
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
  if(idx.x<occ_grid.info.width && idx.y<occ_grid.info.height){
    return true;
  }
  return false;
}

void LidarMap::update_at_idx(MapIdx idx, bool occupied){
  if(occupied){
    log_odds[idx.x][idx.y] += l_occ;
    if(log_odds[idx.x][idx.y]>L_THRESH){
      occ_grid.data[idx.x*width+idx.y] = OCCUPIED;
      add_inflation(idx,occupied);
    }
  }else{
    log_odds[idx.x][idx.y] += l_free;
    if(log_odds[idx.x][idx.y]<-L_THRESH){
      occ_grid.data[idx.x*width+idx.y] = FREE;
      add_inflation(idx,occupied);
    }
  }
}

// occupied is the new occupation status
void LidarMap::add_inflation(MapIdx center,bool occupied){
  MapIdx idx(0,0);
  for(int i=0;i<mask_inflation.size();i++){
    idx.x = center.x+mask_inflation[i].x;
    idx.y = center.y+mask_inflation[i].y;
    if(idx_in_map(idx)){
      if(occupied){
        inflation[idx.x][idx.y].insert(center);
        if(occ_grid.data[width*idx.x+idx.y]==FREE){
          occ_grid.data[width*idx.x+idx.y] = INFLATED;
        }
      }else{
        inflation[idx.x][idx.y].erase(center);
        if(occ_grid.data[width*idx.x+idx.y]==INFLATED && inflation[idx.x][idx.y].empty()){
          occ_grid.data[width*idx.x+idx.y] = FREE;
        }
      }
    }
  }
}
