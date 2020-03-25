#include <drone/pid_controller.hpp>


PIDController::PIDController():
  prev_update_time(ros::Time(0)){
  ros::NodeHandle nh;
  pub_err = nh.advertise<geometry_msgs::Point>("PID_tune_e", 10);
  pub_int = nh.advertise<geometry_msgs::Point>("PID_tune_i", 10);
}

void PIDController::setParameters_freq(double freq){
  dt = 1.0 / freq;
}

void PIDController::setParameters_height(double p,double i, double d){
  p_height = p;
  i_height = i;
  d_height = d;
  if(i>0) max_int_height = anti_windup/i;
}

void PIDController::setParameters_pos(double p,double i, double d){
  p_pos = p;
  i_pos = i;
  d_pos = d;
  if(i>0) max_int_pos = anti_windup/i;
}


void PIDController::set_true_pos(geometry_msgs::Point pos){
    drone_pos.x = pos.x;
    drone_pos.y = pos.y;
    drone_height = pos.z;
}

void PIDController::set_true_angles(geometry_msgs::Quaternion q){
    drone_q = q;
}

void PIDController::set_target_pos(geometry_msgs::Point target){
    goal_pos.x = target.x;
    goal_pos.y = target.y;
    goal_height = target.z;
}

void PIDController::publish_e(double ex, double ey, double ez){
  geometry_msgs::Point e;
  e.x = ex;
  e.y = ey;
  e.z = ez;
  pub_err.publish(e);
}

void PIDController::publish_int(double ix, double iy, double iz){
  geometry_msgs::Point i;
  i.x = ix;
  i.y = iy;
  i.z = iz;
  pub_int.publish(i);
}

void PIDController::update(double &vel_x, double &vel_y,double &vel_z){
    prev_time_delta = (ros::Time::now() - prev_update_time).toSec();

    double e_pos_x = goal_pos.x - drone_pos.x;
    double e_pos_y = goal_pos.y - drone_pos.y;
    double e_height = goal_height - drone_height;
    publish_e(e_pos_x,e_pos_y,e_height);

    // Velocities in world frame
    double vel_x_w = p_pos*e_pos_x + i_pos*pos_integrated_x + d_pos*(e_pos_x-e_prev_pos.x);
    double vel_y_w = p_pos*e_pos_y + i_pos*pos_integrated_y + d_pos*(e_pos_y-e_prev_pos.y);
    double vel_z_w = p_height*e_height + i_height*height_integrated + d_height*(e_height-e_prev_height);
    double abs_vel_pos = sqrt(pow(vel_x_w,2)+pow(vel_y_w,2));

    if(abs_vel_pos>maxv_pos){
      vel_x_w = maxv_pos*(vel_x_w/abs_vel_pos);
      vel_y_w = maxv_pos*(vel_y_w/abs_vel_pos);
    }
    vel_z_w = std::min(std::max(minv_height,vel_z_w),maxv_height);
    height_saturated = false;
    if(vel_z_w==minv_height || vel_z_w==maxv_height) height_saturated = true;
    // Convert velocities to robot frame
    tf::Quaternion q(drone_q.x, drone_q.y, drone_q.z, drone_q.w);
    tf::Quaternion p(vel_x_w,vel_y_w,vel_z_w,0);
    tf::Quaternion pd = q.inverse()*p*q;
    vel_x = pd.getX();
    vel_y = pd.getY();
    vel_z = pd.getZ();

    if(abs(e_pos_x)>threshold_ss){
      pos_integrated_x = 0;
    }else{
      pos_integrated_x = std::min(std::max(pos_integrated_x+e_pos_x*dt,-max_int_pos),max_int_pos);
    }

    if(abs(e_pos_y)>threshold_ss){
      pos_integrated_y = 0;
    }else{
      pos_integrated_y = std::min(std::max(pos_integrated_y+e_pos_y*dt,-max_int_pos),max_int_pos);
    }
    if(height_saturated){
        height_integrated = 0;
    }else{
        height_integrated = std::min(std::max(height_integrated+e_height*dt,-max_int_height),max_int_height);
    }
    publish_int(pos_integrated_x,pos_integrated_y,height_integrated);
    e_prev_pos.x = e_pos_x;
    e_prev_pos.y = e_pos_y;
    e_prev_height = e_height;

    prev_update_time = ros::Time::now();
}
