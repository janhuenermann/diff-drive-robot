#include <drone/pid_controller.hpp>

PIDController::PIDController() :
    prev_update_time(ros::Time(0))
{
    ros::NodeHandle nh;
    pub_err = nh.advertise<geometry_msgs::Point>("PID_tune_e", 10);
    pub_int = nh.advertise<geometry_msgs::Point>("PID_tune_i", 10);
}


// Functions to set parameters:
void PIDController::setParameters_freq(double freq){
    dt = 1.0 / freq;
}


void PIDController::set_true_pos(geometry_msgs::Point pos){
    drone_xy.x = pos.x;
    drone_xy.y = pos.y;
    drone_z = pos.z;
}

void PIDController::set_true_angles(geometry_msgs::Quaternion q){
    drone_q = q;
}

void PIDController::set_true_vel(geometry_msgs::Vector3 vel){
  drone_v_xy.x = vel.x;
  drone_v_xy.y = vel.y;
}

void PIDController::set_target_pos(geometry_msgs::Point target){
    goal_xy.x = target.x;
    goal_xy.y = target.y;
    goal_z = target.z;
}

// Publishing functions
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

void PIDController::set_phase(int phase){
  // Select the appropriate PID coefficients according to phase
  if(phase!=curr_state){
    // Reset integrator on phase change
    integrated_x = 0;
    integrated_y = 0;
  }
  curr_state = phase;
  switch (phase) {
    case 0:
    case 4:
    p_xy = lp_xy; i_xy = li_xy; d_xy = ld_xy;
    p_z = lp_z; i_z = li_z; d_z = ld_z;
    break;
  case 1:
    p_xy = tp_xy; i_xy = ti_xy; d_xy = td_xy;
    p_z = tp_z; i_z = ti_z; d_z = td_z;
    break;
  case 2:
  case 3:
    p_xy = kp_xy; i_xy = ki_xy; d_xy = kd_xy;
    p_z = kp_z; i_z = ki_z; d_z = kd_z;
    break;
  case 5:
    p_xy = lap_xy; i_xy = lai_xy; d_xy = lad_xy;
    p_z = lap_z; i_z = lai_z; d_z = lad_z;
  break;
  case 6:
    p_xy = 0; i_xy = 0; d_xy =0;
    p_z = 0; i_z = 0; d_z = 0;
  break;
  }
}

void PIDController::update(double &vel_x, double &vel_y,double &vel_z){
    // calculate velocity commands from error
    double time_delta = (ros::Time::now() - prev_update_time).toSec();

    // Calculate error
    double e_x = goal_xy.x - drone_xy.x;
    double e_y = goal_xy.y - drone_xy.y;
    double e_z = goal_z - drone_z;
    publish_e(e_x,e_y,e_z);

// Velocities in world frame
    double vel_x_w = p_xy*e_x + i_xy*integrated_x + d_xy*(e_x-e_prev_xy.x)/time_delta;
    double vel_y_w = p_xy*e_y + i_xy*integrated_y + d_xy*(e_y-e_prev_xy.y)/time_delta;
    double vel_z_w = p_z*e_z + i_z*integrated_z + d_z*(e_z-e_prev_z)/time_delta;

    // Limiting to fulfill constraints
    vel_x_w = std::max(std::min(drone_v_xy.x+a_max*time_delta,vel_x_w),drone_v_xy.x-a_max*time_delta);
    vel_y_w = std::max(std::min(drone_v_xy.y+a_max*time_delta,vel_y_w),drone_v_xy.y-a_max*time_delta);

    double abs_vel_xy = sqrt(pow(vel_x_w,2)+pow(vel_y_w,2));
    if(abs_vel_xy>maxv_xy){
        vel_x_w = maxv_xy*(vel_x_w/abs_vel_xy);
        vel_y_w = maxv_xy*(vel_y_w/abs_vel_xy);
    }
    vel_z_w = std::min(std::max(minv_z,vel_z_w),maxv_z);
    saturated_z = false;
    if(vel_z_w==minv_z || vel_z_w==maxv_z) saturated_z = true;

// Convert velocities to robot frame
    tf::Quaternion q(drone_q.x, drone_q.y, drone_q.z,drone_q.w);
    tf::Quaternion p(vel_x_w,vel_y_w,vel_z_w,0);
    tf::Quaternion pd = q.inverse()*p*q;
    vel_x = pd.getX();
    vel_y = pd.getY();
    vel_z = pd.getZ();

    // Only use integrator close to steady state
    if(abs(e_x)>threshold_ss){
        integrated_x = 0;
    }else{
        integrated_x = std::min(std::max(integrated_x+i_xy*e_x*time_delta,-windup_xy),windup_xy);
    }
    if(abs(e_y)>threshold_ss){
        integrated_y = 0;
    }else{
        integrated_y = std::min(std::max(integrated_y+i_xy*e_y*time_delta,-windup_xy),windup_xy);
    }
    if(saturated_z){
      integrated_z = 0;
  }else{
      integrated_z = std::min(std::max(integrated_z+i_z*e_z*time_delta,-windup_z),windup_z);
  }

  publish_int(integrated_x,integrated_y,integrated_z);

  e_prev_xy.x = e_x;
  e_prev_xy.y = e_y;
  e_prev_z = e_z;

  prev_update_time = ros::Time::now();
}
