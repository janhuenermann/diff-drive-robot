#include <slam/odometry_mm.h>
#include <math.h>

OdometryMM::OdometryMM(geometry_msgs::Pose2D pos_init,float wl_init ,float wr_init, ros::NodeHandle *nh):
  pos(pos_init),
  wl(wl_init),
  wr(wr_init)
  {
    t = ros::Time::now().toSec();

    wheel_sub=nh->subscribe("joint_states",1,&OdometryMM::callback_wheels,this);
    pos_pub = nh->advertise<geometry_msgs::Pose2D>("/robot_pose", 5);
    ROS_INFO("Slam inited");
}

void OdometryMM::callback_wheels(const sensor_msgs::JointState& msg){
  float dt = ros::Time::now().toSec()-t;
  float wheel_l = msg.position[0];
  float wheel_r = msg.position[1];
  float dwl = wheel_l - wl;
  float dwr = wheel_r - wr;
  float w = (2*WR/(2*L*dt))*(dwr-dwl);

// MM for straight line
  if(std::abs(w) < 1e-10){
    float vt = ((2*WR)/(4*dt))*(dwl+dwr);
    pos.x += vt*dt*cos(pos.theta);
    pos.y += vt*dt*sin(pos.theta);
    pos.theta += 0;    //unchanged, since straight
  }else{
    // MM for curves
    float rt = (L/2)*(dwl+dwr)/(dwr-dwl);    //turn radius
    float dphi = ((2*WR)/(2*L))*(dwr-dwl);
    pos.x += -rt*sin(pos.theta)+rt*sin(pos.theta+dphi);
    pos.y += rt*cos(pos.theta)-rt*cos(pos.theta+dphi);
    pos.theta += dphi;
  }
  wl = wheel_l;
  wr = wheel_r;
  t += dt;
  publish_pos();
}

void OdometryMM::publish_pos(){
  pos_pub.publish(pos);
}
