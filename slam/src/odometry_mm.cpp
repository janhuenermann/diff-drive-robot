#include <slam/odometry_mm.h>
#include <math.h>

OdometryMM::OdometryMM(geometry_msgs::Pose2D pos_init,float wl_init ,float wr_init, ros::NodeHandle *nh):
  /* Constructor for map and lidar
  Parameters:
      *nh               Node handle required to make subscriptions
      wl_init           Initial angle of left wheel
      wr_init           Initial angle of right wheel
      pos_init          Initial position of the robot
  */
  pos(pos_init),
  wl(wl_init),
  wr(wr_init)
  {
    t_od = ros::Time::now().toSec();
    t_imu = ros::Time::now().toSec();
    // initialize subscribers and publishers
    wheel_sub=nh->subscribe("joint_states",1,&OdometryMM::callback_wheels,this);
    //imu_sub = nh->subscribe("imu",1,&OdometryMM::callback_imu,this);
    pos_pub = nh->advertise<geometry_msgs::Pose2D>("/robot_pose", 5);
}

void OdometryMM::callback_imu(const sensor_msgs::Imu& msg){
  /* Gets information from Imu and saves it
  Parameters:
      msg       Output of Imu
  */
  float dt = ros::Time::now().toSec()-t_imu;
  float compass = tf::getYaw(msg.orientation);
  //ROS_INFO("compass:%f",compass);
  float w = msg.angular_velocity.z;
  float dphi = w*dt;
  float a = msg.linear_acceleration.x;
  float s = (1/2)*a*dt*dt+v*dt;
  float dx = cos(pos.theta)*s;
  float dy = sin(pos.theta)*s;
  pos_imu.theta = compass;
  pos_imu.x += dx;
  pos_imu.y += dy;
  v += a*dt;
  t_imu += dt;
}

void OdometryMM::callback_wheels(const sensor_msgs::JointState& msg){
  /* Gets information from wheel and calculates new position
  Parameters:
      msg       Output of wheel encoder
  */
  float dt = ros::Time::now().toSec()-t_od;
  float wheel_l = msg.position[1];
  float wheel_r = msg.position[0];
  float dwl = wheel_l - wl;
  float dwr = wheel_r - wr;
  float w = (2*WR/(2*L*dt))*(dwr-dwl);

  // MM for straight line
  if(std::abs(w) < 1e-10){
    float vt = ((2*WR)/(4*dt))*(dwl+dwr);
    pos_od.x += vt*dt*cos(pos.theta);
    pos_od.y += vt*dt*sin(pos.theta);
    pos_od.theta += 0;    //unchanged, since straight
  }else{
    // MM for curves
    float rt = (L/2)*(dwl+dwr)/(dwr-dwl);    //turn radius
    float dphi = ((2*WR)/(2*L))*(dwr-dwl);
    pos_od.x += -rt*sin(pos.theta)+rt*sin(pos.theta+dphi);
    pos_od.y += rt*cos(pos.theta)-rt*cos(pos.theta+dphi);
    pos_od.theta += dphi;
  }
  wl = wheel_l;
  wr = wheel_r;
  t_od += dt;

  pos = pos_od;
  // publish the new position
  publish_pos();
}

void OdometryMM::publish_pos(){
  /* Publishes the calculated position
  */
  pos_pub.publish(pos);
}
