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
    imu_sub = nh->subscribe("imu",1,&OdometryMM::callback_imu,this);
    pos_pub = nh->advertise<geometry_msgs::Pose2D>("/robot_pose", 5);
}

void OdometryMM::callback_imu(const sensor_msgs::Imu& msg){
  /* Gets information from Imu and saves it
  Parameters:
      msg       Output of Imu
  */
  float dt = ros::Time::now().toSec()-t_imu;
  float compass = tf::getYaw(msg.orientation);  //extract info from quaterion
  float w = msg.angular_velocity.z;
  float dphi = w*dt;
  float a = msg.linear_acceleration.x;          //use s = 1/2*a*t*t+v*t
  float s = (1/2)*a*dt*dt+v*dt;
  float dx = cos(pos.theta)*s;
  float dy = sin(pos.theta)*s;
  pos_imu.theta = compass;
  pos_imu.x += dx;
  pos_imu.y += dy;
  /*ROS_INFO("x_imu:%f",pos_imu.x);
  ROS_INFO("x_od:%f",pos_od.x);
  ROS_INFO("y_imu:%f",pos_imu.y);
  ROS_INFO("y_od:%f",pos_od.y);
  ROS_INFO("t_imu:%f",pos_imu.theta);
  ROS_INFO("t_od:%f",pos_od.theta);
  ROS_INFO("-----");*/
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
    pos_od.y += vt*dt*sin(pos.theta); //theta unchanged, since straight
  }else{
    // MM for curves
    float rt = (L/2)*(dwl+dwr)/(dwr-dwl);    //turn radius
    float dphi = ((2*WR)/(2*L))*(dwr-dwl);
    pos_od.x += - rt*sin(pos.theta)+rt*sin(pos.theta+dphi);
    pos_od.y += rt*cos(pos.theta)-rt*cos(pos.theta+dphi);
    pos_od.theta += dphi;
  }
  // normalize to [-pi,pi]
  if(pos_od.theta>M_PI) pos_od.theta-=2*M_PI;
  if(pos_od.theta<-M_PI) pos_od.theta+=2*M_PI;
  wl = wheel_l;
  wr = wheel_r;
  t_od += dt;
  if(pos_od.theta>2 && pos_imu.theta<-2){
    pos_od.theta-=2*M_PI;
  }
  if(pos_od.theta<-2 && pos_imu.theta>2){
    pos_od.theta+=2*M_PI;
  }
  /*pos.theta = WEIGHT_ODOMETRY*pos_od.theta+WEIGHT_IMU*pos_imu.theta;
  pos.x = WEIGHT_ODOMETRY*pos_od.x+WEIGHT_IMU*pos_imu.x;
  pos.y = WEIGHT_ODOMETRY*pos_od.y+WEIGHT_IMU*pos_imu.y;*/
  pos.theta = pos_od.theta;
  pos.x = pos_od.x;
  pos.y = pos_od.y;
  // publish the new position
  publish_pos();
}

void OdometryMM::publish_pos(){
  /* Publishes the calculated position
  */
  pos_pub.publish(pos);
}
