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
  wr(wr_init),
  v(0),
  a(0),
  w(0)
  {
    t=ros::WallTime::now();
    // initialize subscribers and publishers
    wheel_sub=nh->subscribe("/joint_states",1,&OdometryMM::callback_wheels,this);
    imu_sub = nh->subscribe("/imu",1,&OdometryMM::callback_imu,this);
    pos_pub = nh->advertise<geometry_msgs::Pose2D>("/robot_pose", 5);
    vel_pub = nh->advertise<geometry_msgs::Twist>("/robot_velocity", 5);
}

void OdometryMM::callback_imu(const sensor_msgs::Imu& msg){
  /* Gets information from Imu and saves it
  Parameters:
      msg       Output of Imu
  */
  compass = tf::getYaw(msg.orientation);  //extract info from quaterion
  w = msg.angular_velocity.z;
  a = msg.linear_acceleration.x;
}

void OdometryMM::callback_wheels(const sensor_msgs::JointState& msg){
  /* Gets information from wheel and calculates new position
  Parameters:
      msg       Output of wheel encoder
  */
  // sometimes it takes a while for the time object to initialize
  /*if(t==0){
    ROS_INFO("waiting for Odometry to start");
    t = ros::Time::now().toSec();
    return;
  }*/
  ros::WallTime start_t = ros::WallTime::now();
  float dt = (start_t-t).toNSec()*1e-9;
  //ROS_INFO("t %f, start %f",t.toNSec()*1e-9,start_t.toNSec()*1e-9);
  if(dt>1){
    t = start_t;
    //ROS_INFO("t %f, start %f",t.toNSec()*1e-9,start_t.toNSec()*1e-9);
    return;
  }
  float wheel_l = msg.position[1];
  float wheel_r = msg.position[0];
  float dwl = wheel_l - wl;
  float dwr = wheel_r - wr;
  float omega = (2*WR/(2*L*dt))*(dwr-dwl);
  float theta_od = pos.theta;
  float w_av;
  // MM for straight line
  if(std::abs(omega) < 3e-4){
    float v_od = ((2*WR)/(4*dt))*(dwl+dwr);
    float v_imu = v+a*dt;
    // fuse sensor readings
    v = WEIGHT_ODOMETRY*v_od+WEIGHT_IMU*v_imu;
    pos.x += v*dt*cos(pos.theta);
    pos.y += v*dt*sin(pos.theta);
    w_av = 0;
  }else{
    // MM for curves
    float rt = (L/2)*(dwl+dwr)/(dwr-dwl);    //turn radius
    float dphi_od = ((2*WR)/(2*L))*(dwr-dwl);
    float dphi_imu = w*dt;
    float dphi_av = WEIGHT_ODOMETRY*dphi_od+WEIGHT_IMU*dphi_imu;
    pos.x += -rt*sin(pos.theta)+rt*sin(pos.theta+dphi_av);
    pos.y += rt*cos(pos.theta)-rt*cos(pos.theta+dphi_av);
    theta_od += dphi_od;
    v = dphi_av*rt/dt;
    w_av = dphi_av/dt;
    //pos.theta += dphi_av;
  }
  wl = wheel_l;
  wr = wheel_r;
  t = start_t;

  //normalize angle to -pi,pi
  if(theta_od>M_PI) theta_od-=2*M_PI;
  if(theta_od<-M_PI) theta_od+=2*M_PI;

  // one angle close to pi and the other close to -pi
  if(theta_od>2 && compass<-2){
    theta_od-=2*M_PI;
  }
  if(theta_od<-2 && compass>2){
    theta_od+=2*M_PI;
  }
  pos.theta = (1-WEIGHT_COMPASS)*theta_od+WEIGHT_COMPASS*compass;

  //publish
  publish_pos();
  publish_vel(w_av,v);

  ros::WallTime t_end = ros::WallTime::now();
  double t_mm = (t_end - start_t).toNSec()*1e-6;
  // ROS_INFO("time for motion model: %f ms",t_mm);
}

void OdometryMM::publish_pos(){
  /* Publishes the calculated position
  */
  pos_pub.publish(pos);
}

void OdometryMM::publish_vel(float wz,float v){
  /* Publishes the calculated velocity
  */
  geometry_msgs::Twist vel;
  vel.linear.x = v;
  vel.angular.z = wz;
  vel_pub.publish(vel);
}
