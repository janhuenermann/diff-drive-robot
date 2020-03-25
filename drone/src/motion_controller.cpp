#include <drone/pid_controller.hpp>

#include "ros/ros.h"

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <hector_uav_msgs/EnableMotors.h>
#include <nav_msgs/Odometry.h>

class ControllerNode
{
public:
    ControllerNode(){
        // 0: take off 1:to goal 2:to robot 3:to start 4:land 5:done
        mission_state = 0;
        // PID params
        ros::NodeHandle nh;
        motor_client = nh.serviceClient<hector_uav_msgs::EnableMotors>("enable_motors");
        srv.request.enable = true;
        motor_client.call(srv);
        const double freq = 100.0;
        const double p_pos = 1.7, i_pos = 0.1, d_pos = 1.3;
        const double p_height = 1.1, i_height = 0.01, d_height = 0.2;

        pid.setParameters_freq(freq);
        pid.setParameters_pos(p_pos,i_pos,d_pos);
        pid.setParameters_height(p_height,i_height,d_height);

        pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

        sub_drone_pos = nh.subscribe<nav_msgs::Odometry>("/drone/ground_truth/state", 1, &ControllerNode::dronePoseCallback, this);
        sub_navigation_state = nh.subscribe<std_msgs::Bool>("/ground/navigation/done",1, &ControllerNode::navDoneCallback, this);
        sub_ground_goal = nh.subscribe<geometry_msgs::Pose2D>("/ground/navigation/goal", 1,&ControllerNode::navGoalCallback, this);
        sub_robot_ground_pos = nh.subscribe<geometry_msgs::Pose2D>("/ground/robot_pose", 1,&ControllerNode::groundRobCallback, this);
        tick_timer = nh.createTimer(ros::Duration(1.0 / freq), &ControllerNode::tickCallback, this);

        ROS_INFO("Drone controller node booted.");
    }

    void groundRobCallback(const geometry_msgs::Pose2D::ConstPtr& msg){
      ground_rob_pos.x = msg->x;
      ground_rob_pos.y = msg->y;
    }

    void navDoneCallback(const std_msgs::Bool::ConstPtr& msg){
      ground_done = msg->data;
    }

    void navGoalCallback(const geometry_msgs::Pose2D::ConstPtr& msg){
      received_target = true;
      curr_goal_ground.x = msg->x;
      curr_goal_ground.y = msg->y;
    }

    void dronePoseCallback(const nav_msgs::Odometry::ConstPtr& msg){
      drone_pos = msg->pose.pose;
      if(!received_drone_pos){
        received_drone_pos = true;
        start_pos = drone_pos.position;
      }
      pid.set_true_pos(drone_pos.position);
      pid.set_true_angles(drone_pos.orientation);
    }

    double dist(geometry_msgs::Point p1, geometry_msgs::Point p2){
      double dx = p1.x-p2.x;
      double dy = p1.y-p2.y;
      double dz = p1.z-p2.z;
      return sqrt(pow(dx,2)+pow(dy,2)+pow(dz,2));
    }

    void tickCallback(const ros::TimerEvent& evt){
      ROS_INFO("d %f",dist(drone_pos.position,goal_pos));
      if(!received_drone_pos) return;
      if(mission_state == 5) return;

      if(mission_state == 1 || mission_state == 2){
        if(dist(drone_pos.position,goal_pos)<thresh_dist){
          if(mission_state == 1){
            mission_state = 2;
          }else{
            mission_state = 1;
          }
        }
      }

      if(mission_state == 0 && dist(drone_pos.position,goal_pos)<thresh_dist){
        mission_state = 1;
      }

      if(dist(goal_pos,drone_pos.position)<thresh_dist && mission_state==3){
        mission_state = 4;
      }
      if(mission_state<3 && ground_done){
        mission_state = 3;
      }
      if(mission_state == 4 && dist(start_pos,drone_pos.position)<thresh_dist/2){
        mission_state = 5;
        srv.request.enable = false;
        motor_client.call(srv);
      }

      if(mission_state == 0){
        goal_pos = start_pos;
        goal_pos.z = cruise_height;
        goal_w = 0;
      }
      if(mission_state == 1){
        goal_pos.x = curr_goal_ground.x;
        goal_pos.y = curr_goal_ground.y;
        goal_pos.z = cruise_height;
        goal_w = yaw_speed;
      }
      if(mission_state == 2){
        goal_pos = ground_rob_pos;
        goal_pos.z = cruise_height;
        goal_w = yaw_speed;
      }
      if(mission_state == 3){
        goal_pos = start_pos;
        goal_pos.z = cruise_height;
        goal_w = yaw_speed;
      }
      if(mission_state == 4){
        goal_pos = start_pos;
        goal_w = 0;
      }

      geometry_msgs::Twist twist;
      double vx,vy,vz;
      pid.set_target_pos(goal_pos);
      pid.update(vx,vy,vz);
      twist.linear.x = vx;
      twist.linear.y = vy;
      twist.linear.z = vz;
      twist.angular.z = goal_w;

      pub_cmd_vel.publish(twist);
    }

protected:
  const double yaw_speed = 36.0/180*M_PI;
  const double thresh_dist = 0.3;
  const double cruise_height = 5;

  hector_uav_msgs::EnableMotors srv;
  ros::ServiceClient motor_client;

  PIDController pid;

  int mission_state;
  bool ground_done = false;

  Point2 curr_goal_ground;
  geometry_msgs::Point goal_pos;
  double goal_w;
  geometry_msgs::Point start_pos;
  geometry_msgs::Point ground_rob_pos;
  geometry_msgs::Pose drone_pos;

  Point2 curr_position;
  bool received_drone_pos = false;
  bool received_target = false;

  ros::Publisher pub_cmd_vel;
  ros::Subscriber sub_drone_pos;
  ros::Subscriber sub_navigation_state;
  ros::Subscriber sub_ground_goal;
  ros::Subscriber sub_robot_ground_pos;
  ros::Timer tick_timer;
};


int main(int argc, char **argv){
  // Init Ros
  ros::init(argc,argv, "drone_motion_node");

  ControllerNode node;

  ros::spin();
  return 0;
}
