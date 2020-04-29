#include <drone/pid_controller.hpp>

#include "ros/ros.h"

#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <hector_uav_msgs/EnableMotors.h>
#include <nav_msgs/Odometry.h>

#include <math/SplinePathData.h>
#include <math/spline.hpp>

class ControllerNode
{
public:
    ControllerNode():ground_spline(nullptr){
        // 0: take off 1:to goal 2:to robot 3:to start 4:land 5:land softly 6:turn off motors
        mission_state = 0;

        ros::NodeHandle nh;
        // Start motors
        motor_client = nh.serviceClient<hector_uav_msgs::EnableMotors>("enable_motors");
        srv.request.enable = true;
        motor_client.call(srv);
        // frequency of PID controller
        pid.setParameters_freq(freq);

        // Initialize and subscribe topics
        pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        pub_goal = nh.advertise<geometry_msgs::Pose2D>("navigation/drone_goal",1);
        pub_intersection = nh.advertise<geometry_msgs::Pose2D>("navigation/intersection",1);

        sub_drone_pos = nh.subscribe<nav_msgs::Odometry>("ground_truth/state", 1, &ControllerNode::dronePoseCallback, this);
        sub_navigation_state = nh.subscribe<std_msgs::Bool>("/ground/navigation/done",1, &ControllerNode::navDoneCallback, this);
        sub_ground_goal = nh.subscribe<geometry_msgs::Pose2D>("/ground/navigation/last_goal", 1,&ControllerNode::navGoalCallback, this);
        sub_robot_ground_pos = nh.subscribe<geometry_msgs::Pose2D>("/ground/robot_pose", 1,&ControllerNode::groundRobCallback, this);
        sub_traj = nh.subscribe<math::SplinePathData>("/ground/navigation/trajectory", 1, &ControllerNode::groundTrajCallback, this);
        sub_sonar = nh.subscribe<sensor_msgs::Range>("sonar_height",1,&ControllerNode::sonarCallback, this);

        tick_timer = nh.createTimer(ros::Duration(1.0 / freq), &ControllerNode::tickCallback, this);

        ROS_INFO("Drone controller node booted.");
    }

    void groundRobCallback(const geometry_msgs::Pose2D::ConstPtr& msg){
      //Save Position of ground robot, set cruise height as target z coordinate
      ground_rob_pos.x = msg->x;
      ground_rob_pos.y = msg->y;
      ground_rob_pos.z = cruise_height;
    }

    void groundTrajCallback(const math::SplinePathData::ConstPtr& msg){
      // Decode and save ground trajectory
      if (ground_spline != nullptr){
          delete ground_spline;
      }
      ground_spline = SplinePath::fromData(*msg);
      goal_ground_rob = calculateNewIntersection();
    }

    void sonarCallback(const sensor_msgs::Range::ConstPtr& msg){
      // Save height measurement of sonar
      sonar_height = msg->range;
    }

    geometry_msgs::Point calculateNewIntersection(){
      // Calculate the point, where drone and turtle take the same amout to reach
      double dist_on_spline = ground_spline->length;
      // Initialize
      geometry_msgs::Point intersection;
      intersection.x = ground_rob_pos.x;
      intersection.y = ground_rob_pos.y;
      intersection.z = cruise_height;
      double i_best;
      geometry_msgs::Point P_best;
      double t_best=-1;
      Point2 intersection_ground;
      // Devide spline in 100 pieces and find the best point out of the 100
      for(double i=0;i<1;i+=0.01){
        intersection_ground = ground_spline->normalizedPosition(i);
        intersection.x = intersection_ground.x;
        intersection.y = intersection_ground.y;
        intersection.z = cruise_height;
        double dist_drone = dist(drone_pos.position,intersection);
        double t_drone = dist_drone/est_drone_speed;
        double t_robot = dist_on_spline*i/est_ground_speed;
        if(std::abs(t_drone-t_robot)<t_best || t_best<0){
          t_best = (std::abs(t_drone-t_robot));
          P_best = intersection;
        }
      }
      return P_best;
    }

    void navDoneCallback(const std_msgs::Bool::ConstPtr& msg){
      // Return home trigger
      ground_done = msg->data;
    }

    void navGoalCallback(const geometry_msgs::Pose2D::ConstPtr& msg){
      // Final goal of the turtle
      received_target = true;
      goal_ground.x = msg->x;
      goal_ground.y = msg->y;
    }

    void dronePoseCallback(const nav_msgs::Odometry::ConstPtr& msg){
      // Read drone pose and give to PID controler
      drone_pos = msg->pose.pose;
      if(!received_drone_pos){
        received_drone_pos = true;
        start_pos = drone_pos.position;
      }
      pid.set_true_pos(drone_pos.position);
      pid.set_true_angles(drone_pos.orientation);
      pid.set_true_vel(msg->twist.twist.linear);
    }

    double dist(geometry_msgs::Point p1, geometry_msgs::Point p2){
      // Eucledian distance
      double dx = p1.x-p2.x;
      double dy = p1.y-p2.y;
      double dz = p1.z-p2.z;
      return sqrt(pow(dx,2)+pow(dy,2)+pow(dz,2));
    }

    void tickCallback(const ros::TimerEvent& evt){
      // Update of FSM
      // Do nothing if drone position not known, or mission not started
      if(!received_drone_pos) return;
      if(mission_state == 6) return;

      // FSM transitions:
      if(dist(drone_pos.position,goal_pos)<thresh_dist && mission_state == 1){
          mission_state = 2;
      }else{
        if(dist(drone_pos.position,ground_rob_pos)<thresh_dist && mission_state == 2){
            mission_state = 1;
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
      if(mission_state == 4 && sonar_height<3){
        mission_state = 5;
      }

      if(mission_state == 0){
        goal_pos = start_pos;
        goal_pos.z = cruise_height;
        goal_w = 0;
      }
      if(mission_state == 1){
        goal_pos.x = goal_ground.x;
        goal_pos.y = goal_ground.y;
        goal_pos.z = cruise_height;
        goal_w = yaw_speed;
      }
      if(mission_state == 2){
        goal_pos = goal_ground_rob;
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
      if(mission_state == 5){
        if(sonar_height>=3){
          mission_state = 4;
        }else{
          goal_pos = start_pos;
          goal_pos.z = drone_pos.position.z-sonar_height+sonar_landed_threshold-0.15;
          if(sonar_height<sonar_landed_threshold){
            landing_time += 1.0/freq;
            if(landing_time >= ground_time_landing){
              // Shut down motors if landed long enough
              srv.request.enable = false;
              motor_client.call(srv);
              mission_state = 6;
            }
          }else{
            landing_time = 0;
          }
        }
      }
      pid.set_phase(mission_state);

      // calculate new velocity commands and publish
      geometry_msgs::Twist twist;
      double vx,vy,vz;
      pid.set_target_pos(goal_pos);
      pid.update(vx,vy,vz);
      twist.linear.x = vx;
      twist.linear.y = vy;
      twist.linear.z = vz;
      twist.angular.z = goal_w;

      pub_cmd_vel.publish(twist);
      geometry_msgs::Pose2D p;
      p.x = goal_pos.x;
      p.y = goal_pos.y;
      pub_goal.publish(p);
    }

protected:
  // Constants
  const double freq = 50.0;

  const double yaw_speed = 36.0/180*M_PI;
  const double thresh_dist = 0.2;
  const double cruise_height = 5.5;

  const double est_ground_speed = 0.8;
  const double est_drone_speed = 1.7;

  const double sonar_landed_threshold = 0.18;
  const double ground_time_landing = 5;

  // Motor service
  hector_uav_msgs::EnableMotors srv;
  ros::ServiceClient motor_client;

  PIDController pid;

  int mission_state;
  bool ground_done = false;

  // Important positions
  Point2 goal_ground;
  geometry_msgs::Point goal_pos;
  double goal_w;
  geometry_msgs::Point start_pos;
  geometry_msgs::Point ground_rob_pos;
  geometry_msgs::Point goal_ground_rob;
  geometry_msgs::Pose drone_pos;
  SplinePath *ground_spline;
  double sonar_height;
  double landing_time = 0;
  Point2 curr_position;


  bool received_drone_pos = false;
  bool received_target = false;
  bool landing = false;

  ros::Publisher pub_cmd_vel;
  ros::Publisher pub_goal;
  ros::Publisher pub_intersection;
  ros::Subscriber sub_drone_pos;
  ros::Subscriber sub_navigation_state;
  ros::Subscriber sub_ground_goal;
  ros::Subscriber sub_robot_ground_pos;
  ros::Subscriber sub_traj;
  ros::Subscriber sub_sonar;
  ros::Timer tick_timer;
};


int main(int argc, char **argv){
  // Init Ros
  ros::init(argc,argv, "drone_motion_node");

  ControllerNode node;

  ros::spin();
  return 0;
}
