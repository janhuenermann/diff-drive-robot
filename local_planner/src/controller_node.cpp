#include "ros/ros.h"

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <local_planner/spline.hpp>
#include <local_planner/pid_controller.hpp>
#include <local_planner/trajectory.hpp>

class ControllerNode
{
public:

    ControllerNode() :
        nh_()
    {
        // PID params
        const double p_pos = 1.0, i_pos = 0.1, d_pos = 0.1;
        const double p_ang = 1.0, i_ang = 0.1, d_ang = 0.1;

        const double vel = 0.5;
        const double freq = 50.0;

        traj_.setVelocity(vel);
        traj_.setFrequency(freq);

        pid_.setParameters(freq, Vec2(p_pos, p_ang), Vec2(i_pos, i_ang), Vec2(d_pos, d_ang));

        pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        sub_robot_pose_ = nh_.subscribe<geometry_msgs::Pose2D>("/robot_pose", 1, &ControllerNode::robotPoseCallback, this);
        sub_robot_twist_ = nh_.subscribe<geometry_msgs::Twist>("/robot_velocity", 1, &ControllerNode::robotTwistCallback, this);
        sub_path_ = nh_.subscribe<nav_msgs::Path>("/navigation/path", 1, &ControllerNode::pathCallback, this);
    
        tick_timer_ = nh_.createTimer(ros::Duration(1.0 / freq), &ControllerNode::tickCallback, this);
    }

    void robotTwistCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        robot_velocity_.x = msg->linear.x * cos(msg->angular.z);
        robot_velocity_.y = msg->linear.x * sin(msg->angular.z);
        received_robot_twist_ = true;
    }

    void robotPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
    {
        robot_position_.x = msg->x;
        robot_position_.y = msg->y;
        received_robot_pose_ = true;

        pid_.setTruePosition(robot_position_);
        pid_.setTrueAngle(msg->theta);
    }

    void pathCallback(const nav_msgs::Path::ConstPtr& msg)
    {
        if (!received_robot_pose_ || !received_robot_twist_)
        {
            return ; // unlikely to happen due to planner only planning if it received position
        }

        std::vector<Point2> path;

        for (const geometry_msgs::PoseStamped &pt : msg->poses)
        {
            path.push_back(Point2(pt.pose.position.x, pt.pose.position.y));
        }

        // update our trajectory
        traj_.update(path, robot_position_, robot_velocity_);

        ROS_INFO("Trajectory updated; remaining path length: %.4lf", traj_.getRemainingPathLength());
    }

    void tickCallback(const ros::TimerEvent& evt)
    {
        if (!traj_.hasPath())
        {
            return ;
        }

        double linear_vel, angular_vel;

        Point2 intermediate_point = traj_.getNextTargetPoint();

        pid_.setTargetPosition(intermediate_point);
        pid_.update(linear_vel, angular_vel);

        geometry_msgs::Twist twist;
        twist.linear.x = linear_vel;
        twist.angular.z = angular_vel;

        pub_cmd_vel_.publish(twist);

        traj_.next();
    }

protected:
    Trajectory traj_;
    PIDController pid_;

    Point2 robot_position_;
    bool received_robot_pose_;

    Point2 robot_velocity_;
    bool received_robot_twist_;

    ros::NodeHandle nh_;
    ros::Publisher pub_cmd_vel_;
    ros::Subscriber sub_robot_pose_;
    ros::Subscriber sub_robot_twist_;
    ros::Subscriber sub_path_;
    ros::Timer tick_timer_;

};

// Main function of the slam package
int main(int argc, char **argv){
  // Init Ros
  ros::init(argc,argv, "controller_node");
  ros::NodeHandle n;

  ros::spin();
  return 0;
}
