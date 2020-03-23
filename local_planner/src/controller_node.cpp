#include "ros/ros.h"

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <math/SplinePathData.h>
#include <math/util.hpp>

#include <local_planner/pid_controller.hpp>
#include <local_planner/trajectory.hpp>

class ControllerNode
{
public:

    ControllerNode() :
        nh_()
    {
        // PID params
        const double freq = 100.0;
        const double p_pos = 1.80, i_pos = 0.00, d_pos = 0.100;
        const double p_ang = 1.00, i_ang = 0.00, d_ang = 0.000;

        pid_.setParameters(freq, Vec2(p_pos, p_ang), Vec2(i_pos, i_ang), Vec2(d_pos, d_ang));
        
        traj_.setDistance(0.25);
        traj_.setStepSize(1e-3);

        ros::Duration(5.0).sleep();

        pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        pub_traj_ = nh_.advertise<math::SplinePathData>("navigation/trajectory", 1, true);
        pub_pursuit_point_ = nh_.advertise<geometry_msgs::Pose2D>("navigation/pursuit_point", 5);

        sub_robot_pose_ = nh_.subscribe<geometry_msgs::Pose2D>("robot_pose", 1, &ControllerNode::robotPoseCallback, this);
        sub_robot_twist_ = nh_.subscribe<geometry_msgs::Twist>("robot_velocity", 1, &ControllerNode::robotTwistCallback, this);
        sub_path_ = nh_.subscribe<nav_msgs::Path>("navigation/path", 1, &ControllerNode::pathCallback, this);
    
        tick_timer_ = nh_.createTimer(ros::Duration(1.0 / freq), &ControllerNode::tickCallback, this);
        
        ROS_INFO("Controller node booted.");
    }

    void robotTwistCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        robot_velocity_.x = msg->linear.x * cos(robot_angle_);
        robot_velocity_.y = msg->linear.x * sin(robot_angle_);
        received_robot_twist_ = true;
    }

    void robotPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
    {
        robot_position_.x = msg->x;
        robot_position_.y = msg->y;
        robot_angle_ = msg->theta;
        received_robot_pose_ = true;

        pid_.setTruePosition(robot_position_);
        pid_.setTrueAngle(robot_angle_);
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

        if (path.size() < 2)
        {
            return ;
        }

        profiler_traj_.start();

        // update our trajectory
        traj_.update(path, robot_velocity_);

        nav_goal_ = path.back();
        num_waypoints_ = path.size();

        if (traj_.hasPath())
        {
            const math::SplinePathData pathData = traj_.getSplinePath()->toData();
            pub_traj_.publish(pathData);

            // ROS_INFO("Trajectory updated; remaining path length: %.4lf", traj_.getRemainingPathLength());
        }

        profiler_traj_.stop();
        profiler_traj_.print("trajectory fitting");
    }

    void tickCallback(const ros::TimerEvent& evt)
    {
        profiler_tick_.start();

        geometry_msgs::Twist twist;

        double linear_vel, angular_vel;

        Point2 pursuit_point;

        if (num_waypoints_ == 2 && (robot_position_ - nav_goal_).norm<double>() < 0.20)
        {
            pursuit_point = nav_goal_;
        }
        else if (traj_.hasPath())
        {
            pursuit_point = traj_.nextPoint(robot_position_);
        }
        else
        {
            pursuit_point = robot_position_;
        }
        
        // ROS_INFO("Next point x: %.3lf y: %.3lf", pursuit_point.x, pursuit_point.y);

        pid_.setTargetPosition(pursuit_point);
        pid_.update(linear_vel, angular_vel);

        linear_vel = std::max(0.0, linear_vel - 0.75 * angular_vel * angular_vel);

        twist.linear.x = linear_vel;
        twist.angular.z = angular_vel;

        geometry_msgs::Pose2D pursuit_point_pose;
        pursuit_point_pose.x = pursuit_point.x;
        pursuit_point_pose.y = pursuit_point.y;

        pub_pursuit_point_.publish(pursuit_point_pose);
        pub_cmd_vel_.publish(twist);

        if (traj_.hasPath())
        {
            vel_tracker_.update(linear_vel);
            vel_tracker_.print("velocity", "m/s");
        }
       

        profiler_tick_.stop();
        profiler_tick_.print("pid controller");
    }

protected:
    Trajectory traj_;
    PIDController pid_;

    int num_waypoints_;
    Point2 nav_goal_;

    Point2 robot_position_;
    double robot_angle_;
    bool received_robot_pose_;

    Point2 robot_velocity_;
    bool received_robot_twist_;

    ros::NodeHandle nh_;
    ros::Publisher pub_cmd_vel_;
    ros::Publisher pub_traj_;
    ros::Publisher pub_pursuit_point_;
    ros::Subscriber sub_robot_pose_;
    ros::Subscriber sub_robot_twist_;
    ros::Subscriber sub_path_;
    ros::Timer tick_timer_;

    Profiler profiler_tick_;
    Profiler profiler_traj_;

    MinMaxTracker vel_tracker_;

};

// Main function of the slam package
int main(int argc, char **argv){
  // Init Ros
  ros::init(argc,argv, "controller_node");
  ros::NodeHandle n;

  ControllerNode node;

  ros::spin();
  return 0;
}
