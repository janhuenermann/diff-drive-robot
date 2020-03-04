#include <ros/ros.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <global_planner/priority_grid.hpp>
#include <global_planner/thetastar.hpp>
#include <global_planner/flood_fill.hpp>

#include <math/util.hpp>

template<class T>
class PlannerNode
{
public:

    PlannerNode() :
        nh_(), 
        grid_(),
        algorithm_(new T(&grid_)),
        flood_fill_(new FloodFill(&grid_)),
        received_map_(false),
        received_robot_pose_(false),
        received_nav_goal_(false)
    {
        const double freq = 20.0;

        sub_occ_grid_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &PlannerNode::mapCallback, this);
        sub_robot_pose_ = nh_.subscribe<geometry_msgs::Pose2D>("/robot_pose", 1, &PlannerNode::robotPoseCallback, this);
        sub_goal_ = nh_.subscribe<geometry_msgs::Pose2D>("/navigation/goal", 1, &PlannerNode::goalCallback, this);
        pub_path_ = nh_.advertise<nav_msgs::Path>("/navigation/path", 10);

        tick_timer_ = nh_.createTimer(ros::Duration(1.0 / freq), &PlannerNode::tickCallback, this);

        ROS_INFO("Booted planner node");
    }

    Index2 getIndexFromPosition(Point2 pt)
    {
        return ((pt - map_origin_) / map_resolution_).cast<int>();
    }

    Point2 getPositionFromIndex(Index2 idx)
    {
        return idx.cast<double>() * map_resolution_ + map_origin_;
    }

    void findPath()
    {
        if (!received_map_ || !received_nav_goal_ || !received_robot_pose_)
        {
            return ;
        }

        Index2 start = getIndexFromPosition(robot_position_);
        Index2 target = getIndexFromPosition(goal_);

        Index2 start_traversable = start;
        Index2 target_traversable = target;

        if (!algorithm_->isTraversable(start_traversable))
        {
            std::vector<Index2> path_to_start = flood_fill_->search(start);
            start_traversable = path_to_start.back();
        }

        if (!algorithm_->isTraversable(target_traversable))
        {
            std::vector<Index2> path_to_target = flood_fill_->search(target);
            target_traversable = path_to_target.back();
        }

        path_ = algorithm_->search(start_traversable, target_traversable);

        if (start != start_traversable)
        {
            path_.insert(path_.begin(), start);
        }

        if (target != target_traversable)
        {
            path_.push_back(target);
        }

        publishPath();
    }

    void publishPath()
    {
        nav_msgs::Path msg;
        msg.header.stamp = last_update_stamp_;

        for (Index2& point : path_)
        {
            Point2 world_point = getPositionFromIndex(point);

            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.pose.position.x = world_point.x;
            pose_stamped.pose.position.y = world_point.y;

            msg.poses.push_back(pose_stamped);
        }

        pub_path_.publish(msg);

        // ROS_INFO("Published path with %d way-points!", (int)path_.size());
    }

    void updateMap(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        int w = msg->info.width,
            h = msg->info.height;

        // resize map if necessary
        if (grid_.width != w || grid_.height != h)
        {
            grid_.resize(w, h);
        }

        map_resolution_ = (double)msg->info.resolution;
        map_origin_ = Point2(msg->info.origin.position.x, msg->info.origin.position.y);

        for (int j = 0; j < h; ++j)
        {
            for (int i = 0; i < w; ++i)
            {
                grid_.occupancy[i + j * w] = msg->data[i + j * w] > 0 ? 1 : 0; // row-major order
            }
        }
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        last_update_stamp_ = msg->header.stamp;
        updateMap(msg);
        received_map_ = true;
    }

    void robotPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
    {
        last_update_stamp_ = ros::Time::now();
        robot_position_ = Point2(msg->x, msg->y);
        robot_angle_ = msg->theta;
        received_robot_pose_ = true;
    }

    void goalCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
    {
        last_update_stamp_ = ros::Time::now();
        goal_ = Point2(msg->x, msg->y);
        received_nav_goal_ = true;
    }

    void tickCallback(const ros::TimerEvent& evt)
    {
        profiler_.start();
        findPath();
        profiler_.stop();
        profiler_.print("path finding");
    }

protected:

    double map_resolution_;

    NodeGrid grid_;
    T* algorithm_;
    FloodFill *flood_fill_;

    bool received_map_;
    bool received_robot_pose_;
    bool received_nav_goal_;

    Point2 map_origin_;
    Point2 robot_position_;
    double robot_angle_;

    Point2 goal_;

    ros::NodeHandle nh_;
    ros::Publisher pub_path_;
    ros::Subscriber sub_goal_;
    ros::Subscriber sub_occ_grid_;
    ros::Subscriber sub_robot_pose_;

    ros::Timer tick_timer_;

    std::vector<Index2> path_;

    ros::Time last_update_stamp_;

    Profiler profiler_;

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_planner");

    PlannerNode<ThetaStar::LazySearch> node;
    ros::spin();
}