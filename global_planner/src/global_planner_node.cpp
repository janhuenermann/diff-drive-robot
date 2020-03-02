#include <ros/ros.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <global_planner/thetastar.hpp>

template<class T>
class PlannerNode
{
public:

    PlannerNode() : nh_(), algorithm_(new T(0, 0))
    {
        sub_occ_grid_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &PlannerNode::mapCallback, this);
        sub_robot_pose_ = nh_.subscribe<geometry_msgs::Pose2D>("/robot_pose", 1, &PlannerNode::robotPoseCallback, this);
        sub_goal_ = nh_.subscribe<geometry_msgs::Pose2D>("/navigation/goal", 1, &PlannerNode::goalCallback, this);
        pub_path_ = nh_.advertise<nav_msgs::Path>("/navigation/path", 10);
    }

    void findPath()
    {
        Index2 start = (robot_position_ / map_resolution_).cast<int>();
        Index2 target = (goal_ / map_resolution_).cast<int>();

        path_ = algorithm_->search(start, target);

        publishPath();
    }

    void publishPath()
    {
        nav_msgs::Path msg;
        msg.header.stamp = last_update_stamp_;

        for (Index2& point : path_)
        {
            Point2 world_point = point.cast<double>() * map_resolution_;

            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.pose.position.x = world_point.x;
            pose_stamped.pose.position.y = world_point.y;

            msg.poses.push_back(pose_stamped);
        }

        pub_path_.publish(msg);
    }

    void updateMap(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        int w = msg->info.width,
            h = msg->info.height;

        // resize map if necessary
        if (algorithm_->getWidth() < w
            || algorithm_->getHeight() < h)
        {
            algorithm_->resize(msg->info.width, msg->info.height);
        }

        map_resolution_ = (double)msg->info.resolution;

        // copy data
        for (int j = 0; j < h; ++j)
        {
            for (int i = 0; i < w; ++i)
            {
                bool is_free_or_unknown = msg->data[i + j * w] <= 0;
                algorithm_->getNodeAt(i, j)->traversable = is_free_or_unknown; // row-major order
            }
        }
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        last_update_stamp_ = msg->header.stamp;
        updateMap(msg);
        findPath();
    }

    void robotPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
    {
        last_update_stamp_ = ros::Time::now();
        robot_position_ = Point2(msg->x, msg->y);
        robot_angle_ = msg->theta;
        findPath();
    }

    void goalCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
    {
        last_update_stamp_ = ros::Time::now();
        goal_ = Point2(msg->x, msg->y);
        findPath();
    }

protected:

    double map_resolution_;
    T* algorithm_;

    Point2 robot_position_;
    double robot_angle_;

    Point2 goal_;

    ros::NodeHandle nh_;
    ros::Publisher pub_path_;
    ros::Subscriber sub_goal_;
    ros::Subscriber sub_occ_grid_;
    ros::Subscriber sub_robot_pose_;

    std::vector<Index2> path_;

    ros::Time last_update_stamp_;

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_planner");

    PlannerNode<ThetaStar::LazySearch> node;
    ros::spin();
}