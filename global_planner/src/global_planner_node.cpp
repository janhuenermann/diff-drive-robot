#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <global_planner/thetastar.hpp>


class PlannerNode
{
public:

    PlannerNode() : nh_(), algorithm_(new LazyThetaStarSearch(0, 0))
    {
        sub_occ_grid_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &PlannerNode::mapCallback, this);
    }

    void spin()
    {
        ros::spin();
    }

    void findPath()
    {}

    void publishPath()
    {}

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

        // copy data
        for (int j = 0; j < h; ++j)
        {
            for (int i = 0; i < w; ++i)
            {
                algorithm_->getNodeAt(i, j)->state = msg->data[i + j * w]; // row-major order
            }
        }
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        updateMap(msg);
        findPath();
        publishPath();
    }

protected:
    LazyThetaStarSearch* algorithm_;

    ros::NodeHandle nh_;
    ros::Subscriber sub_occ_grid_;

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_planner");

    PlannerNode node;
    node.spin();
}