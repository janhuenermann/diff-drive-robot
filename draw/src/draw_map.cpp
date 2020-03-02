#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>

#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>

using namespace cv;

class DrawMapNode
{
public:
    DrawMapNode() : nh_(), frame_()
    {
        sub_map_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &DrawMapNode::mapCallback, this);
        sub_robot_pose_ = nh_.subscribe<geometry_msgs::Pose2D>("/robot_pose", 1, &DrawMapNode::robotPoseCallback, this);
        sub_goal_ = nh_.subscribe<geometry_msgs::Pose2D>("/navigation/goal", 1, &DrawMapNode::goalCallback, this);
        sub_path_ = nh_.subscribe<nav_msgs::Path>("/navigation/path", 1, &DrawMapNode::navigationCallback, this);

        namedWindow("Map", WINDOW_AUTOSIZE);
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        map_ = *msg;

        if (frame_.cols != map_.info.width || frame_.rows != map_.info.height)
        {
            frame_.create(map_.info.height, map_.info.width, CV_8UC3);
        }

        draw();
    }

    void robotPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
    {
        robot_pose_ = *msg;
        draw();
    }

    void goalCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
    {
        nav_goal_ = *msg;
        draw();
    }

    void navigationCallback(const nav_msgs::Path::ConstPtr& msg)
    {
        planned_path_ = *msg;
        draw();
    }

    void draw()
    {
        const int w = map_.info.width;
        const int h = map_.info.height;
        const float res = map_.info.resolution;
        
        for (int y = 0; y < h; ++y)
        {
            for (int x = 0; x < w; ++x)
            {
                const int state = map_.data[y * w + x];

                switch (state)
                {
                    case 0: // free
                        setColor(x, y, 0);
                        break ;
                    case 1: // occupied
                        setColor(x, y, 255);
                        break ;
                    case 2: // inflated
                        setColor(x, y, 128);
                        break ;

                    case -1: // unknown
                    default:
                        setColor(x, y, 64);
                        break ;
                }
            }
        }

        setColor(robot_pose_, 255, 0, 0);
        setColor(nav_goal_, 0, 255, 0);

        assert(planned_path_.poses.size() >= 2); // start and end point

        for (int k = 1; k < planned_path_.poses.size()-1; ++k)
        {
            geometry_msgs::PoseStamped& pose_stamped = planned_path_.poses[k];
            setColor(pose_stamped.pose.position, 0, 0, 255);
        }

        imshow("Map", frame_);
    }

    inline void setColor(int x, int y, uint8_t r, uint8_t g, uint8_t b)
    {
        Vec3b &color = frame_.at<Vec3b>(y, x);
        color[0] = r;
        color[1] = g;
        color[2] = b;
    }

    inline void setColor(int x, int y, uint8_t l)
    {
        setColor(x, y, l, l, l);
    }

    inline void setColor(geometry_msgs::Pose2D pose, uint8_t r, uint8_t g, uint8_t b)
    {
        setColor((int)(pose.x / map_.info.resolution), (int)(pose.y / map_.info.resolution), r, g, b);
    }

    inline void setColor(geometry_msgs::Point pt, uint8_t r, uint8_t g, uint8_t b)
    {
        setColor((int)(pt.x / map_.info.resolution), (int)(pt.y / map_.info.resolution), r, g, b);
    }


protected:

    Mat frame_;

    geometry_msgs::Pose2D nav_goal_;
    geometry_msgs::Pose2D robot_pose_;
    nav_msgs::OccupancyGrid map_;
    nav_msgs::Path planned_path_;

    ros::NodeHandle nh_;
    ros::Subscriber sub_goal_;
    ros::Subscriber sub_map_;
    ros::Subscriber sub_path_;
    ros::Subscriber sub_robot_pose_;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "draw_map_cpp");

    DrawMapNode node;
    ros::spin();
}