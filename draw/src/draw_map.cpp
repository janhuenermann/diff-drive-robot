#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>

#include <math/vector2.hpp>
#include <math/SplinePathData.h>
#include <math/spline.hpp>

using namespace cv;

const int PX_CELL_SIZE = 3;

class DrawMapNode
{
public:
    DrawMapNode() :
        nh_(), frame_(),
        received_pose_(false), received_map_(false), received_path_(false), received_nav_goal_(false),
        received_pursuit_point_(false),
        trajectory_(nullptr)
    {
        const double freq = 5.0;

        sub_map_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &DrawMapNode::mapCallback, this);
        sub_robot_pose_ = nh_.subscribe<geometry_msgs::Pose2D>("/robot_pose", 1, &DrawMapNode::robotPoseCallback, this);
        sub_goal_ = nh_.subscribe<geometry_msgs::Pose2D>("/navigation/goal", 1, &DrawMapNode::goalCallback, this);
        sub_path_ = nh_.subscribe<nav_msgs::Path>("/navigation/path", 1, &DrawMapNode::navigationCallback, this);
        sub_pursuit_point_ = nh_.subscribe<geometry_msgs::Pose2D>("/navigation/pursuit_point", 1, &DrawMapNode::pursuitPointCallback, this);
        sub_traj_ = nh_.subscribe<math::SplinePathData>("/navigation/trajectory", 1, &DrawMapNode::trajectoryCallback, this);

        tick_timer_ = nh_.createTimer(ros::Duration(1.0 / freq), &DrawMapNode::tickCallback, this);

        ROS_INFO("Booting draw map node");
        namedWindow("Map", WINDOW_NORMAL);
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        map_ = *msg;

        if (frame_.cols != map_.info.width || frame_.rows != map_.info.height)
        {
            frame_.create(map_.info.height * PX_CELL_SIZE, map_.info.width * PX_CELL_SIZE, CV_8UC3);
        }

        received_map_ = true;
    }

    void robotPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
    {
        robot_pose_ = *msg;
        received_pose_ = true;
    }

    void goalCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
    {
        nav_goal_ = *msg;
        received_nav_goal_ = true;
    }

    void navigationCallback(const nav_msgs::Path::ConstPtr& msg)
    {
        planned_path_ = *msg;
        received_path_ = true;
    }

    void trajectoryCallback(const math::SplinePathData::ConstPtr& msg)
    {
        if (trajectory_ != nullptr)
        {
            delete trajectory_;
        }

        trajectory_ = SplinePath::fromData(*msg);
    }

    void pursuitPointCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
    {
        pursuit_point_.x = msg->x;
        pursuit_point_.y = msg->y;
        received_pursuit_point_ = true;
    }

    void tickCallback(const ros::TimerEvent& evt)
    {
        draw();
    }

    void draw()
    {
        if (!received_map_)
        {
            return ;
        }

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

        if (trajectory_ != nullptr)
        {
            drawTrajectory();
        }

        if (received_path_ && planned_path_.poses.size() > 1)
        {
            for (int k = 1; k < planned_path_.poses.size()-1; ++k)
            {
                geometry_msgs::PoseStamped& pose_stamped = planned_path_.poses[k];
                setColor(pose_stamped.pose.position, 0, 0, 255);
            }
        }

        if (received_pursuit_point_)
        {
            drawCircle(pursuit_point_, 5, 0, 255, 255);
        }

        if (received_pose_)
        {
            drawCircle(Point2(robot_pose_.x, robot_pose_.y), 7, 255, 0, 0);
        }

        if (received_nav_goal_)
        {
            drawCircle(Point2(nav_goal_.x, nav_goal_.y), 7, 0, 255, 0);
        }

        double ar = (double)frame_.rows / (double)frame_.cols;

        const double max_side = 768;
        double out_w, out_h;

        if (ar > 1)
        {
            out_w = max_side / ar;
            out_h = max_side;
        }
        else
        {
            out_w = max_side;
            out_h = max_side * ar;
        }

        cv::resize(frame_, output_, cv::Size((int)out_w, (int)out_h), 0, 0, CV_INTER_LINEAR);

        imshow("Map", output_);
        waitKey(1);
    }

    void drawTrajectory()
    {
        if (trajectory_ == nullptr)
        {
            return ;
        }

        const double resolution = 100.0;
        const double step_size = 1.0 / resolution;

        double s = 0.0;
        Point2 p;

        while (1)
        {
            try
            {
                p = trajectory_->position(s);
            }
            catch (const std::exception& e)
            {
                break ;
            }

            setPixel(p, 255, 0, 255);

            if (s >= trajectory_->length)
            {
                break ;
            }

            s = std::min(s + step_size, trajectory_->length);
        }
    }

    inline void setPixel(int x, int y, uint8_t r, uint8_t g, uint8_t b)
    {
        Vec3b &v = frame_.at<Vec3b>(y, x);
        v[0] = r;
        v[1] = g;
        v[2] = b;
    }

    inline void setPixel(Point2 pt, uint8_t r, uint8_t g, uint8_t b)
    {
        setPixel(
            (int)std::round((pt.x - map_.info.origin.position.x) / map_.info.resolution * (double)PX_CELL_SIZE), 
            (int)std::round((pt.y - map_.info.origin.position.y) / map_.info.resolution * (double)PX_CELL_SIZE),
            r, g, b);
    }

    inline void setColor(int x, int y, uint8_t r, uint8_t g, uint8_t b)
    {
        if (x < 0 || y < 0 || x >= map_.info.width || y >= map_.info.height)
        {
            return ;
        }

        Rect rect(3 * x, 3 * y, PX_CELL_SIZE, PX_CELL_SIZE);
        cv::rectangle(frame_, rect, Scalar(r, g, b), FILLED);
    }

    inline void setColor(int x, int y, uint8_t l)
    {
        setColor(x, y, l, l, l);
    }

    inline void setColor(geometry_msgs::Pose2D pose, uint8_t r, uint8_t g, uint8_t b)
    {
        setColor(
            (int)((pose.x - map_.info.origin.position.x) / map_.info.resolution), 
            (int)((pose.y - map_.info.origin.position.y) / map_.info.resolution),
            r, g, b);
    }

    inline void setColor(geometry_msgs::Point pt, uint8_t r, uint8_t g, uint8_t b)
    {
        setColor(
            (int)((pt.x - map_.info.origin.position.x) / map_.info.resolution), 
            (int)((pt.y - map_.info.origin.position.y) / map_.info.resolution),
            r, g, b);
    }

    inline Point toCVPoint(Point2 p)
    {
        return Point((int)std::round(3.0 * (p.x - map_.info.origin.position.x) / map_.info.resolution), 
                     (int)std::round(3.0 * (p.y - map_.info.origin.position.y) / map_.info.resolution));
    }

    inline void drawCircle(Point2 p, int radius, uint8_t r, uint8_t g, uint8_t b)
    {
        
        circle(frame_, toCVPoint(p), radius, Scalar(r, g, b), FILLED);
    }


protected:

    Mat frame_;
    Mat output_;

    SplinePath *trajectory_;

    geometry_msgs::Pose2D nav_goal_;
    bool received_nav_goal_;

    geometry_msgs::Pose2D robot_pose_;
    bool received_pose_;

    nav_msgs::OccupancyGrid map_;
    bool received_map_;

    nav_msgs::Path planned_path_;
    bool received_path_;

    Point2 pursuit_point_;
    bool received_pursuit_point_;

    ros::NodeHandle nh_;
    ros::Subscriber sub_goal_;
    ros::Subscriber sub_map_;
    ros::Subscriber sub_path_;
    ros::Subscriber sub_pursuit_point_;
    ros::Subscriber sub_robot_pose_;
    ros::Subscriber sub_traj_;

    ros::Timer tick_timer_;


};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "draw_map_cpp");

    DrawMapNode node;
    ros::spin();
}
