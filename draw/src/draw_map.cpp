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
#include <math/util.hpp>

using namespace cv;
typedef Point3_<uint8_t> Pixel;

const int PX_CELL_SIZE = 3;
const std::string WINDOW_NAME = "map";

class DrawMapNode : public ParallelLoopBody
{
public:
    DrawMapNode() :
        nh_(), frame_(),
        received_pose_(false), received_map_(false), received_path_(false), received_nav_goal_(false),
        received_pursuit_point_(false),
        trajectory_(nullptr)
    {
        const double freq = 20.0;

        sub_map_ = nh_.subscribe<nav_msgs::OccupancyGrid>("map", 1, &DrawMapNode::mapCallback, this);
        sub_robot_pose_ = nh_.subscribe<geometry_msgs::Pose2D>("robot_pose", 1, &DrawMapNode::robotPoseCallback, this);
        sub_goal_ = nh_.subscribe<geometry_msgs::Pose2D>("navigation/goal", 1, &DrawMapNode::goalCallback, this);
        sub_path_ = nh_.subscribe<nav_msgs::Path>("navigation/path", 1, &DrawMapNode::navigationCallback, this);
        sub_pursuit_point_ = nh_.subscribe<geometry_msgs::Pose2D>("navigation/pursuit_point", 1, &DrawMapNode::pursuitPointCallback, this);
        sub_traj_ = nh_.subscribe<math::SplinePathData>("navigation/trajectory", 1, &DrawMapNode::trajectoryCallback, this);

        tick_timer_ = nh_.createTimer(ros::Duration(1.0 / freq), &DrawMapNode::tickCallback, this);

        ROS_INFO("Booting draw map node");
        namedWindow(WINDOW_NAME, WINDOW_NORMAL);
    }

    void resizeWind()
    {
        const double max_side = 768;
        double out_w, out_h;
        double ar = (double)frame_.rows / (double)frame_.cols;

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

        resizeWindow(WINDOW_NAME, (int)out_w, (int)out_h);
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        map_ = *msg;

        if (frame_.cols != map_.info.width || frame_.rows != map_.info.height)
        {
            frame_.create(map_.info.height * PX_CELL_SIZE, map_.info.width * PX_CELL_SIZE, CV_8UC3);
            frame_data_ = (uint8_t *)frame_.data;

            resizeWind();
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
        pursuit_point_ = *msg;
        received_pursuit_point_ = true;
    }

    void tickCallback(const ros::TimerEvent& evt)
    {
        if (!received_map_)
        {
            return ;
        }

        profiler_.start();
        draw();
        profiler_.stop();
        profiler_.print("drawing");

        imshow(WINDOW_NAME, frame_);
        waitKey(1);
    }

    void operator () (const cv::Range &r) const
    {
        const int mw = map_.info.width;
        const int cw = PX_CELL_SIZE * 3;
        const int mcw = cw * mw;

        uint8_t color;

        unsigned char* rgb0;
        unsigned char* rgb = const_cast<unsigned char *>(frame_.ptr(r.start));

        const signed char* state = map_.data.data() + r.start/PX_CELL_SIZE*mw;
        int y, k, l, x;

        for (y = r.start; y != r.end; )
        {
            rgb0 = rgb;

            for (x = 0; x != mw; ++x, ++state)
            {
                if (*state == 0) // free
                {
                    color = 0;
                }
                else if (*state == 1) // occupied
                {
                    color = 255;
                }
                else if (*state == 2) // inflated
                {
                    color = 128;
                }
                else if (*state == -1) // unknown
                {
                    color = 64;
                }

                memset(rgb, color, cw);
                rgb += cw;
            }

            ++y;
            
            // copy line
            for (k = 1; k != PX_CELL_SIZE && y != r.end; ++y, ++k, rgb += mcw)
                memcpy(rgb, rgb0, mcw);
        }
    }

    void draw()
    {
        // draw map
        cv::parallel_for_(cv::Range(0, frame_.rows), *this);

        // draw trajectory
        if (trajectory_ != nullptr)
        {
            drawTrajectory();
        }

        // draw waypoints
        if (received_path_ && planned_path_.poses.size() > 1)
        {
            for (int k = 1; k < planned_path_.poses.size()-1; ++k)
            {
                geometry_msgs::PoseStamped& pose_stamped = planned_path_.poses[k];
                drawCell(toCVPoint(pose_stamped.pose.position), 0, 0, 255);
            }
        }

        // draw pursuit points
        if (received_pursuit_point_)
        {
            drawCircle(toCVPoint(pursuit_point_), 4, 0, 255, 255);
        }

        // draw pose
        if (received_pose_)
        {
            drawRobotPose(map_.info.resolution * 5);
        }

        // draw goal
        if (received_nav_goal_)
        {
            drawCircle(toCVPoint(nav_goal_), 7, 0, 255, 0);
        }
    }

    void drawRobotPose(double size)
    {
        const double w = size / 10.0;
        const double len = size;
        cv::Point vertices[4];

        Point2 center = Point2(robot_pose_.x, robot_pose_.y);
        Point2 a = Point2(0, len).rotate(robot_pose_.theta);
        Point2 b = Point2(len, 0).rotate(robot_pose_.theta);

        cv::line(frame_, toCVPoint(center - a), toCVPoint(center + a), Scalar(255, 0, 0), 2);
        cv::arrowedLine(frame_, toCVPoint(center - b), toCVPoint(center + b), Scalar(255, 0, 0), 2, CV_AA, 0, 0.3);
    }

    void drawTrajectory()
    {
        if (trajectory_ == nullptr)
        {
            return ;
        }

        const double resolution = 10.0;
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

            drawCell(toCVPoint(p), 0, 0, 255);

            if (s >= trajectory_->length)
            {
                break ;
            }

            s = std::min(s + step_size, trajectory_->length);
        }
    }

    inline void setPixel(const int &x, const int &y, const uint8_t &r, const uint8_t &g, const uint8_t &b)
    {
        unsigned char *data = frame_data_ + 3 * (y * frame_.cols + x);
        *(data++) = r;
        *(data++) = g;
        *(data++) = b;
    }

    inline void setPixel(const cv::Point p, const uint8_t r, const uint8_t g, const uint8_t b)
    {
        setPixel(p.x, p.y, r, g, b);
    }

    inline void drawCell(cv::Point p, uint8_t r, uint8_t g, uint8_t b)
    {
        unsigned char *data;

        for (int j = 0; j<PX_CELL_SIZE; ++j)
        {
            data = frame_data_ + 3 * ((p.y+j) * frame_.cols + p.x);

            for (int i = 0; i<PX_CELL_SIZE; ++i)
            {
                *(data++) = r;
                *(data++) = g;
                *(data++) = b;
            }
        }
    }

    inline void drawCell(cv::Point p, uint8_t l)
    {
        drawCell(p, l, l, l);
    }

    template<class T>
    inline cv::Point toCVPoint(T p)
    {
        return toCVPoint(Point2(p.x, p.y));
    }

    inline cv::Point toCVPoint(int x, int y)
    {
        return toCVPoint(Point2(x, y));
    }

    inline cv::Point toCVPoint(Point2 p)
    {
        return Point((int)std::round((double)PX_CELL_SIZE * (p.x - map_.info.origin.position.x) / map_.info.resolution), 
                     (int)std::round((double)PX_CELL_SIZE * (p.y - map_.info.origin.position.y) / map_.info.resolution));
    }

    inline void drawCircle(cv::Point p, int radius, uint8_t r, uint8_t g, uint8_t b)
    {
        
        circle(frame_, p, radius, Scalar(r, g, b), FILLED);
    }


protected:

    Mat frame_;
    Mat output_;

    uint8_t *frame_data_;

    SplinePath *trajectory_;

    geometry_msgs::Pose2D nav_goal_;
    bool received_nav_goal_;

    geometry_msgs::Pose2D robot_pose_;
    bool received_pose_;

    nav_msgs::OccupancyGrid map_;
    bool received_map_;

    nav_msgs::Path planned_path_;
    bool received_path_;

    geometry_msgs::Pose2D pursuit_point_;
    bool received_pursuit_point_;

    ros::NodeHandle nh_;
    ros::Subscriber sub_goal_;
    ros::Subscriber sub_map_;
    ros::Subscriber sub_path_;
    ros::Subscriber sub_pursuit_point_;
    ros::Subscriber sub_robot_pose_;
    ros::Subscriber sub_traj_;

    ros::Timer tick_timer_;

    Profiler profiler_;


};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "draw_map_cpp");

    DrawMapNode node;
    ros::spin();
}
