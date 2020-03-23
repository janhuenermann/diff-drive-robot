#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <ros/console.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>

#include <math/vector2.hpp>
#include <math/SplinePathData.h>
#include <math/spline.hpp>
#include <math/util.hpp>

#define row_major_i(idx) (idx.y() * map_.info.width + idx.x())

const bool CROP_TO_SPLINE = true;

class LatexWriter
{
public:

    LatexWriter() : nh_(ros::NodeHandle("~")), trajectory_(nullptr)
    {
        if (!nh_.getParam("file_path", out_fp_))
        {
            ROS_ERROR("Could not find output file path.");
            return ;
        }

        sub_map_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &LatexWriter::mapCallback, this);
        sub_robot_pose_ = nh_.subscribe<geometry_msgs::Pose2D>("/robot_pose", 1, &LatexWriter::robotPoseCallback, this);
        sub_goal_ = nh_.subscribe<geometry_msgs::Pose2D>("/navigation/goal", 1, &LatexWriter::goalCallback, this);
        sub_path_ = nh_.subscribe<nav_msgs::Path>("/navigation/path", 1, &LatexWriter::navigationCallback, this);
        sub_pursuit_point_ = nh_.subscribe<geometry_msgs::Pose2D>("/navigation/pursuit_point", 1, &LatexWriter::pursuitPointCallback, this);
        sub_traj_ = nh_.subscribe<math::SplinePathData>("/navigation/trajectory", 1, &LatexWriter::trajectoryCallback, this);
    
        tick_timer_ = nh_.createTimer(ros::Duration(1.0), &LatexWriter::tickCallback, this);

        ROS_INFO("Booted");
    }


    void write()
    {
        if (!received_map_)
        {
            ROS_INFO("Did not receive map");

            return ;
        }

        std::ofstream out(out_fp_, std::ofstream::out | std::ofstream::trunc);

        const Vector2<double> size = CROP_TO_SPLINE && received_path_ ?
            max_path_ - min_path_ : Vector2<double>((double)map_.info.width, (double)map_.info.height);
        
        const Index2 offset = min_path_.floor().cast<int>();
        const Vector2<int> sizei = size.ceil().cast<int>();

        out << "\\pgfmathsetmacro\\scale{0.15};" << std::endl;
        out << "\\pgfmathsetmacro\\mapw{" << sizei.x() << "};" << std::endl;
        out << "\\pgfmathsetmacro\\maph{" << sizei.y() << "};" << std::endl;

        out << "\\begin{scope}[scale=\\scale, transform shape,rotate=90]" << std::endl;
        out << "\\draw [step=2, very thin, gray] (0,0) grid (\\mapw, \\maph);" << std::endl;
        out << "\\fill [inflated] " << std::endl;

        ROS_INFO("offset %d %d, size %d %d", offset.x(), offset.y(), sizei.x(), sizei.y());

        for (int y = 0; y < sizei.y(); ++y)
        {
            for (int x = 0; x < sizei.x(); ++x)
            {
                const Index2 i(x,y);
                const Index2 j = CROP_TO_SPLINE ? i + offset : i;

                if (map_.data[row_major_i(j)] == 2)
                {
                    out << "(" << x << ", " << y << ") rectangle ++(1, 1)" << std::endl;
                }
            }
        }

        out << ";" << std::endl;
        out << "\\fill [occupied] " << std::endl;

        for (int y = 0; y < sizei.y(); ++y)
        {
            for (int x = 0; x < sizei.x(); ++x)
            {
                const Index2 i(x,y);
                const Index2 j = CROP_TO_SPLINE ? i + offset : i;

                if (map_.data[row_major_i(j)] == 1)
                {
                    out << "(" << x << ", " << y << ") rectangle ++(1, 1)" << std::endl;
                }
            }
        }

        out << ";" << std::endl;

        if (received_path_)
        {
            std::string path_str;
            std::string dots_str;

            dots_str += "\\pgfmathsetmacro\\dotsradius{1}\n";
            dots_str += "\\fill [red] ";
            path_str += "\\draw [red, line width = 4pt] plot [smooth, tension=0.5] coordinates { ";

            for (int k = 0; k < planned_path_.poses.size(); ++k)
            {
                geometry_msgs::PoseStamped& pose_stamped = planned_path_.poses[k];
                Point2 p = toGridPoint(Point2(pose_stamped.pose.position.x, pose_stamped.pose.position.y));

                if (CROP_TO_SPLINE)
                {
                    p -= offset.cast<double>();
                }

                dots_str += "(" + std::to_string(p.x()) + "," + std::to_string(p.y()) + ") circle (\\dotsradius) ";
                path_str += "(" + std::to_string(p.x()) + "," + std::to_string(p.y()) + ") ";
            }

            path_str += "};\n";
            dots_str += ";\n";

            out << dots_str;
            out << path_str;
        }

        if (trajectory_ != nullptr)
        {
            const double resolution = 16.0;
            const double step_size = 1.0 / resolution;

            double s = 0.0;
            Point2 p;

            out << "\\draw [blue, line width = 2pt] plot [] coordinates { " << std::endl;

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

                p = toGridPoint(p);

                if (CROP_TO_SPLINE)
                {
                    p -= offset.cast<double>();
                }

                out << "(" << p.x() << "," << p.y() << ") ";

                if (s >= trajectory_->length)
                {
                    break ;
                }

                s = std::min(s + step_size, trajectory_->length);
            }

            out << "};" << std::endl;
        }

        out << "\\end{scope}" << std::endl;
        out.close();


        ROS_INFO("Wrote tikz file");
    }

    Point2 toGridPoint(Point2 pt)
    {
        return Point2((pt.x() - map_.info.origin.position.x), 
                      (pt.y() - map_.info.origin.position.y)) / (double)map_.info.resolution;
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        map_ = *msg;
        received_map_ = true;
        ROS_INFO("Received map");
    }

    void robotPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
    {
        robot_pos_.x() = msg->x;
        robot_pos_.y() = msg->y;
        robot_angle_ = msg->theta;
        received_pose_ = true;
    }

    void goalCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
    {
        nav_goal_ = *msg;
        received_nav_goal_ = true;
    }

    void navigationCallback(const nav_msgs::Path::ConstPtr& msg)
    {
        if (!received_map_)
        {
            return ;
        }

        planned_path_ = *msg;

        const Point2 grid_minp = Point2(0,0);
        const Point2 grid_maxp = Point2((double)map_.info.width - 1.0, (double)map_.info.height - 1.0);

        Point2 minp, maxp;

        minp.x() = std::numeric_limits<double>::infinity();
        minp.y() = std::numeric_limits<double>::infinity();

        maxp.x() = -std::numeric_limits<double>::infinity();
        maxp.y() = -std::numeric_limits<double>::infinity();

        const double expand = 3.0;

        for (auto &pose_stamped : planned_path_.poses)
        {
            Point2 gp = toGridPoint(Point2(pose_stamped.pose.position.x, pose_stamped.pose.position.y));
            minp.x() = std::min(minp.x(), gp.x() - expand);
            minp.y() = std::min(minp.y(), gp.y() - expand);
            maxp.x() = std::max(maxp.x(), gp.x() + expand);
            maxp.y() = std::max(maxp.y(), gp.y() + expand);
        }

        min_path_.x() = std::max(grid_minp.x(), minp.x());
        min_path_.y() = std::max(grid_minp.y(), minp.y());

        max_path_.x() = std::min(grid_maxp.x(), maxp.x());
        max_path_.y() = std::min(grid_maxp.y(), maxp.y());

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
        ROS_INFO("Tick");
        profiler_.start();
        write();
        profiler_.stop();
        profiler_.print("latex writer");
    }

private:

    ros::NodeHandle nh_;
    ros::Subscriber sub_goal_;
    ros::Subscriber sub_map_;
    ros::Subscriber sub_path_;
    ros::Subscriber sub_pursuit_point_;
    ros::Subscriber sub_robot_pose_;
    ros::Subscriber sub_traj_;

    ros::Timer tick_timer_;
    std::string out_fp_;

    SplinePath *trajectory_;

    Profiler profiler_;

    geometry_msgs::Pose2D nav_goal_;
    bool received_nav_goal_;

    double robot_angle_;
    Point2 robot_pos_;
    bool received_pose_;

    nav_msgs::OccupancyGrid map_;
    bool received_map_;

    nav_msgs::Path planned_path_;
    bool received_path_;

    geometry_msgs::Pose2D pursuit_point_;
    bool received_pursuit_point_;

    Point2 min_path_;
    Point2 max_path_;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "write_latex");

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
        ros::console::notifyLoggerLevelsChanged();

    ROS_INFO("HEY WAZ UP");

    LatexWriter writer;
    ros::spin();
}