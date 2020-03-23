#include <ros/ros.h>

#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>

#include <vector>
#include <set>
#include <algorithm>

#include <math/vector2.hpp>
#include <math/util.hpp>

#define row_major_i(idx) (idx.y * width + idx.x)

enum CellState
{
    unknown = -1,
    empty = 0,
    occupied = 1,
    inflated = 2
};

class LidarMap
{
private:
    ros::Subscriber scan_sub;       // subscribe to wheel angle and robot position
    ros::Subscriber pos_sub;
    ros::Publisher grid_pub;        // map publisher

    double max_range = 3.5;          // maximal range of LIDAR
    double inflation_radius;         // radius added to obstacle to avoid robot touching them
    
    // log odds parameters
    double l_occ = 0.1;
    double l_free = -0.02;
    double l_thresh = 0.149;
    double l_max = 1.5;
    
    // Map info
    Point2 min_pos;

    double cell_size;
    int width;
    int height;

    double robot_angle;
    Point2 robot_pos;
    Index2 robot_idx;
    bool received_robot_pose_;

    nav_msgs::OccupancyGrid occ_grid;   // published map
    double *log_odds;
    std::set<int> *inflation;    // holds inflation info

    std::vector<Index2> mask_inflation; // holds relative positions of to be inflated cells
    Profiler profiler;

    /**
     * Returns the state of the cell at the given index
     * @param  idx Index of cell
     * @return     Cell state
     */
    CellState get_state(const Index2 &idx);

    /**
     * Takes output of lidar and gives world position of the end of the laser ray
     * @param  range Distance to obstacle, or max range if no obstacle
     * @param  deg   Angle of laser measurement
     * @return       Position of the end point of measurement in world coordinates
     */
    inline Point2 inverseSensorModel(double range, double deg)
    {
        return robot_pos + range * Point2::unitCircle(robot_angle + deg * (M_PI/180));
    }

public:
    LidarMap(Point2 mi_pos, Point2 max_pos, double c_size, double inf_radius);
    ~LidarMap()
    {
        delete [] inflation;
        delete [] log_odds;
    }

    /**
     * Converts World coordinates to map indices
     * @param  pos World coordinate in form (x,y)
     * @return     Index in map
     */
    inline Index2 pos2idx(Point2 pos)
    {
        return ((pos - min_pos) / cell_size).round();
    }

    /**
     * Converts World coordinates to map indices but does not round for integer indices
     * @param  pos World coordinate in form (x,y)
     * @return     Index in map
     */
    inline Point2 pos2idx_no_round(Point2 pos)
    {
        return (pos - min_pos) / cell_size;
    }

    /**
     * Converts map indices to world coordinates
     * @param  idx Index in map
     * @return     Position of the cell in world coordinates
     */
    inline Point2 idx2pos(Index2 idx)
    {
        return idx.cast<double>() * cell_size + min_pos;
    }

    /**
     * Gets information from new LIDAR scan
     * @param msg Output of laser scan
     */
    void callback_scan(const sensor_msgs::LaserScan& msg);
    void callback_pos(const geometry_msgs::Pose2D& msg);

    /**
     * Publishes the calculated Occupancy grid
     */
    void publish_grid();

    /**
     * Checks if an index is inside the bounds of the map
     * @param  idx Map index to be checked
     * @return     true if index is in bounds, false otherwise
     */
    inline bool idx_in_map(Index2 idx)
    {
        return idx.x >= 0 && idx.y >= 0 && idx.x < occ_grid.info.width && idx.y < occ_grid.info.height;
    };

    /**
     * Update log odds and occupancy grid with new found information
     * @param idx      index to update
     * @param occupied true if the cell is beliefed to be occupide, false else
     */
    void update_at_idx(Index2 idx, bool occupied);

    /**
     * Marks cells to be inflated, when a change happens at center
     * @param center   The changed cell
     * @param occupied true if the cell is newly occupied, false if cell is newly free
     */
    void add_inflation(Index2 center,bool occupied);
    
    Index2 end_idx;

};

std::vector<Index2> gen_mask(double r);
