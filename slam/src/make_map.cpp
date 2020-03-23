#include <slam/make_map.h>
#include <math/line_of_sight.hpp>

LidarMap::LidarMap(Point2 mi_pos, Point2 max_pos, double c_size, double inf_radius):
    inflation_radius(inf_radius),
    min_pos(mi_pos),
    cell_size(c_size),received_robot_pose_(false)
{
    ros::NodeHandle nh;
    // Init subscribers and publishers
    scan_sub = nh.subscribe("scan", 10, &LidarMap::callback_scan, this);
    pos_sub = nh.subscribe("robot_pose", 1, &LidarMap::callback_pos, this);
    grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 5);

    width = ceil((max_pos.x-min_pos.x)/cell_size);
    height = ceil((max_pos.y-min_pos.y)/cell_size);

    // Initialize occupancy grid metadata
    occ_grid.info.resolution = cell_size;
    occ_grid.info.origin.position.x = min_pos.x;
    occ_grid.info.origin.position.y = min_pos.y;
    occ_grid.info.width = width;
    occ_grid.info.height = height;
    occ_grid.data.resize(width * height);

    std::fill(occ_grid.data.begin(),occ_grid.data.end(), CellState::unknown);

    // Generate relative coordinates to inflate
    mask_inflation = gen_mask(inflation_radius / cell_size);

    log_odds = new double[width * height];

    for (int k = 0; k < width * height; ++k)
        log_odds[k] = 0;

    inflation = new std::set<int>[width * height];
}

void lidar_map_line_of_sight_cb(const Index2& p, bool& stop, void *data)
{
    LidarMap *map = (LidarMap *)data;

    if (p == map->end_idx)
    {
        return ;
    }

    if(map->idx_in_map(p))
    {
        map->update_at_idx(p, false);
    }
    else
    {
        stop = true;
    }
}

void LidarMap::callback_scan(const sensor_msgs::LaserScan& msg)
{
    if (!received_robot_pose_)
    {
        return ;
    }

    Point2 robot_idx_no_round = pos2idx_no_round(robot_pos);
    Point2 end_idx_no_round, end_pos;

    profiler.start();

    // Scan is 360 degrees
    for(int i = 0; i < 360; i++)
    {
        Index2 end_idx;

        bool obstacle_found = msg.ranges[i] != INFINITY;
        double range = obstacle_found ? msg.ranges[i] : max_range;

        end_pos = inverseSensorModel(range, i);
        end_idx = pos2idx(end_pos);
        end_idx_no_round = pos2idx_no_round(end_pos);

        // find free cells between robot position and scan endpoint
        line_of_sight(robot_idx_no_round, end_idx_no_round, &lidar_map_line_of_sight_cb, this);

        if (idx_in_map(end_idx))
        {
            update_at_idx(end_idx, obstacle_found);
        }
    }

    // publish the new grid
    publish_grid();

    profiler.stop();
    profiler.print("mapping");
}

void LidarMap::callback_pos(const geometry_msgs::Pose2D& msg)
{
    robot_pos.x = msg.x;
    robot_pos.y = msg.y;
    robot_angle = msg.theta;
    robot_idx = pos2idx(robot_pos);
    received_robot_pose_ = true;
}


void LidarMap::publish_grid()
{
    grid_pub.publish(occ_grid);
}

void LidarMap::update_at_idx(Index2 idx, bool occupied)
{
    const int k = row_major_i(idx);
    double new_l, old_l = log_odds[k];

    new_l = std::clamp(old_l + (occupied ? l_occ : l_free), -l_max, l_max);

    log_odds[k] = new_l;
    occ_grid.data[k] = (uint8_t)get_state(idx);

    if (old_l < l_thresh && new_l > l_thresh)
    {
        add_inflation(idx, true);
    }
    else if (old_l > l_thresh && new_l < l_thresh)
    {
        add_inflation(idx, false);
    }
}

CellState LidarMap::get_state(const Index2 &idx)
{
    const int k = row_major_i(idx);

    if(log_odds[k] > l_thresh)
    {
        return CellState::occupied;
    }
    else if (!inflation[k].empty())
    {
        return CellState::inflated;
    }
    else
    {
        return log_odds[k] < -l_thresh ? CellState::empty : CellState::unknown;
    }
}

void LidarMap::add_inflation(Index2 center, bool occupied)
{
    const int center_k = row_major_i(center);
    int k;
    Index2 idx;

    for(int i = 0; i < mask_inflation.size(); i++)
    {
        idx = center + mask_inflation[i];

        if(idx_in_map(idx))
        {
            k = row_major_i(idx);

            if(occupied)
            {
                inflation[k].insert(center_k);
                occ_grid.data[k] = get_state(idx);
            }
            else
            {
                inflation[k].erase(center_k);
                occ_grid.data[k] = get_state(idx);
            }
        }
    }
}

std::vector<Index2> gen_mask(double r)
{
    std::vector<Index2> res;
    int ri = ceil(r);

    // Only look at first quadrant and then use symmetry for the rest, while not addiding twice
    for(int x = 0; x < ri; x++)
    {
        for(int y = 0;y < ri; y++)
        {
            if(x == 0 && y == 0)
            {
                continue;
            }

            if(std::sqrt((double)(x*x + y*y)) > r)
            {
                continue;
            }

            if(x != 0) res.push_back(Index2(-x, y));
            if(y != 0) res.push_back(Index2(x, -y));
            if(x != 0 && y != 0) res.push_back(Index2(-x, -y));

            res.push_back(Index2(x, y));
        }
    }

    return res;
}
