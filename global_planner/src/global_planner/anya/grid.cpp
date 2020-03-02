#include <global_planner/anya.hpp>

using namespace ANYA;

Grid::Grid() : 
    occupancy_(nullptr),
    point_data_(nullptr),
    width_(-1), height_(-1)
{
    update(nullptr, 0, 0);
}

void Grid::resize(int w, int h)
{
    if (occupancy_ != nullptr)
        delete [] occupancy_;

    if (point_data_ != nullptr)
        delete [] point_data_;

    width_ = w;
    height_ = h;

    padded_width_ = w + 2;
    padded_height_ = h + 2;

    occupancy_ = new unsigned char[padded_width_ * padded_height_];
    point_data_ = new long[(w + 1) * (h + 1)]; // lattice includes end points [w, h], excludes outer points (edge of padding)

    intersection_step_size = 1.0 / (double)std::max(padded_width_, padded_height_);
    intersection_step_size_div2 = intersection_step_size / 2;
}

int Grid::scanCellsLeft(int x, int y) const
{
    assert(x >= -1 && x <= width_);
    assert(y >= -1 && y <= height_);

    int k, l;
    for (k = x, l = getOccupancyIndex(k - 1, y); k >= 0; --k, --l)
    {
        if (occupancy_[l])
        {
            return k;
        }
    }

    std::unexpected();
}

int Grid::scanCellsRight(int x, int y) const
{
    assert(x >= -1 && x <= width_);
    assert(y >= -1 && y <= height_);

    int k, l;
    for (k = x, l = getOccupancyIndex(k, y); k <= width_; ++k, ++l)
    {
        if (occupancy_[l])
        {
            return k;
        }
    }

    std::unexpected();
}

int Grid::scanLatticeLeft(double x, int y) const
{
    assert(x >= 0 && x <= (double)width_);
    assert(y >= 0 && y <= (double)height_);

    int k, m, l;
    for (k = (int)(x), 
         l = getPointIndex(k, y),
         m = getOccupancyIndex(k - 1, y); k >= 0; --k, --l, --m)
    {
        if (point_data_[l] & POINT_CORNER || occupancy_[m])
        {
            return k;
        }
    }

    std::unexpected();
}

int Grid::scanLatticeRight(double x, int y) const
{
    assert(x >= 0 && (int)x <= width_);
    assert(y >= 0 && (int)y <= height_);

    int k, m, l;
    for (k = (int)(x), 
         l = getPointIndex(k, y),
         m = getOccupancyIndex(k, y); k <= width_; ++k, ++l, ++m)
    {
        if (point_data_[l] & POINT_CORNER || occupancy_[m])
        {
            return k;
        }
    }

    std::unexpected();
}

void Grid::update(unsigned char *occupancy, int w, int h)
{
    if (w != width_ || h != height_)
    {
        resize(w, h);
    }

    long data;
    bool ne, nw, se, sw;
    bool double_corner, corner, visible;

    for (int y = -1; y < h + 1; ++y)
    {
        for (int x = -1; x < w + 1; ++x)
        {
            if (x >= 0 && y >= 0)
            {
                // true if traversable
                // north-east, south-east, south-west, ...
                ne = y > 0 && x < w && !occupancy[(y-1) * w + x];
                se = y < h && x < w && !occupancy[y * w + x];
                sw = y < h && x > 0 && !occupancy[y * w + x-1];
                nw = y > 0 && x > 0 && !occupancy[(y-1) * w + x-1];

                double_corner = (!nw && !se && sw && ne)
                             || (!ne && !sw && se && nw);

                corner =  ((!nw || !se) && sw && ne)
                       || ((!ne || !sw) && se && nw);

                visible = ne || nw || sw || se;

                data = 0;
                data |= (visible ? POINT_VISIBLE : 0);
                data |= (corner ? POINT_CORNER : 0);
                data |= (double_corner ? POINT_DOUBLE_CORNER : 0);

                getPointData(x, y) = data;
            }
            
            if (x >= 0 && y >= 0 && x < w && y < h)
            {
                getOccupancy(x, y) = occupancy[y * w + x];
            }
            else
            {
                getOccupancy(x, y) = 1;
            }
        }
    }

}