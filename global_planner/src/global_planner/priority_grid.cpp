#include <global_planner/priority_grid.hpp>

void NodeGrid::resize(int w, int h)
{
    if (w == width && h == height)
    {
        return ;
    }

    if (nodes != nullptr)
    {
        delete [] nodes;
    }

    if (occupancy != nullptr)
    {
        free(occupancy);
        occupancy = nullptr;
    }

    occupancy = (uint8_t *)calloc(w * h, sizeof(uint8_t));
    nodes = new Node*[w * h];

    for (int y = 0; y < h; ++y)
    {
        for (int x = 0; x < w; ++x)
        {
            nodes[y * w + x] = new Node(x, y);
        }
    }

    width = w;
    height = h;

    run = 0;
}