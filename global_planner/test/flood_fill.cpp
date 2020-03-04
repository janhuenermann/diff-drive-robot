#include <gtest/gtest.h>

#include <global_planner/flood_fill.hpp>


TEST(FloodFillTest, testFloodFillSimple)
{
    const int W = 15;
    const int innerW = 3;
    const int mid = (W-1)/2;
    const int offset = mid-(innerW-1)/2;

    NodeGrid grid(W, W);

    FloodFill *algo = new FloodFill(&grid);

    for (int y = -1; y < innerW + 1; ++y)
    {
        for (int x = 0; x < innerW + 1; ++x)
        {
            grid.setOccupied(offset + x, offset + y, true);
        }
    }

    std::vector<Index2> path = algo->search(Index2(mid, mid));

    for (int k = 0; k < innerW; ++k)
    {
        EXPECT_EQ(path[k], Index2(mid - k, mid));
    }
}