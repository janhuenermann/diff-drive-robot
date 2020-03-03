#include <gtest/gtest.h>
#include <global_planner/astar.hpp>

TEST(AStarTest, testPathFindingSimple)
{
    int W = 21;
    AStar::Search *algo = new AStar::Search(W, 3);

    for (int k = 0; k < W-1; ++k)
    {
        algo->setOccupied(k, 1, true);
    }

    std::vector<Index2> path = algo->search(Index2(0, 0), Index2(0, 2));

    for (int k = 0; k < W-1; ++k)
    {
        EXPECT_EQ(path[k], Index2(k,0));
    }

    EXPECT_EQ(path[W-1], Index2(W-1, 1));

    for (int k = 0; k < W-1; ++k)
    {
        EXPECT_EQ(path[W+k], Index2(W-2-k, 2));
    }
}

TEST(AStarTest, testPathFindingHard)
{
    int m = 10;
    int W = 2*m+1;

    AStar::Search *algo = new AStar::Search(W, W);

    for (int k = 1; k < W-1; k += 2)
    {
        int l = (std::min(W - k, k)-1)/2;

        for (int j = m-1-l; j < m+l; ++j)
        {
            algo->setOccupied(j, k, true);
        }
    }

    std::vector<Index2> path = algo->search(Index2(m, 0), Index2(m, W-1));

    EXPECT_TRUE(path.size() > 0);
}
