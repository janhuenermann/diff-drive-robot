#include <gtest/gtest.h>
#include <global_planner/astar.hpp>

TEST(AStarTest, testPathFinding)
{
    int W = 21;
    AStar *algo = new AStar(W, 3);

    for (int k = 0; k < W-1; ++k)
    {
        algo->getNodeAt(k, 1)->state = 1;
    }

    std::vector<PlanningNode *> path = algo->search(Index(0, 0), Index(0, 2));

    for (int k = 0; k < W-1; ++k)
    {
        EXPECT_EQ(path[k]->index, Index(k,0));
    }

    EXPECT_EQ(path[W-1]->index, Index(W-1, 1));

    for (int k = 0; k < W-1; ++k)
    {
        EXPECT_EQ(path[W+k]->index, Index(W-2-k, 2));
    }
}
