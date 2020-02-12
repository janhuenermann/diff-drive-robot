#include <gtest/gtest.h>
#include <global_planner/astar.hpp>

TEST(AStarTest, testPathFindingSimple)
{
    int W = 21;
    AStarSearch *algo = new AStarSearch(W, 3);

    for (int k = 0; k < W-1; ++k)
    {
        algo->getNodeAt(k, 1)->state = 1;
    }

    std::vector<AStarSearch::Node *> path = algo->search(Index(0, 0), Index(0, 2));

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

TEST(AStarTest, testPathFindingHard)
{
    int m = 10;
    int W = 2*m+1;

    AStarSearch *algo = new AStarSearch(W, W);

    for (int k = 1; k < W-1; k += 2)
    {
        int l = (std::min(W - k, k)-1)/2;

        for (int j = m-1-l; j < m+l; ++j)
        {
            algo->getNodeAt(j, k)->state = 1;
        }
    }

    std::vector<AStarSearch::Node *> path = algo->search(Index(m, 0), Index(m, W-1));

    EXPECT_TRUE(path.size() > 0);
}
