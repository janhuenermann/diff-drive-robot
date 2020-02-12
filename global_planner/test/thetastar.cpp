#include <gtest/gtest.h>
#include <global_planner/thetastar.hpp>

TEST(ThetaStarTest, testPathFindingSimple)
{
    int W = 5, H = 5;
    LazyThetaStarSearch *algo = new LazyThetaStarSearch(W, H);

    for (int k = 0; k < W-1; ++k)
    {
        algo->getNodeAt(k, (H-1) / 2)->state = 1;
    }

    std::vector<LazyThetaStarSearch::Node *> path = algo->search(Index(0, 0), Index(0, H-1));

    EXPECT_EQ(path[0]->index, Index(0, 0));
    EXPECT_EQ(path[1]->index, Index(W-2, (H-1)/2-1));
    EXPECT_EQ(path[2]->index, Index(W-1, (H-1)/2));
    EXPECT_EQ(path[3]->index, Index(W-2, (H-1)/2+1));
    EXPECT_EQ(path[4]->index, Index(0, H-1));
}

TEST(ThetaStarTest, testPathFindingHard)
{
    int m = 250;
    int W = 2*m+1;

    LazyThetaStarSearch *algo = new LazyThetaStarSearch(W, W);

    for (int k = 1; k < W-1; k += 2)
    {
        int l = (std::min(W - k, k)-1)/2;

        for (int j = m-1-l; j < m+l; ++j)
        {
            algo->getNodeAt(j, k)->state = 1;
        }
    }

    std::vector<LazyThetaStarSearch::Node *> path = algo->search(Index(m, 0), Index(m, W-1));

    EXPECT_TRUE(path.size() > 0);

    // for (auto &n : path)
    // {
    //     std::cout << n->index.x() << "," << n->index.y() << std::endl;
    // }
}