#include <gtest/gtest.h>
#include <global_planner/thetastar.hpp>


TEST(ThetaStarTest, testPathFindingSimple)
{
    int W = 5, H = 5;
    ThetaStar::LazySearch *algo = new ThetaStar::LazySearch(W, H);

    for (int k = 0; k < W-1; ++k)
    {
        algo->setOccupied(k, (H-1) / 2, true);
    }

    std::vector<Index2> path = algo->search(Index2(0, 0), Index2(0, H-1));

    EXPECT_EQ(path[0], Index2(0, 0));
    EXPECT_EQ(path[1], Index2(W-2, (H-1)/2-1));
    EXPECT_EQ(path[2], Index2(W-1, (H-1)/2));
    EXPECT_EQ(path[3], Index2(W-2, (H-1)/2+1));
    EXPECT_EQ(path[4], Index2(0, H-1));
}

TEST(ThetaStarTest, testPathFindingHard)
{
    int m = 20;
    int W = 2*m+1;

    // Mat frame = Mat::zeros(Size(W, W), CV_8UC3);
    ThetaStar::LazySearch *algo = new ThetaStar::LazySearch(W, W);

    for (int k = 1; k < W-1; k += 2)
    {
        int l = (std::min(W - k, k)-1)/2;
        for (int j = m-l; j < m+1+l; ++j)
        {
            // Vec3b & color = frame.at<Vec3b>(k, j);
            // color[0] = 255;
            // color[1] = 255;
            // color[2] = 255;

            algo->setOccupied(j, k, true);
        }
    }

    std::vector<Index2> path = algo->search(Index2(m, 0), Index2(m, W-1));

    EXPECT_TRUE(path.size() > 0);

    // for (int i = 0; i < W; ++i)
    // {
    //     for (int j = 0; j < W; ++j)
    //     {
    //         if (algo->getNodeAt(i, j)->visited && algo->getNodeAt(i, j)->parent != nullptr)
    //         {
    //             Vec3b & color = frame.at<Vec3b>(j, i);
    //             color[0] = algo->getNodeAt(i, j)->parent->index.y * 30 % 255;
    //             color[1] = 0;
    //             color[2] = 0;
    //         }
    //     }
    // }

    double pathlen = 0.0;

    for (int i = 1; i < path.size(); ++i)
    {
        pathlen += (path[i-1] - path[i]).norm<double>();
    }

    // for (auto &n : path)
    // {
    //     Vec3b & color = frame.at<Vec3b>(n.y, n.x);
    //     color[0] = 128;
    //     color[1] = 255;
    //     color[2] = 0;
    // }

    // imwrite("some.png", frame);

    std::cout << "Path length: " << pathlen << std::endl;
}