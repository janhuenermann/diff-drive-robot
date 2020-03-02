#include <gtest/gtest.h>
#include <global_planner/anya.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace ANYA;

TEST(ANYATest, testPathFindingSimple)
{
    int W = 5, H = 5;

    unsigned char *occupancy = new unsigned char[W * H];
    std::fill_n(occupancy, W*H, 0);

    for (int k = 0; k <= W-2; ++k)
    {
        int x = k;
        int y = (H-1)/2; // middle row

        occupancy[x + W * y] = 1;
    }

    Grid *grid = new Grid();
    grid->update(occupancy, W, H);

    std::cout << (*grid);

    Search *algo = new Search(grid);

    std::vector<Point2> path = algo->search(Point2(0, 0), Point2(0, H-1));

    EXPECT_EQ(path[0], Point2(0, 0));
    EXPECT_EQ(path[1], Point2(W-1, (H-1)/2));
    EXPECT_EQ(path[2], Point2(W-1, (H-1)/2+1));
    EXPECT_EQ(path[3], Point2(0, H-1));
}

TEST(ANYATest, testPathFindingHard)
{
    int m = 3;
    int W = 2*m+1;
    int H = W;

    unsigned char *occupancy = new unsigned char[W * H];
    std::fill_n(occupancy, W*H, 0);

    Mat frame = Mat::zeros(Size(W, W), CV_8UC3);

    for (int y = 1; y < W-1; y += 2)
    {
        const int l = (std::min(W - y, y)-1)/2;

        for (int x = m-l; x < m+1+l; ++x)
        {
            Vec3b & color = frame.at<Vec3b>(y, x);
            color[0] = 255;
            color[1] = 255;
            color[2] = 255;

            occupancy[x + W * y] = 1;
        }
    }

    Grid *grid = new Grid();
    grid->update(occupancy, W, H);

    std::cout << (*grid);

    Search *algo = new Search(grid);

    std::vector<Point2> path = algo->search(Point2(m, 0), Point2(m, W-1));

    // EXPECT_TRUE(path.size() > 0);

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

    // double pathlen = 0.0;

    // for (int i = 1; i < path.size(); ++i)
    // {
    //     pathlen += (path[i-1] - path[i]).norm<double>();
    // }

    // for (auto &n : path)
    // {
    //     Vec3b & color = frame.at<Vec3b>(n.y, n.x);
    //     color[0] = 128;
    //     color[1] = 255;
    //     color[2] = 0;
    // }

    // imwrite("some.png", frame);

    // std::cout << "Path length: " << pathlen << std::endl;
}