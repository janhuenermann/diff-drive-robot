#include <iostream>
#include <chrono>

#include <math/util.hpp>
#include <global_planner/thetastar.hpp>

void run_profile1(const int m, const int N)
{
    Profiler profiler;

    const int W = 2*m+1;

    NodeGrid grid(W, W);
    ThetaStar::LazySearch algo(&grid);

    for (int k = 1; k < W-1; k += 2)
    {
        int l = (std::min(W - k, k)-1)/2;
        
        for (int j = m-l; j < m+1+l; ++j)
        {
            grid.setOccupied(j, k, true);
        }
    }

    profiler.start();

    for (int k; k < N; ++k)
    {
        std::vector<Index2> path = algo.search(Index2(m, 0), Index2(m, W-1));
        if (path.size() == 0)
        {
            std::cout << "Test failed. No path found." << std::endl;
            return ;
        }
    }

    profiler.stop();

    double T = profiler.getTimeTaken() / (double)N;
    std::cout << "Lazy-Theta* took " << T << "ms on average for m = " << m << "." << std::endl;
}


int main(int argc, char** argv)
{
    run_profile1(10, 1000);
    run_profile1(100, 100);
    run_profile1(1000, 10);

    return 0;
}
