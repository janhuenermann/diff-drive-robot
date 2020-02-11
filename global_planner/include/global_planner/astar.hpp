#ifndef ASTAR_H
#define ASTAR_H

#include <vector>

#include <global_planner/data_structures/fibonacci_queue.hpp>
#include <global_planner/data_structures/distance.hpp>

struct PlanningNode
{
    Index index;
    int state; // 0 - free, 1 - occupied, 2 - inflated
    Distance g, h, f;
    bool visited;
    PlanningNode *parent;

    PlanningNode();

    void setGCost(Distance d)
    {
        g = d;
        f = g + h;
    };

    void setHCost(Distance d)
    {
        h = d;
        f = g + h;
    };
};

class AStar
{
public:
    AStar(int width, int height);

    void resize(int width, int height);
    std::vector<PlanningNode *> search(Index start, Index end);

protected:
    PlanningNode& getNodeAt(Index i);
    PlanningNode& getNodeAt(int x, int y);
    void findSuccessors(PlanningNode *node);

    PlanningNode *grid_;
    FibonacciQueue<Distance, PlanningNode> queue_;

    int width_, height_;
};

#endif
