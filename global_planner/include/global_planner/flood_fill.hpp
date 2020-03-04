#ifndef FLOOD_FILL_H
#define FLOOD_FILL_H

/**
 * Author: Jan Huenermann <jan@huenermann.de>
 */

#include <vector>
#include <iostream>

#include <math/vector2.hpp>

#include <global_planner/queue/fibonacci_queue.hpp>
#include <global_planner/priority_grid.hpp>

class FloodFill
{
public:
    FloodFill(NodeGrid *grid);

    std::vector<Index2> search(Index2 start);

protected:

    inline bool isTarget(Node *n)
    {
        return isTarget(n->index);
    }

    inline bool isTarget(Index2 i)
    {
        return grid_->occupancy[i.y * grid_->width + i.x] == 0;
    }

    virtual void findSuccessors(Node *node);
    virtual bool updateVertexHasLowerCost(Node *node, Node *neighbor);

    NodeGrid *grid_;
    FibonacciQueue<Cost>::Container<Node> queue_;
};

#endif
