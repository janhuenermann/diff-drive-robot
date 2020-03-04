#ifndef ASTAR_H
#define ASTAR_H

/**
 * Author: Jan Huenermann <jan@huenermann.de>
 */

#include <vector>
#include <iostream>

#include <math/vector2.hpp>

#include <global_planner/queue/fibonacci_queue.hpp>
#include <global_planner/priority_grid.hpp>

namespace AStar
{

    class Search
    {
    public:
        Search(NodeGrid *grid);

        std::vector<Index2> search(Index2 start, Index2 target);

        inline bool isTraversable(Node *n)
        {
            return isTraversable(n->index);
        }

        inline bool isTraversable(Index2 i)
        {
            if (i.x < 0 || i.x >= grid_->width || i.y < 0 || i.y >= grid_->height)
            {
                return false;
            }

            return grid_->occupancy[i.y * grid_->width + i.x] == 0;
        }

    protected:


        virtual void findSuccessors(Node *node, Index2 target);
        virtual bool setVertexShouldSkip(Node *node) { return false; };
        virtual bool shouldPrune(Node *a, Node *b, Node *c) { return false; };

        /**
         * Updates the vertex. 
         * @param Node
         * @param Neighbor node
         * @param Distance of neighbor to node.
         * @return True if the G-cost has decreased.
         */
        virtual bool updateVertexHasLowerCost(Node *node, Node *neighbor);

        /**
         * Checks if two nodes represent a vertex with lower cost
         * than previously discovered.
         * Vertex from node to neighbor.
         * @param  node     The start node of the vertex.
         * @param  neighbor The end node of the vertex, for which the parent should be updated.
         * @return          True if lower G-cost.
         */
        virtual bool relax(Node *node, Node *neighbor);

        NodeGrid *grid_;
        FibonacciQueue<Cost>::Container<Node> queue_;
    };
}

#endif
