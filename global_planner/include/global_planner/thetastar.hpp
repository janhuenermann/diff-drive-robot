#ifndef THETASTAR_H
#define THETASTAR_H

/**
 * Author: Jan Huenermann <jan@huenermann.de>
 */

#include <math/vector2.hpp>

#include <global_planner/priority_grid.hpp>
#include <global_planner/astar.hpp>

namespace ThetaStar
{

    // Implemented from https://en.wikipedia.org/wiki/Theta*
    class Search : public AStar::Search
    {
    public:
        using AStar::Search::Search;
    protected:
        virtual bool hasLineOfSight(Node *a, Node *b);
        virtual bool updateVertexHasLowerCost(Node *node, Node *neighbor);

        inline bool isTraversableSwapped(Index2 i, bool axes_swapped)
        {
            if (axes_swapped)
                return grid_->occupancy[i.x * grid_->width + i.y] == 0;
            else
                return grid_->occupancy[i.y * grid_->width + i.x] == 0;
        };

        virtual bool shouldPrune(Node *a, Node *b, Node *c)
        {
            return hasLineOfSight(a, c);
        };
    };

    // Implemented from https://www.aaai.org/ocs/index.php/AAAI/AAAI10/paper/view/1930/1945
    class LazySearch : public Search
    {
    public:
        using Search::Search;
    protected:
        virtual bool updateVertexHasLowerCost(Node *node, Node *neighbor);
        virtual bool setVertexShouldSkip(Node *node);
    };
};
// 


#endif