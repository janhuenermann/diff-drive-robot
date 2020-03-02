#ifndef THETASTAR_H
#define THETASTAR_H

/**
 * Author: Jan Huenermann <jan@huenermann.de>
 */

#include <global_planner/astar.hpp>
#include <global_planner/data_structures/math.hpp>

namespace ThetaStar
{

    typedef AStar::Node Node;

    // Implemented from https://en.wikipedia.org/wiki/Theta*
    class Search : public AStar::Search
    {
    public:
        Search(int w, int h) : AStar::Search(w, h) {}
    protected:
        virtual void resetNode(Node *n, Index2 end);
        virtual bool hasLineOfSight(Node *a, Node *b);
        virtual bool updateVertex(Node *node, Node *neighbor);

        inline bool isTraversableSwapped(Index2 i, bool axes_swapped)
        {
            if (axes_swapped)
                return grid_[i.x * width_ + i.y]->state == 0;
            else
                return grid_[i.y * width_ + i.x]->state == 0;
        };
    };

    // Implemented from https://www.aaai.org/ocs/index.php/AAAI/AAAI10/paper/view/1930/1945
    class LazySearch : public Search
    {
    public:
        LazySearch(int w, int h) : Search(w, h) {}
    protected:
        virtual bool updateVertex(Node *node, Node *neighbor);
        virtual bool setVertex(Node *node);
    };
};
// 


#endif