#ifndef THETASTAR_H
#define THETASTAR_H

/**
 * Author: Jan Huenermann <jan@huenermann.de>
 */

#include <math/vector2.hpp>
#include <global_planner/astar.hpp>

namespace ThetaStar
{

    typedef AStar::Node Node;

    // Implemented from https://en.wikipedia.org/wiki/Theta*
    class Search : public AStar::Search
    {
    public:
        Search(int w, int h) : AStar::Search(w, h) {}
    protected:
        virtual bool hasLineOfSight(Node *a, Node *b);
        virtual bool updateVertex(Node *node, Node *neighbor);

        inline bool isTraversableSwapped(Index2 i, bool axes_swapped)
        {
            if (axes_swapped)
                return occupancy_[i.x * width_ + i.y] == 0;
            else
                return occupancy_[i.y * width_ + i.x] == 0;
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
        LazySearch(int w, int h) : Search(w, h) {}
    protected:
        virtual bool updateVertex(Node *node, Node *neighbor);
        virtual bool setVertexShouldSkip(Node *node);
    };
};
// 


#endif