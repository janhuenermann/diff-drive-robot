#ifndef THETASTAR_H
#define THETASTAR_H

/**
 * Author: Jan Huenermann <jan@huenermann.de>
 */

#include <global_planner/astar.hpp>

// Implemented from https://en.wikipedia.org/wiki/Theta*
class ThetaStarSearch : public AStarSearch
{
public:
    ThetaStarSearch(int w, int h) : AStarSearch(w, h) {}
protected:
    void resetNode(Node *n, Index end);
    bool hasLineOfSight(Node *a, Node *b);

    virtual bool computeCost(Node *node, Node *neighbor, const Distance &d);

    inline bool isTraversable(Index i, bool axes_swapped)
    {
        if (axes_swapped)
            return grid_[i.x() * width_ + i.y()]->state == 0;
        else
            return grid_[i.y() * width_ + i.x()]->state == 0;
    };
};

// Implemented from https://www.aaai.org/ocs/index.php/AAAI/AAAI10/paper/view/1930/1945
class LazyThetaStarSearch : public ThetaStarSearch
{
public:
    LazyThetaStarSearch(int w, int h) : ThetaStarSearch(w, h) {}
protected:
    bool computeCost(Node *node, Node *neighbor, const Distance &d);
    void setVertex(Node *node);
};


#endif