#ifndef ASTAR_H
#define ASTAR_H

/**
 * Author: Jan Huenermann <jan@huenermann.de>
 */

#include <vector>
#include <iostream>

#include <global_planner/data_structures/fibonacci_queue.hpp>
#include <global_planner/data_structures/point.hpp>

class AStarSearch
{
public:
    struct Node : public FibonacciQueue<Distance>::Element
    {
        Index2 index;
        int state; // 0 - free, 1 - occupied, 2 - inflated
        Distance g, h, f;
        bool visited;
        Node *parent;

        Node(int x, int y) :
            Element(),
            index(Index2(x, y)),
            g(0,0), h(0,0), f(0,0),
            visited(false), parent(nullptr), state(0)
        {
        }

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

    AStarSearch(int width, int height);

    void resize(int width, int height);
    std::vector<Node *> search(Index2 start, Index2 end);

    inline Node*& getNodeAt(Index2 i)
    {
        return getNodeAt(i.x, i.y);
    }

    inline Node*& getNodeAt(int x, int y)
    {
        return grid_[y * width_ + x];
    }

    inline bool isTraversable(Node *n)
    {
        return n->state == 0;
    }

    inline bool isTraversable(Index2 i)
    {
        if (i.x < 0 || i.x >= width_ || i.y < 0 || i.y >= height_)
        {
            return false;
        }

        return isTraversable(getNodeAt(i));
    }

    inline int getWidth()
    {
        return width_;
    }

    inline int getHeight()
    {
        return height_;
    }

protected:
    virtual void resetNode(Node *node, Index2 goal);
    virtual void findSuccessors(Node *node);
    virtual void setVertex(Node *node) {};

    /**
     * Updates the cost. 
     * @param Node
     * @param Neighbor node
     * @param Distance of neighbor to node.
     * @return True if the cost has decreased.
     */
    virtual bool computeCost(Node *node, Node *neighbor, const Distance &d);

    Node **grid_;
    FibonacciQueue<Distance>::Container<Node> queue_;

    int width_, height_;
};

#endif
