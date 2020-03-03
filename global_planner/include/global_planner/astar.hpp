#ifndef ASTAR_H
#define ASTAR_H

/**
 * Author: Jan Huenermann <jan@huenermann.de>
 */

#include <vector>
#include <iostream>

#include <math/vector2.hpp>
#include <global_planner/queue/fibonacci_queue.hpp>

namespace AStar
{
    struct Cost
    {
        double g, h, f;

        Cost() : g(0), h(0), f(0)
        {}

        Cost(double g, double h) : g(g), h(h), f(g+h)
        {}

        void setGCost(double d)
        {
            g = d; 
            f = g + h;
        };

        void setHCost(double d)
        {
            h = d;
            f = g + h;
        };

        Cost operator-() const {
            return Cost(-g, -h);
        }

        inline void reset(Index2 pos, Index2 target)
        {
            h = (pos - target).norm<double>(); // Distance::octileDistance(cur, goal);
            g = std::numeric_limits<double>::infinity();
            f = std::numeric_limits<double>::infinity();
        }
    };

    struct Node : public FibonacciQueue<Cost>::Element
    {
        Index2 index;

        Cost cost;
        bool visited;
        Node *parent;
        int run;

        Node(int x, int y) :
            Element(),
            index(Index2(x, y)),
            cost(),
            visited(false), parent(nullptr),
            run(0)
        {}

        inline void reset(Index2 target, int _run)
        {
            visited = false;
            parent = nullptr;
            run = _run;
            cost.reset(index, target);
        };
    };

    class Search
    {
    public:
        Search(int width, int height);

        void resize(int width, int height);
        std::vector<Index2> search(Index2 start, Index2 target);

        inline int getWidth()
        {
            return width_;
        }

        inline int getHeight()
        {
            return height_;
        }

        inline Node*& getNodeAt(Index2 i)
        {
            return getNodeAt(i.x, i.y);
        }

        inline Node*& getNodeAt(int x, int y)
        {
            return grid_[y * width_ + x];
        }

        inline uint8_t *getOccupancyData()
        {
            return occupancy_;
        }

        inline void swapOccupancyData(uint8_t *data)
        {
            occupancy_ = data;
        }

        inline void setOccupied(int x, int y, bool occupied)
        {
            occupancy_[y * width_ + x] = occupied ? 1 : 0;
        }

    protected:

        inline bool isTraversable(Node *n)
        {
            return isTraversable(n->index);
        }

        inline bool isTraversable(Index2 i)
        {
            if (i.x < 0 || i.x >= width_ || i.y < 0 || i.y >= height_)
            {
                return false;
            }

            return occupancy_[i.y * width_ + i.x] == 0;
        }

        void introduce(Node *node);

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
        virtual bool updateVertex(Node *node, Node *neighbor);

        /**
         * Checks if two nodes represent a vertex with lower cost
         * than previously discovered.
         * Vertex from node to neighbor.
         * @param  node     The start node of the vertex.
         * @param  neighbor The end node of the vertex, for which the parent should be updated.
         * @return          True if lower G-cost.
         */
        virtual bool relax(Node *node, Node *neighbor);

        uint8_t *occupancy_;

        Node **grid_;
        FibonacciQueue<Cost>::Container<Node> queue_;

        int width_, height_;

        Index2 target_;
        int current_run_;
    };

    inline bool operator <(const Cost& a, const Cost& b)
    {
        if (a.f == b.f)
        {
            return a.g > b.g;
        }

        return a.f < b.f;
    }

    inline bool operator ==(const Cost& a, const Cost& b)
    {
        return a.f == b.f;
    }
}

namespace std {
    template<> class numeric_limits<AStar::Cost> {
    public:
       static AStar::Cost infinity() { return AStar::Cost(numeric_limits<double>::infinity(), 0); };
    };
}

#endif
