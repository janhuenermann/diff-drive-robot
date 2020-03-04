#ifndef GRID_H
#define GRID_H

#include <math/vector2.hpp>

#include <global_planner/queue/fibonacci_queue.hpp>

struct Cost
{
    double g, h, f;

    Cost() : g(0), h(0), f(0)
    {}

    Cost(double g, double h) : g(g), h(h), f(g+h)
    {}

    Cost operator-() const {
        return Cost(-g, -h);
    }

    inline void setGCost(double _g)
    {
        g = _g;
        f = g + h;
    }

    inline void setHCost(double _h)
    {
        h = _h;
        f = g + h;
    }

    inline void reset()
    {
        setHCost(0);
        setGCost(std::numeric_limits<double>::infinity());
    }

    inline void reset(Index2 pos, Index2 target)
    {
        setHCost((pos - target).norm<double>());
        setGCost(std::numeric_limits<double>::infinity());
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

    inline void reset(int _run)
    {
        visited = false;
        parent = nullptr;
        run = _run;
        cost.reset();
    }

    inline void reset(Index2 target, int _run)
    {
        visited = false;
        parent = nullptr;
        run = _run;
        cost.reset(index, target);
    };
};

struct NodeGrid
{

    NodeGrid(int w, int h) : nodes(nullptr), occupancy(nullptr), width(0), height(0), run(0)
    {
        if (w != 0 && h != 0)
        {
            resize(w, h);
        }
    }

    NodeGrid() : NodeGrid(0,0)
    {}

    Node **nodes;
    uint8_t *occupancy;

    int width;
    int height;

    int run;

    void resize(int w, int h);

    inline Node*& getNodeAt(Index2 i)
    {
        return getNodeAt(i.x, i.y);
    }

    inline Node*& getNodeAt(int x, int y)
    {
        return nodes[y * width + x];
    }

    inline void setOccupied(int x, int y, bool occupied)
    {
        occupancy[y * width + x] = occupied ? 1 : 0;
    }
};

namespace std {
    template<> class numeric_limits<Cost> {
    public:
       static Cost infinity() { return Cost(numeric_limits<double>::infinity(), 0); };
    };
}

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

#endif