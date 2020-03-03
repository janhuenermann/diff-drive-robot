#include <global_planner/astar.hpp>

#include <stack>
#include "ros/ros.h"

using namespace AStar;

Search::Search(int width, int height) :
    grid_(nullptr),
    width_(0),
    height_(0),
    occupancy_(nullptr),
    current_run_(0)
{
    resize(width, height);
}

void Search::resize(int width, int height)
{
    if (grid_ != nullptr)
    {
        delete [] grid_;
    }

    grid_ = new Node*[width * height];
    width_ = width;
    height_ = height;

    for (int j = 0; j < height; ++j)
    {
        for (int i = 0; i < width; ++i)
        {
            getNodeAt(i, j) = new Node(i, j);
        }
    }

    if (occupancy_ != nullptr)
    {
        free(occupancy_);
        occupancy_ = nullptr;
    }

    if (width != 0 && height != 0)
    {
        occupancy_ = (uint8_t *)calloc(width * height, sizeof(uint8_t));
    }
}

std::vector<Index2> Search::search(Index2 start, Index2 target)
{
    std::vector<Index2> path;

    if (!isTraversable(start) || !isTraversable(target))
    {
        return path;
    }

    current_run_ += 1; // unique id for this run

    Node*& start_node = getNodeAt(start);
    start_node->reset(target, current_run_);
    start_node->cost.setGCost(0);

    queue_.clear();
    queue_.insert(start_node->cost, start_node);

    while (!queue_.empty())
    {
        Node *node = queue_.pop();

        if (node->visited)
        {
            continue ;
        }

        if (setVertexShouldSkip(node))
        {
            continue ;
        }

        if (node->index == target)
        {
            std::stack<Node *> st;
            Node *last = nullptr;

            while (node != nullptr)
            {
                if (last == nullptr || node->parent == nullptr || !shouldPrune(node->parent, node, last))
                {
                    st.push(node);
                    last = node;
                }
                
                node = node->parent;
            }

            while (!st.empty())
            {
                path.push_back(st.top()->index);
                st.pop();
            }

            break ;
        }

        node->visited = true;

        findSuccessors(node, target);
    }

    return path;
}

void Search::findSuccessors(Node *node, Index2 target)
{
    bool lower_cost;
    int bx = node->index.x, by = node->index.y;

    for (int i = -1; i <= 1; ++i)
    {
        for (int j = -1; j <= 1; ++j)
        {
            if ((i == 0 && j == 0) ||
                    bx+i < 0 || bx+i >= width_ || by+j < 0 || by+j >= height_)
            {
                continue ;
            }


            Node*& neighbor = getNodeAt(bx+i, by+j);

            if (neighbor->run != current_run_)
            {
                neighbor->reset(target, current_run_);
            }

            if (!isTraversable(neighbor) || neighbor->visited)
            {
                continue ;
            }

            // dead corners
            if (i != 0 && j != 0 && !isTraversable(Index2(bx+i, by)) && !isTraversable(Index2(bx, by+j)))
            {
                continue ;
            }

            lower_cost = updateVertex(node, neighbor);

            if (lower_cost)
            {
                queue_.upsert(neighbor->cost, neighbor, true);
            }
        }
    }
}

bool Search::updateVertex(Node *node, Node *neighbor)
{
    return relax(node, neighbor);
}

bool Search::relax(Node *node, Node *neighbor)
{
    double tentative_g = node->cost.g + (node->index - neighbor->index).norm<double>();

    if (tentative_g < neighbor->cost.g)
    {
        neighbor->cost.setGCost(tentative_g);
        neighbor->parent = node;

        return true;
    }

    return false;
}