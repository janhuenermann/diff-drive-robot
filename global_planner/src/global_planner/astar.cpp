#include <global_planner/astar.hpp>

#include <stack>
#include "ros/ros.h"

using namespace AStar;

Search::Search(NodeGrid *grid) :
    grid_(grid)
{
}

std::vector<Index2> Search::search(Index2 start, Index2 target)
{
    std::vector<Index2> path;

    if (!isTraversable(start) || !isTraversable(target))
    {
        return path;
    }

    grid_->run += 1; // unique id for this run

    Node*& start_node = grid_->getNodeAt(start);
    start_node->reset(target, grid_->run);
    start_node->cost.setGCost(0);

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

    queue_.clear();

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
                    bx+i < 0 || bx+i >= grid_->width || by+j < 0 || by+j >= grid_->height)
            {
                continue ;
            }

            Node*& neighbor = grid_->getNodeAt(bx+i, by+j);

            if (neighbor->run != grid_->run)
            {
                neighbor->reset(target, grid_->run);
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

            if (updateVertexHasLowerCost(node, neighbor))
            {
                queue_.upsert(neighbor->cost, neighbor, true);
            }
        }
    }
}

bool Search::updateVertexHasLowerCost(Node *node, Node *neighbor)
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