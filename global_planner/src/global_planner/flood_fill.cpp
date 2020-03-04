#include <global_planner/flood_fill.hpp>

#include <stack>
#include "ros/ros.h"

FloodFill::FloodFill(NodeGrid *grid) :
    grid_(grid)
{
}

std::vector<Index2> FloodFill::search(Index2 start)
{
    std::vector<Index2> path;

    grid_->run += 1; // unique id for this run

    Node*& start_node = grid_->getNodeAt(start);
    start_node->reset(grid_->run);
    start_node->cost.setGCost(0);

    queue_.insert(start_node->cost, start_node);

    while (!queue_.empty())
    {
        Node *node = queue_.pop();

        if (node->visited)
        {
            continue ;
        }

        if (isTarget(node))
        {
            std::stack<Node *> st;
            Node *last = nullptr;

            while (node != nullptr)
            {
                st.push(node);
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

        findSuccessors(node);
    }

    queue_.clear();

    return path;
}

void FloodFill::findSuccessors(Node *node)
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
                neighbor->reset(grid_->run);
            }
            
            if (neighbor->visited)
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

bool FloodFill::updateVertexHasLowerCost(Node *node, Node *neighbor)
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