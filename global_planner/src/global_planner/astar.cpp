#include <global_planner/astar.hpp>

#include <stack>

using namespace AStar;

Search::Search(int width, int height) :
    grid_(nullptr),
    width_(0),
    height_(0)
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
}

void Search::resetNode(Node *n, Index2 end)
{
    n->visited = false;
    n->parent = nullptr;
    n->cost.reset(n->index, end);
}

std::vector<Index2> Search::search(Index2 start, Index2 end)
{
    std::vector<Index2> path;

    // reset costs

    for (int j = 0; j < height_; ++j)
    {
        for (int i = 0; i < width_; ++i)
        {
            Node*& n = getNodeAt(i, j);
            resetNode(n, end);
        }
    }

    Node*& start_node = getNodeAt(start);
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

        bool skip = setVertex(node);
        if (skip)
        {
            continue ;
        }

        if (node->index == end)
        {
            std::stack<Index2> st;

            while (node != nullptr)
            {
                st.push(node->index);
                node = node->parent;
            }

            while (!st.empty())
            {
                path.push_back(st.top());
                st.pop();
            }

            break ;
        }

        node->visited = true;

        findSuccessors(node);
    }

    return path;
}

void Search::findSuccessors(Node *node)
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