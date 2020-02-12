#include <global_planner/astar.hpp>

#include <stack>

AStarSearch::AStarSearch(int width, int height) :
    grid_(nullptr),
    width_(0),
    height_(0)
{
    resize(width, height);
}

void AStarSearch::resize(int width, int height)
{
    if (grid_ != nullptr)
    {
        delete [] grid_;
    }

    Distance zero_cost(0,0);
    grid_ = new AStarSearch::Node*[width * height];
    width_ = width;
    height_ = height;

    for (int j = 0; j < height; ++j)
    {
        for (int i = 0; i < width; ++i)
        {
            getNodeAt(i, j) = new AStarSearch::Node(i, j);
        }
    }
}

void AStarSearch::resetNode(Node *n, Index end)
{
    n->visited = false;
    n->parent = nullptr;
    n->h = Distance::octileDistance(n->index, end);
    n->g = DISTANCE_INFINITY;
    n->f = DISTANCE_INFINITY;
}

std::vector<AStarSearch::Node *> AStarSearch::search(Index start, Index end)
{
    std::vector<AStarSearch::Node *> path;

    // reset costs

    for (int j = 0; j < height_; ++j)
    {
        for (int i = 0; i < width_; ++i)
        {
            AStarSearch::Node*& n = getNodeAt(i, j);
            resetNode(n, end);
        }
    }

    AStarSearch::Node*& startNode = getNodeAt(start);
    startNode->setGCost(Distance(0,0));

    queue_.clear();
    queue_.insert(startNode->f, startNode);

    while (!queue_.empty())
    {
        AStarSearch::Node *node = queue_.pop();

        if (node->visited)
        {
            continue ;
        }

        setVertex(node);

        if (node->index == end)
        {
            std::stack<AStarSearch::Node *> st;

            while (node != nullptr)
            {
                st.push(node);
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

void AStarSearch::findSuccessors(AStarSearch::Node *node)
{
    const Distance d_ord(1,0), d_card(0,1);
    Distance f_old(0,0);
    bool lower_cost;
    int bx = node->index.x(), by = node->index.y();

    for (int i = -1; i <= 1; ++i)
    {
        for (int j = -1; j <= 1; ++j)
        {
            if ((i == 0 && j == 0) ||
                    bx+i < 0 || bx+i >= width_ || by+j < 0 || by+j >= height_)
            {
                continue ;
            }


            AStarSearch::Node*& neighbor = getNodeAt(bx+i, by+j);

            if (!isTraversable(neighbor) || neighbor->visited)
            {
                continue ;
            }

            f_old = neighbor->f;
            lower_cost = computeCost(node, neighbor, i != 0 && j != 0 ? d_ord : d_card);

            if (lower_cost)
            {
                if (f_old < neighbor->f) // remove because key is not smaller
                {
                    queue_.remove(neighbor);
                }

                queue_.upsert(neighbor->f, neighbor);
            }
        }
    }
}

bool AStarSearch::computeCost(AStarSearch::Node *node, AStarSearch::Node *neighbor, const Distance &d)
{
    Distance tentative_g = node->g + d;

    if (tentative_g < neighbor->g)
    {
        neighbor->setGCost(tentative_g);
        neighbor->parent = node;

        return true;
    }

    return false;
}