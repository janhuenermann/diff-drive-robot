#include <global_planner/astar.hpp>

#define INF_D std::numeric_limits<double>::infinity()

AStar::AStar(int width, int height) :
    grid_(nullptr),
    width_(0),
    height_(0)
{
    resize(width, height);
}

void AStar::resize(int width, int height)
{
    if (width_ == width && height_ == height)
    {
        return ;
    }

    if (grid_ != nullptr)
    {
        delete [] grid_;
    }

    Distance zero_cost(0,0);
    grid_ = new PlanningNode[width * height];

    for (int j = 0; j < height; ++j)
    {
        for (int i = 0; i < width; ++i)
        {
            PlanningNode &n = getNodeAt(i, j);
            n.index.x = i;
            n.index.y = j;
        }
    }

    width_ = width;
    height_ = height;
}

PlanningNode& AStar::getNodeAt(int x, int y)
{
    return grid_[y * width_ + x];
}

PlanningNode& AStar::getNodeAt(Index i)
{
    return getNodeAt(i.x, i.y);
}

std::vector<PlanningNode *> AStar::search(Index start, Index end)
{
    Distance inf = Distance::infinity();
    std::vector<PlanningNode *> path;

    // reset costs

    for (int j = 0; j < height_; ++j)
    {
        for (int i = 0; i < width_; ++i)
        {
            PlanningNode &n = getNodeAt(i, j);
            n.visited = false;
            n.parent = nullptr;
            n.h = Distance::octileDistance(Index(i, j), end);
            n.g = inf;
            n.f = inf;
        }
    }

    PlanningNode &startNode = getNodeAt(start.x, start.y);
    startNode.setGCost(Distance(0,0));

    queue_.clear();
    queue_.insert(startNode.f, &startNode);

    while (!queue_.isEmpty())
    {
        PlanningNode *node = queue_.pop();

        if (node->visited)
        {
            continue ;
        }

        node->visited = true;

        if (node->index == end)
        {
            while (node != nullptr)
            {
                path.push_back(node->parent);
                node = node->parent;
            }

            break ;
        }

        findSuccessors(node);
    }

    return path;
}

void AStar::findSuccessors(PlanningNode *node)
{
    Distance tentative_g(0,0);
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

            PlanningNode suc = getNodeAt(bx+i, by+j);

            if (suc.state != 0)
            {
                continue ;
            }

            if (i != 0 && j != 0)
            {
                tentative_g = node->g + Distance(1, 0);
            }
            else
            {
                tentative_g = node->g + Distance(0, 1);
            }

            if (tentative_g < suc.g)
            {
                suc.setGCost(tentative_g);
                suc.parent = node;

                // add to queue
                queue_.insert(suc.f, &suc);
            }
        }
    }
}
