#include <global_planner/thetastar.hpp>

inline double euclidean_distance(Index2 a, Index2 b)
{
    return (a - b).norm<double>();
}

template<class T>
inline T swapped(bool should_swap, T v)
{
    if (should_swap)
        return T(v.y, v.x);
    else
        return v;
}

bool ThetaStarSearch::hasLineOfSight(ThetaStarSearch::Node *a, ThetaStarSearch::Node *b)
{
    const Point2 start = a->index.cast<double>();
    const Point2 end = b->index.cast<double>(); // center of cell

    const Point2 d = (end - start);
    const bool should_swap = std::abs(d.x) < std::abs(d.y);

    Point2 _ls = swapped(should_swap, start);
    Point2 _ls_f = swapped(should_swap, end);

    Point2 dls = _ls_f - _ls;
    Index2 ls = _ls.round<int>();
    Index2 ls_f = _ls_f.round<int>();

    const bool inv_err = dls.y < 0;

    const double sgn_ld = copysign(1.0, dls.x);
    const double sgn_sd = copysign(1.0, dls.y);

    const int sgn_l = (int)sgn_ld;
    const int sgn_s = (int)sgn_sd;

    const double phi = dls.y / std::abs(dls.x);

    bool first_blocked;
    double eps_s = _ls.y - ls.y;
    double lambda = std::abs(dls.y / dls.x) * (0.5 + (_ls.x - ls.x) * sgn_l) - 0.5;
    double lambda_;

    Index2 cur;

    while (ls != ls_f)
    {
        ls.x += sgn_l;
        eps_s += phi;

        if (inv_err ? eps_s < -0.5 : eps_s >= 0.5) // check if error is big
        {
            eps_s -= sgn_sd;
            ls.y += sgn_s;

            lambda_ = eps_s * sgn_s;

            if (lambda_ < lambda)
            {
                cur = Index2(ls.x, ls.y - sgn_s);

                if (!isTraversable(cur, should_swap)) return false;
                if (cur == ls_f) break ;
            }
            else if (lambda_ > lambda)
            {
                cur = Index2(ls.x - sgn_l, ls.y);

                if (!isTraversable(cur, should_swap)) return false;
                if (cur == ls_f) break ;
            }
            else // corner point
            {
                first_blocked = false;

                cur = Index2(ls.x - sgn_l, ls.y);
                if (!isTraversable(cur, should_swap)) first_blocked = true;
                if (cur == ls_f) break ;

                cur = Index2(ls.x, ls.y - sgn_s);
                if (first_blocked && !isTraversable(cur, should_swap)) return false;
                if (cur == ls_f) break ;
            }
        }

        if (!isTraversable(ls, should_swap)) return false;
    }

    return true;
}

void ThetaStarSearch::resetNode(Node *n, Index2 end)
{
    AStarSearch::resetNode(n, end);
    n->h = Distance(0, euclidean_distance(n->index, end));
}

bool ThetaStarSearch::computeCost(ThetaStarSearch::Node *node, ThetaStarSearch::Node *neighbor, const Distance &d)
{
    if (node->parent != nullptr && hasLineOfSight(node->parent, neighbor))
    {
        Distance tentative_g = node->parent->g + euclidean_distance(node->parent->index, neighbor->index);

        if (tentative_g < neighbor->g)
        {
            neighbor->setGCost(tentative_g);
            neighbor->parent = node->parent;

            return true;
        }
    }
    else
    {
        Distance tentative_g = node->g + d;

        if (tentative_g < neighbor->g)
        {
            neighbor->setGCost(tentative_g);
            neighbor->parent = node;

            return true;
        }
    }

    return false;
}

bool LazyThetaStarSearch::computeCost(ThetaStarSearch::Node *node, ThetaStarSearch::Node *neighbor, const Distance &d)
{
    if (node->parent == nullptr)
    {
        neighbor->setGCost(node->g + d);
        neighbor->parent = node;

        return true;
    }

    // assume there is a line-of-sight
    Distance tentative_g = node->parent->g + euclidean_distance(node->parent->index, neighbor->index);

    if (tentative_g < neighbor->g)
    {
        neighbor->setGCost(tentative_g);
        neighbor->parent = node->parent;

        return true;
    }

    return false;
}


void LazyThetaStarSearch::setVertex(LazyThetaStarSearch::Node *node)
{
    if (node->parent == nullptr)
    {
        return ;
    }

    if (!hasLineOfSight(node->parent, node))
    {
        const Distance d_ord(1,0), d_card(0,1);
        const int bx = node->index.x, by = node->index.y;

        Distance min_g = std::numeric_limits<Distance>::infinity();
        LazyThetaStarSearch::Node *best = nullptr;

        // Iterate over neighbors
        for (int i = -1; i <= 1; ++i)
        {
            for (int j = -1; j <= 1; ++j)
            {
                if ((i == 0 && j == 0) ||
                    bx+i < 0 || bx+i >= width_ || by+j < 0 || by+j >= height_)
                {
                    continue ;
                }

                LazyThetaStarSearch::Node *neighbor = getNodeAt(bx+i, by+j);

                if (!neighbor->visited)
                {
                    continue ;
                }

                Distance c = i != 0 && j != 0 ? d_ord : d_card;
                Distance tentative_g = neighbor->g + c;

                if (tentative_g < min_g)
                {
                    min_g = tentative_g;
                    best = neighbor;
                }
            }
        }

        assert(best != nullptr);

        node->parent = best;
        node->setGCost(min_g);
    }
}