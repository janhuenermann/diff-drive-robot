#include <global_planner/thetastar.hpp>

inline double euclidean_distance(Index a, Index b)
{
    double dx = (double)(a.x() - b.x());
    double dy = (double)(a.y() - b.y());
    return sqrt((dx * dx) + (dy * dy));
}

template<class T>
inline T swap_axes(T v)
{
    return T(v.y(), v.x());
}

bool ThetaStarSearch::hasLineOfSight(ThetaStarSearch::Node *a, ThetaStarSearch::Node *b)
{
    Eigen::Vector2d start = a->index.cast<double>();
    Eigen::Vector2d end = b->index.cast<double>(); // center of cell

    Eigen::Vector2d d = (end - start);

    bool axes_swapped = abs(d.x()) < abs(d.y());

#define swap_if_needed(v) (axes_swapped ? swap_axes(v) : (v))

    Eigen::Vector2d _ls = swap_if_needed(start);
    Eigen::Vector2d _ls_f = swap_if_needed(end);

    Eigen::Vector2d dls = _ls_f - _ls;
    Eigen::Vector2i ls((int)round(_ls.x()), (int)round(_ls.y()));
    Eigen::Vector2i ls_f((int)round(_ls_f.x()), (int)round(_ls_f.y()));

    const bool inv_err = dls.y() < 0;

    const double sgn_ld = copysign(1.0, dls.x());
    const double sgn_sd = copysign(1.0, dls.y());

    const int sgn_l = (int) sgn_ld;
    const int sgn_s = (int) sgn_sd;

    const double phi = dls.y() / abs(dls.x());

    bool first_blocked;
    double eps_s = _ls.y() - ls.y();
    double lambda = abs(dls.y() / dls.x()) * (0.5 + (_ls.x() - ls.x()) * sgn_l) - 0.5;
    double lambda_;

    Eigen::Vector2i cur;
    const Eigen::Vector2i v_sgn_l(sgn_l, 0),
                          v_sgn_s(0, sgn_s);

    while (ls != ls_f)
    {
        ls.x() += sgn_l;
        eps_s += phi;

        if (inv_err ? eps_s < -0.5 : eps_s >= 0.5) // check if error is big
        {
            eps_s -= sgn_sd;
            ls.y() += sgn_s;

            lambda_ = eps_s * sgn_s;

            if (lambda_ < lambda)
            {
                cur = ls - v_sgn_s;

                if (!isTraversable(cur, axes_swapped)) return false;
                if (cur == ls_f) break ;
            }
            else if (lambda_ > lambda)
            {
                cur = ls - v_sgn_l;

                if (!isTraversable(cur, axes_swapped)) return false;
                if (cur == ls_f) break ;
            }
            else // corner point
            {
                first_blocked = false;

                cur = ls - v_sgn_l;
                if (!isTraversable(cur, axes_swapped)) first_blocked = true;
                if (cur == ls_f) break ;

                cur = ls - v_sgn_s;
                if (first_blocked && !isTraversable(cur, axes_swapped)) return false;
                if (cur == ls_f) break ;
            }
        }

        if (!isTraversable(ls, axes_swapped)) return false;
    }

    return true;
}

void ThetaStarSearch::resetNode(Node *n, Index end)
{
    AStarSearch::resetNode(n, end);
    n->h = Distance(0, euclidean_distance(n->index, end));
}

bool ThetaStarSearch::computeCost(ThetaStarSearch::Node *node, ThetaStarSearch::Node *neighbor, const Distance &d)
{
    if (node->parent != nullptr && hasLineOfSight(node->parent, neighbor))
    {
        Distance tentative_g = node->parent->g + euclidean_distance(node->parent->index, neighbor->index);;

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

    Distance tentative_g = node->parent->g + euclidean_distance(node->parent->index, neighbor->index);;

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
        const int bx = node->index.x(), by = node->index.y();

        Distance min_g = DISTANCE_INFINITY;
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