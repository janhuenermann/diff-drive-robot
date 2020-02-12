#include <global_planner/thetastar.hpp>

inline double euclidean_distance(Index a, Index b)
{
    double dx = (double)(a.x() - b.x());
    double dy = (double)(a.y() - b.y());
    return sqrt((dx * dx) + (dy * dy));
}

inline Eigen::Vector2d swap_axes(Eigen::Vector2d v)
{
    return Eigen::Vector2d(v.y(), v.x());
}

bool ThetaStarSearch::hasLineOfSight(ThetaStarSearch::Node *a, ThetaStarSearch::Node *b)
{
    const Eigen::Vector2d cell_offset = 0.5 * Eigen::Vector2d::Ones();

    Eigen::Vector2d start = a->index.cast<double>();
    Eigen::Vector2d end = b->index.cast<double>(); // center of cell

    Eigen::Vector2d d = (end - start);
    Eigen::Vector2d da = d.cwiseAbs();

    bool axes_swapped = da.x() < da.y();
#define swap_if_needed(v) (axes_swapped ? swap_axes(v) : (v))

    Eigen::Vector2d _ls = swap_if_needed(start);
    Eigen::Vector2d _ls_f = swap_if_needed(end);

    Eigen::Vector2d ls = _ls.array().round().matrix();
    Eigen::Vector2d ls_f = _ls_f.array().round().matrix();
    Eigen::Vector2d dls = _ls_f - _ls;

    bool inv_err = dls.y() < 0;

    double sgn_l = copysign(1.0, dls.x());
    double sgn_s = copysign(1.0, dls.y());
    double phi = dls.y() / abs(dls.x());

    double eps_s = _ls.y() - ls.y();
    double lambda = abs(dls.y() / dls.x()) * (0.5 + (_ls.x() - ls.x()) * sgn_l) - 0.5;
    double lambda_;
    bool first_blocked, second_blocked;

    Eigen::Vector2d cur;
    Eigen::Vector2d v_sgn_l(sgn_l, 0);
    Eigen::Vector2d v_sgn_s(0, sgn_s);

    while (ls != ls_f)
    {
        ls.x() += sgn_l;
        eps_s += phi;

        if (inv_err ? eps_s < -0.5 : eps_s >= 0.5) // check if error is big
        {
            eps_s -= sgn_s;
            ls.y() += sgn_s;

            lambda_ = eps_s * sgn_s;

            if (lambda_ < lambda)
            {
                cur = ls - v_sgn_s;

                if (!isTraversable(swap_if_needed(cur).cast<int>())) return false;
                if (cur == ls_f) break ;
            }
            else if (lambda_ > lambda)
            {
                cur = ls - v_sgn_l;

                if (!isTraversable(swap_if_needed(cur).cast<int>())) return false;
                if (cur == ls_f) break ;
            }
            else // corner point
            {
                first_blocked = false;
                second_blocked = false;

                cur = ls - v_sgn_l;
                if (!isTraversable(swap_if_needed(cur).cast<int>())) first_blocked = true;
                if (cur == ls_f) break ;

                cur = ls - v_sgn_s;
                if (!isTraversable(swap_if_needed(cur).cast<int>())) second_blocked = true;
                if (cur == ls_f) break ;

                if (first_blocked && second_blocked) return false;
            }
        }

        if (!isTraversable(swap_if_needed(ls).cast<int>())) return false;
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