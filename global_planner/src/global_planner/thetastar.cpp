#include <global_planner/thetastar.hpp>

#include <math/line_of_sight.hpp>

using namespace ThetaStar;

template<class T>
inline T swapped(bool should_swap, T v)
{
    if (should_swap)
        return T(v.y, v.x);
    else
        return v;
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

inline bool isColinear(Index2 w, Index2 u, Index2 v)
{
    return w.x * (u.y - v.y) + u.x * (v.y - w.y) + v.x * (w.y - u.y) == 0;
}

void lazythetastar_line_of_sight_cb(const Vector2<int>& p, bool& stop, void *data)
{
    if (!((Search *)data)->isTraversable(p))
    {
        stop = true;
    }
}

bool Search::hasLineOfSight(Node *a, Node *b)
{
    return !line_of_sight(a->index.cast<float>(), b->index.cast<float>(), 
                          &lazythetastar_line_of_sight_cb, this);
}

bool Search::updateVertexHasLowerCost(Node *node, Node *neighbor)
{
    if (node->parent != nullptr && hasLineOfSight(node->parent, neighbor))
    {
        return relax(node->parent, neighbor);
    }
    else
    {
        return relax(node, neighbor);
    }

    return false;
}

bool LazySearch::updateVertexHasLowerCost(Node *node, Node *neighbor)
{
    if (node->parent == nullptr)
    {
        return relax(node, neighbor);
    }

    // assume there is a line-of-sight
    return relax(node->parent, neighbor);
}

bool LazySearch::setVertexShouldSkip(Node *node)
{
    if (node->parent == nullptr)
    {
        return false;
    }

    if (!hasLineOfSight(node->parent, node))
    {
        const int bx = node->index.x, by = node->index.y;

        double min_g = std::numeric_limits<double>::infinity();
        Node *best = nullptr;

        // Iterate over neighbors
        for (int i = -1; i <= 1; ++i)
        {
            for (int j = -1; j <= 1; ++j)
            {
                if ((i == 0 && j == 0) ||
                    bx+i < 0 || bx+i >= grid_->width || by+j < 0 || by+j >= grid_->height)
                {
                    continue ;
                }

                Node *neighbor = grid_->getNodeAt(bx+i, by+j);

                if (!neighbor->visited || neighbor->run != grid_->run)
                {
                    continue ;
                }

                // dead corners
                if (i != 0 && j != 0 && !isTraversable(Index2(bx+i, by)) && !isTraversable(Index2(bx, by+j)))
                {
                    continue ;
                }

                double d = i != 0 && j != 0 ? M_SQRT2 : 1;
                double tentative_g = neighbor->cost.g + d;

                if (tentative_g < min_g)
                {
                    min_g = tentative_g;
                    best = neighbor;
                }
            }
        }

        assert(best != nullptr);

        node->parent = best;
        node->cost.setGCost(min_g);

        queue_.insert(node->cost, node);

        return true;
    }

    return false;
}

// double StrictSearch::penalty(Node *node, Node *neighbor)
// {
//     if (node->parent != nullptr && isTaut(node->parent->index, node->index, neighbor->index))
//     {
//         std::cout << "TAUT" << node->index << std::endl;
//         return 0;
//     }
//     else
//     {
//         return 1000.0;
//     }
// }

// bool StrictSearch::isTaut(Index2 w, Index2 u, Index2 v)
// {
//     Index2 wu = w - u;
//     Index2 uv = v - u;

//     if (isColinear(w, u, v))
//     {
//         return true;
//     }

//     if (((uv.x > 0) == (wu.x > 0) && (uv.x < 0) == (wu.x < 0))      // both x > 0 || x < 0
//         || ((uv.y > 0) == (wu.y > 0) && (uv.y < 0) == (wu.y < 0)))  // both y > 0 || y < 0
//     {
//         return false; // acute angle
//     }

//     Index2 i;
//     if (uv.x >= 0 && uv.y >= 0)
//     {
//         i = Index2(-1, 0);
//     }
//     else if (uv.x <= 0 && uv.y >= 0)
//     {
//         i = Index2(0, 0);
//     }
//     else if (uv.x >= 0 && uv.y >= 0)
//     {
//         i = Index2(0, -1);
//     }
//     else if (uv.x >= 0 && uv.y <= 0)
//     {
//         i = Index2(-1, -1);
//     }

//     return !isTraversable(u+i);
// }