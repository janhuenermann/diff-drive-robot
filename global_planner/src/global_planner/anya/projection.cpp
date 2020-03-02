#include <global_planner/anya.hpp>

using namespace ANYA;

void Projection::project(Node *node, Grid *grid)
{
    project(node->interval.a, node->interval.b, node->interval.row,
            (int)node->root.x, (int)node->root.y, grid);
}

void Projection::project(double a, double b, int row,
                         int rx, int ry, Grid *grid)
{
    observable = false;
    valid = false;

    if (row == ry) // flat
    {
        projectFlat(a, b, rx, ry, grid);
    }
    else // cone
    {
        projectCone(a, b, row, rx, ry, grid);
    }
}

void Projection::projectFlatThroughCone(Node *node, Grid *grid)
{
    projectFlatThroughCone(node->interval.a, node->interval.b, node->interval.row,
                           (int)node->root.x, (int)node->root.y, grid);
}

void Projection::projectCone(double ia, double ib, int irow,
                             int rx, int ry, Grid *grid)
{
    if (ry < irow)
    {
        tile_row = irow;
        row = irow + 1;
        type_iii_check_row = irow - 1;
    }
    else
    {
        assert(ry > irow); // not a flat node
        row = tile_row = irow - 1;
        type_iii_check_row = irow;
    }

    valid = grid->isTraversable((int)(ia + grid->intersection_step_size_div2), tile_row)
         && grid->isTraversable((int)(ib - grid->intersection_step_size_div2), tile_row);

    if (!valid)
    {
        return ;
    }

    double rise = (double)std::abs(irow - ry);
    double lrun = rx - ia;
    double rrun = ib - rx;

    max_left = grid->scanCellsLeft((int)ia, tile_row);
    max_right = grid->scanCellsRight((int)ib, tile_row);

    left = std::max(ia - lrun/rise, max_left);
    right = std::min(ib + rrun/rise, max_right);

    observable = left < right;

    if (left >= max_right)
    {
        left = grid->isTraversable((int)(ia - grid->intersection_step_size_div2), tile_row) ? right : max_left;
    }

    if (right <= max_left)
    {
        right = grid->isTraversable((int)ib, tile_row) ? left : max_right;
    }
}

void Projection::projectFlat(double ia, double ib,
                             int rx, int ry, Grid *grid)
{
    if (rx <= ia)
    {
        left = ib;
        right = grid->scanLatticeRight(left, ry);

        // maybe replace with double-corner check?
        deadend = !grid->isTraversable((int)right - 1, ry)
               || !grid->isTraversable((int)right - 1, ry-1);
    }
    else
    {
        right = ia;
        left = grid->scanLatticeLeft(right, ry);
        // maybe replace with double-corner check?
        deadend = !grid->isTraversable((int)left, ry)
               || !grid->isTraversable((int)left, ry-1);
    }


    row = ry;
    valid = left != right;
    observable = true; // flat projections are always observable

    max_left = NAN;
    max_right = NAN;

}

void Projection::projectFlatThroughCone(double ia, double ib, int irow,
                                        int rx, int ry, Grid *grid)
{
    if (rx <= ia)
    {
        // can we make a valid turn? valid means 
        // (i) the path bends around a corner; 
        // (ii) we do not step through any obstacles or through 
        // double-corner points.
        bool can_step = grid->isTraversable((int)ib, irow)
                     && grid->isTraversable((int)ib, irow-1);

        if (!can_step)
        {
            valid = false;
            observable = false;
            return ;
        }

        if (!grid->isTraversable((int)ib-1, irow))
        {
            // going down
            row = irow+1;
            tile_row = irow;
        }
        else
        {
            // going up
            row = tile_row = irow-1;
        }

        left = max_left = ib;
        right = max_right = grid->scanCellsRight((int)left, tile_row);
    }
    else
    {
        assert(rx >= ib);
        bool can_step = grid->isTraversable((int)ia-1, irow)
                     && grid->isTraversable((int)ia-1, irow-1);

        if (!can_step)
        {
            valid = false;
            observable = false;
            return ;
        }

        if (!grid->isTraversable((int)ia, irow))
        {
            // going down
            row = irow+1;
            tile_row = irow;
        }
        else
        {
            // going up
            row = tile_row = irow-1;
        }

        right = max_right = ia;
        left = max_left = grid->scanCellsLeft((int)right, tile_row);
    }

    valid = true;
    observable = false;
}