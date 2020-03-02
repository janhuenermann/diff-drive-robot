#include <global_planner/anya.hpp>

using namespace ANYA;

bool Expander::isSterile(double left, double right, int row)
{
    int l = (int)(left + epsilon);
    int r = (int)(right - epsilon);

    return !grid_->isTraversable(l, row) || !grid_->isTraversable(r, row);
}

void Expander::splitInterval(double max_left, double max_right, int irow, 
                             int rx, int ry, Node *parent,
                             std::list<Node *>& array)
{

    if (max_left == max_right)
    {
        return ;
    }

    double succ_left = max_right;
    double succ_right;

    int num_successors = array.size();

    Node *successor = nullptr;

    do
    {
        succ_right = succ_left;
        succ_left = grid_->scanLatticeLeft(succ_right, irow);

        const int a = succ_left < max_left ? max_left : succ_left;
        const int b = succ_right;

        successor = new Node(Interval(a, b, irow), Point2(rx, ry), parent);

        array.push_back(successor);
    }
    while ((succ_left != succ_right) && (succ_left > max_left));

}

void Expander::generateStandardSuccessors(Node *node, std::list<Node *>& arr)
{
    Projection proj;

    if (node->root.y == node->interval.row)
    {
        proj.project(node, grid_);
        makeFlatObservables(node, proj, arr);
        proj.projectFlatThroughCone(node, grid_);
        makeFlatNonObservables(node, proj, arr);
    }
    else
    {
        proj.project(node, grid_);
        makeConeObservables(node, proj, arr);
        makeConeNonObservables(node, proj, arr);
    }

    std::cout << "Expanded " << *node << std::endl;
    for (auto &n : arr)
    {
        std::cout << *n << std::endl;
    }
}

void Expander::makeFlatObservables(Node *node, Projection& proj, std::list<Node *>& arr)
{
    makeFlatObservables((int)node->root.x, (int)node->root.y, node, proj, arr);
}

void Expander::makeFlatObservables(int rx, int ry, Node *parent, Projection& proj, std::list<Node *>& arr)
{
    assert(proj.row == ry);

    if (!proj.valid)
    {
        return ;
    }

    arr.push_back(new Node(
            Interval(proj.left, proj.right, proj.row), 
            Point2(rx, ry),
            parent));
}

void Expander::makeFlatNonObservables(Node *node, Projection& proj, std::list<Node *>& arr)
{
    if (!proj.valid)
    {
        return ;
    }

    int new_rx;
    int new_ry = node->interval.row;

    if (node->root.x <= node->interval.a)
    {
        new_rx = (int)node->interval.b;
    }
    else
    {
        new_rx = (int)node->interval.a;
    }

    splitInterval(proj.left, proj.right, proj.row, new_rx, new_ry, node, arr);
}

void Expander::makeConeObservables(Node *node, Projection& proj, std::list<Node *>& arr)
{
    makeConeObservables((int)node->root.x, (int)node->root.y, node, proj, arr);
}

void Expander::makeConeObservables(int rx, int ry, Node *parent, Projection& proj, std::list<Node *>& arr)
{
    if (!proj.valid || !proj.observable)
    {
        return ;
    }

    splitInterval(proj.left, proj.right, proj.row, rx, ry, parent, arr);
}



void Expander::makeConeNonObservables(Node *node, Projection& proj, std::list<Node *>& arr)
{
    if (!proj.valid)
    {
        return ;
    }

    double ia = node->interval.a;
    double ib = node->interval.b;
    int irow = node->interval.row;

    if (!proj.observable)
    {
        if (node->root.x > ib && grid_->isPointCorner((int)ib, irow))
        {
            splitInterval(proj.max_left, ib, proj.row,
                          ib, irow, node, arr);
        }
        else if (node->root.x < ia && grid_->isPointCorner((int)ia, irow))
        {
            splitInterval(ia, proj.max_right, proj.row,
                          ia, irow, node, arr);
        }

        if (node->interval.isDiscreteOnLeft()
            && !grid_->isTraversable((int)ia-1, proj.type_iii_check_row)
            && grid_->isTraversable((int)ia-1, proj.tile_row))
        {
            proj.projectFlat(ia - grid_->intersection_step_size_div2, ia, (int)ia, irow, grid_);
            makeFlatObservables((int)ia, irow, node, proj, arr);
        }

        if (node->interval.isDiscreteOnRight()
            && !grid_->isTraversable((int)ib, proj.type_iii_check_row)
            && grid_->isTraversable((int)ib, proj.tile_row))
        {
            proj.projectFlat(ib, ib + grid_->intersection_step_size_div2, (int)ib, irow, grid_);
            std::cout << proj << std::endl;
            makeFlatObservables((int)ib, irow, node, proj, arr);
        }
    }
    else
    {
        Projection flatproj;
        int corner_row = irow - sign(node->root.y - irow);

        if (node->interval.isDiscreteOnLeft() &&
                grid_->isPointCorner((int)ia, irow))
        {
            if (!grid_->isTraversable((int)(ia - 1), corner_row))
            {
                flatproj.projectFlat(ia, ib, ia, irow, grid_);
                makeFlatObservables(ia, irow, node, flatproj, arr);
            }

            splitInterval(proj.max_left, proj.left, proj.row, 
                          (int)ia, irow, node, arr);
        }

        if (node->interval.isDiscreteOnRight() &&
                grid_->isPointCorner((int)ib, irow))
        {
            if (!grid_->isTraversable((int)(ib), corner_row))
            {
                flatproj.projectFlat(ia, ib, ib, irow, grid_);
                makeFlatObservables(ib, irow, node, flatproj, arr);
            }

            splitInterval(proj.right, proj.max_right, proj.row, 
                          (int)ib, irow, node, arr);
        }
    }
}

void Expander::generateStartSuccessors(Node *node, std::list<Node *>& arr)
{
    assert(node->interval.a == node->interval.b
            && node->interval.a == (int)node->root.x
            && node->interval.row == (int)node->root.y);

    int rx = (int)node->root.x;
    int ry = (int)node->root.y;

    bool start_dc = grid_->isPointDoubleCorner(rx, ry);

    // we cannot solve these
    if (start_dc && !grid_->isTraversable(rx, ry))
    {
        return ;
    }

    std::cout << "dc: " << (start_dc ? 1 : 0) << std::endl;

    Projection proj;

    if (!start_dc)
    {
        proj.projectFlat(rx, rx, rx+1, ry, grid_);
        makeFlatObservables(rx, ry, node, proj, arr);
    }

    proj.projectFlat(rx, rx, rx-1, ry, grid_);
    makeFlatObservables(rx, ry, node, proj, arr);

    int max_left, max_right;

    // generate conical observable successors below the start point 
    
    if (ry < grid_->height_)
    {
        max_left = grid_->scanCellsLeft(rx, ry);
        max_right = grid_->scanCellsRight(rx, ry);

        std::cout << "max_left: " << max_left << std::endl;
        std::cout << "max_right: " << max_right << std::endl;

        if (max_left != rx && !start_dc)
        {
            splitInterval(max_left, rx, ry + 1, rx, ry, node, arr);
        }

        if (max_right != rx)
        {
            splitInterval(rx, max_right, ry + 1, rx, ry, node, arr);
        }
    }

    // generate conical observable successors above the start point

    if (ry > 0)
    {
        max_left = grid_->scanCellsLeft(rx, ry-1);
        max_right = grid_->scanCellsRight(rx, ry-1);

        if (max_left != rx && !start_dc)
        {
            splitInterval(max_left, rx, ry - 1, rx, ry, node, arr);
        }

        if (max_right != rx)
        {
            splitInterval(rx, max_right, ry - 1, rx, ry, node, arr);
        }

    }
    


}

std::list<Node *> Expander::expand(Node *node)
{
    std::list<Node *> successors;

    if (node->start)
    {
        generateStartSuccessors(node, successors);
    }
    else
    {
        generateStandardSuccessors(node, successors);
    }

    return successors;
}