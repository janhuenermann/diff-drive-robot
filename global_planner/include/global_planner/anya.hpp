#ifndef ANYA_H
#define ANYA_H

#include <global_planner/data_structures/fibonacci_queue.hpp>
#include <global_planner/data_structures/math.hpp>

#include <cassert>
#include <iomanip>


namespace ANYA
{

    const double epsilon = 1e-9;

    #define round(a) std::round(a)
    #define is_discrete(a) (std::abs(round(a) - a) < epsilon)

    struct Interval
    {

        const double a;
        const double b;

        int row;

        bool left_open;
        bool right_open;

        Interval(double a, double b, int row, bool left_open = false, bool right_open = false) :
            a(a), b(b),
            row(row),
            left_open(left_open),
            right_open(right_open)
        {
            assert(a <= b);
        }

        Interval(double a, int row)
            : Interval(a, a, row, false, false) {}

        inline bool isDiscreteOnLeft()
        { return is_discrete(a); }

        inline bool isDiscreteOnRight()
        { return is_discrete(b); }

        bool contains(Point2 p);

    };

    inline std::ostream& operator<<(std::ostream &strm, const Interval &a) {
      return strm << "Interval([" << a.a << ", " << a.b << "], row: " << a.row << ")";
    }

    struct Node : public FibonacciQueue<double>::Element
    {

        Interval interval;
        Point2 root;
        Node *parent;
        bool start;
        bool visited;

        double f_cost;
        double g_cost;
        double h_cost;

        Node(Interval interval, Point2 root, Node *parent = nullptr, bool start = false);
        
        void update(Point2 target);

    protected:
        double calculateHCost(Point2 target);

    };

    inline std::ostream& operator<<(std::ostream &strm, const Node &node) {
      return strm << "Node(interval: " << node.interval << ", root: " << node.root << ")";
    }

    const long POINT_VISIBLE = 1<<0;
    const long POINT_CORNER = 1<<1;
    const long POINT_DOUBLE_CORNER = 1<<2;

    struct Grid
    {
        unsigned char *occupancy_; // grid
        long *point_data_; // lattice

        int width_;
        int height_;

        int padded_width_;
        int padded_height_;

        Grid();

        // since we have finite points, the interesections
        // of two points are finite as well.
        // this variable contains the smallest distance between two
        // intersections.
        double intersection_step_size;
        double intersection_step_size_div2;

        inline bool isTraversable(int x, int y) const
        { return !getOccupancy(x, y); }

        inline unsigned char& getOccupancy(int x, int y) const
        { return occupancy_[(y+1) * padded_width_ + x+1]; }

        inline long& getPointData(int x, int y) const
        { return point_data_[y * (width_+1) + x]; };

        inline int getOccupancyIndex(int x, int y) const
        { return (y+1) * padded_width_ + x+1; }

        inline int getPointIndex(int x, int y) const
        { return y * (width_+1) + x; }

        inline bool isPointVisible(int x, int y) const
        { return (getPointData(x, y) & POINT_VISIBLE) == POINT_VISIBLE; }

        inline bool isPointCorner(int x, int y) const
        { return (getPointData(x, y) & POINT_CORNER) == POINT_CORNER; }

        inline bool isPointDoubleCorner(int x, int y) const
        { return (getPointData(x, y) & POINT_DOUBLE_CORNER) == POINT_DOUBLE_CORNER; }

        /**
         * Updates occupancy of the grid.
         * Complexity: O(n)
         * @param w         [description]
         * @param h         [description]
         * @param occupancy [description]
         */
        void update(unsigned char *occupancy, int w, int h);

        // scan obstacles only
        int scanCellsLeft(int x, int y) const;
        int scanCellsRight(int x, int y) const;

        // scan obstacles and corners
        int scanLatticeLeft(double x, int y) const;
        int scanLatticeRight(double x, int y) const;

    private:
        void resize(int w, int h);


    };

    inline std::ostream& operator<<(std::ostream &strm, const Grid &a)
    {
        strm << "Grid(width: " << a.width_ << ", height: " << a.height_ << ", contents: )" << std::endl;
        strm << "   ";
        for (int x = 0; x < a.width_; ++x)
        {
            strm << std::setw(2) << std::setfill('0') << x << " ";
        }

        strm << std::endl;

        for (int y = 0; y < a.height_; ++y)
        {
            strm << std::setw(2) << std::setfill('0') << y << " ";
            for (int x = 0; x < a.width_; ++x)
            {
                strm << (a.isTraversable(x, y) ? ".  " : "X  ");
            }

            strm << std::endl;
        }

        return strm;
    }

    class Projection
    {
    public:

        Projection()
            : valid(false), observable(false)
        {}

        int row;
        int tile_row;
        int type_iii_check_row;

        double left;
        double right;

        double max_left;
        double max_right;

        bool valid;
        bool observable;
        bool deadend;

        void project(Node *node, Grid *grid);
        void projectFlatThroughCone(Node *node, Grid *grid);

        void project(double a, double b, int row,
                     int rx, int ry, Grid *grid);
        void projectCone(double a, double b, int row,
                         int root_x, int root_y, Grid *grid);
        void projectFlat(double a, double b,
                         int root_x, int root_y, Grid *grid);
        void projectFlatThroughCone(double a, double b, int row,
                                    int root_x, int root_y, Grid *grid);
    };

    inline std::ostream& operator<<(std::ostream &strm, const Projection &a) {
      return strm << "Projection([" << a.left << ", " << a.right << "], max: [" << a.max_left << ", " << a.max_right << "], row: " << a.row << ", valid: " << (a.valid ? "True" : "False") << ", obervable: " << (a.observable ? "True" : "False") << ")";
    }

    class Expander
    {
    public:

        Expander(Grid *grid) :
            grid_(grid),
            prune_(false)
        {
        }

        std::list<Node *> expand(Node *node);

    protected:

        void generateStartSuccessors(Node *node, std::list<Node *>& arr);
        void generateStandardSuccessors(Node *node, std::list<Node *>& arr);

        void makeFlatObservables(Node *node, Projection& proj, std::list<Node *>& arr);
        void makeFlatObservables(int rx, int ry, Node *parent, Projection& proj, std::list<Node *>& arr);

        void makeFlatNonObservables(Node *node, Projection& proj, std::list<Node *>& arr);
        
        void makeConeObservables(Node *node, Projection& proj, std::list<Node *>& arr);
        void makeConeObservables(int rx, int ry, Node *parent, Projection& proj, std::list<Node *>& arr);

        void makeConeNonObservables(Node *node, Projection& proj, std::list<Node *>& arr);

        void splitInterval(double max_left, double max_right,
                           int irow, int rx, int ry, Node *parent,
                           std::list<Node *>& array);

        bool isSterile(double left, double right, int row);

    private:
        Grid *grid_;
        bool prune_;
    };

    class Search
    {
    public:

        Search(Grid *grid) :
            grid_(grid)
        {
        }

        std::vector<Point2> search(Point2 start, Point2 goal);

    protected:

        FibonacciQueue<double>::Container<Node> queue_;
        Grid *grid_;

    };

}

#endif