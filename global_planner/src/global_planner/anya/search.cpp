#include <global_planner/anya.hpp>
#include <stack>

using namespace ANYA;

std::vector<Point2> Search::search(Point2 start, Point2 target)
{
    Expander expander(grid_);

    std::vector<Point2> result;
    Node *start_node, *active;

    assert(is_discrete(start.y));

    start_node = new Node(Interval(start.x, (int)round(start.y)), start, nullptr, true);
    start_node->update(target);

    queue_.insert(start_node->f_cost, start_node);

    while (!queue_.empty())
    {
        Node *current = queue_.pop();

        if (current->visited)
        {
            continue ;
        }

        if (current->interval.contains(target.cast<double>()))
        {
            // done
            std::cout << "DONE" << std::endl;

            std::stack<Point2> st;
            st.push(target);

            active = current;
            while (active != nullptr)
            {
                st.push(active->root);
                active = active->parent;
            }

            while (!st.empty())
            {
                Point2 pt = st.top();

                if (result.empty() || pt != result.back())
                {
                    std::cout << pt << std::endl;
                    result.push_back(pt);
                }

                st.pop();
            }

            break ;
        }

        std::list<Node *> successors = expander.expand(current);

        for (Node *& succ : successors)
        {
            assert(succ->parent == current);

            // set loss
            succ->update(target);

            // add to queue
            queue_.insert(succ->f_cost, succ);
        }

    }

    return result;

}