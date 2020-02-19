#ifndef FIBONACCI_HEAP_H
#define FIBONACCI_HEAP_H

/**
 * Author: Jan Huenermann <jan@huenermann.de>
 */

#include <cmath>
#include <stdexcept>
#include <limits>
#include <list>
#include <iostream>

static constexpr double ONE_OVER_LOG_PHI = 1.0 / logf((1.0 + sqrtf(5.0)) / 2.0);

template<class T, class U = std::less<T>>
class FibonacciHeap
{
public:

    class Node
    {
    public:

        Node(void *data) :
            key(key),
            payload(data),
            mark(false),
            parent(nullptr),
            left(nullptr),
            right(nullptr),
            child(nullptr),
            num_children(0)
        {
        }

        T key;
        void *payload;

        bool mark;
        Node *parent;
        Node *left;
        Node *right;
        Node *child;
        int num_children;
    };

    FibonacciHeap() : n_(0), min_node_(nullptr), cmp_(U()), _arr(32)
    {};

    /**
     * Inserts node into heap.
     * Complexity: O(1)
     */
    void insert(T key, Node *node)
    {
        node->num_children = 0;
        node->key = key;

        if (min_node_ != nullptr)
        {
            // insert to the left of min_node
            min_node_->left->right = node;
            node->left = min_node_->left;
            min_node_->left = node;
            node->right = min_node_;

            if (cmp_(node->key, min_node_->key))
            {
                min_node_ = node;
            }
        }
        else
        {
            min_node_ = node;
            min_node_->left = node;
            min_node_->right = node;
        }

        ++n_;
    }

    Node *top()
    {
        return min_node_;
    }

    /**
     * Removes smallest element and returns it.
     * Complexity: O(log n)
     * @return The element with the lowest key that was removed.
     */
    Node *pop()
    {
        Node *z, *x, *next;
        Node **children;

        z = min_node_;

        if (z != nullptr)
        {
            x = z->child;

            if (x != nullptr)
            {
                do
                {
                    next = x->right;

                    // insert to left of min_node
                    min_node_->left->right = x;
                    x->left = min_node_->left;
                    min_node_->left = x;
                    x->right = min_node_;
                    x->parent = nullptr;

                    x = next;
                }
                while (x != z->child);

                z->child = nullptr;
            }

            // remove z from the list
            z->left->right = z->right;
            z->right->left = z->left;

            if (z == z->right) // no node left
            {
                min_node_ = nullptr;
            }
            else
            {
                min_node_ = z->right;
                consolidate();
            }

            --n_;
        }

        return z;
    }

    /**
     * Decreases the key of a node in the heap.
     * Complexity: O(1) amortized
     * @param A node.
     * @param Key. Must be lower than the current key of the node.
     */
    void decreaseKey(Node *x, T k)
    {
        if (cmp_(x->key, k))
        {
            throw new std::invalid_argument("decreaseKey() got a larger key");
        }

        x->key = k;

        Node *y = x->parent;

        if (y != nullptr && cmp_(x->key, y->key))
        {
            cut(x, y);
            cascadingCut(y);
        }

        if (cmp_(x->key, min_node_->key))
        {
            min_node_ = x;
        }
    }

    void remove(Node *x)
    {
        decreaseKey(x, - std::numeric_limits<T>::infinity());
        pop();
    }

    /**
     * Removes x from the child list of y.
     * Complexity: O(1)
     * @param
     * @param
     */
    void cut(Node *x, Node *y)
    {
        x->left->right = x->right;
        x->right->left = x->left;
        y->num_children--;

        if (y->child == x)
        {
            y->child = x->right;
        }

        if (y->num_children == 0)
        {
            y->child = nullptr;
        }

        x->left = min_node_;
        x->right = min_node_->right;
        min_node_->right = x;
        x->right->left = x;

        x->parent = nullptr;
        x->mark = false;
    }

    /**
     * Cuts y from its parent and does the same for the parent and so on.
     * Complexity: O(log n)
     * @param Node to perform the cascading on.
     */
    void cascadingCut(Node *y)
    {
        Node *z = y->parent;

        if (z != nullptr)
        {
            if (!y->mark)
            {
                y->mark = true;
            }
            else
            {
                // it's marked, cut it from parent
                cut(y, z);

                // cut parents
                cascadingCut(z);
            }
        }
    }

    /**
     * Make node y child of x.
     * Time: O(1)
     */
    void link(Node *y, Node *x)
    {
        y->left->right = y->right;
        y->right->left = y->left;

        y->parent = x;

        if (x->child == nullptr)
        {
            x->child = y;
            y->right = y;
            y->left = y;
        }
        else
        {
            x->child->left->right = y;
            y->left = x->child->left;
            x->child->left = y;
            y->right = x->child;
        }

        x->num_children++;
        y->mark = false;
    }

    /**
     * Complexity: O(log n)
     */
    void consolidate()
    {
        int d;
        int arr_size = 1 + (int)floor(logf(n_) * ONE_OVER_LOG_PHI);
        
        Node *x, *y;
        std::list<Node *> root_nodes;

        // increase size if needed
        if (_arr.size() < arr_size)
        {
            _arr.resize(std::max(static_cast<int>(2 * _arr.size()), arr_size), nullptr);
        }

        // reset existing values
        std::fill_n(_arr.begin(), arr_size, nullptr);

        x = min_node_;

        do
        {
            root_nodes.push_back(x);
            x = x->right;
        }
        while (x != min_node_);

        for (auto it = root_nodes.begin(); it != root_nodes.end(); ++it)
        {
            x = *it;
            d = x->num_children;

            while (_arr[d] != nullptr)
            {
                y = _arr[d];

                if (cmp_(y->key, x->key))
                {
                    std::swap(x, y);
                }

                link(y, x);

                _arr[d] = nullptr;
                ++d;
            }

            _arr[d] = x;
        }

        min_node_ = nullptr;

        for (int j = 0; j < arr_size; ++j)
        {
            y = _arr[j];

            if (y == nullptr)
            {
                continue ;
            }

            if (min_node_ != nullptr)
            {
                // insert y to the left of min_node
                min_node_->left->right = y;
                y->left = min_node_->left;
                min_node_->left = y;
                y->right = min_node_;

                if (cmp_(y->key, min_node_->key))
                {
                    min_node_ = y;
                }
            }
            else
            {
                min_node_ = y;
                min_node_->left = y;
                min_node_->right = y;
            }
        }
    }

    /**
     * Removes all nodes from the heap.
     * Complexity: O(1)
     */
    void clear()
    {
        n_ = 0;
        min_node_ = nullptr;
    };

    /**
     * Complexity: O(1)
     * @return Number of elements on the heap
     */
    int size()
    {
        return n_;
    };

    U cmp_;
    Node *min_node_;
    int n_;

private:
    std::vector<Node *> _arr;

};


#endif
