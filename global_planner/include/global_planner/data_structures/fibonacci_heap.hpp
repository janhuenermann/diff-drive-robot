#ifndef FIBONACCI_HEAP_H
#define FIBONACCI_HEAP_H

#include <cmath>
#include <stdexcept>


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

    FibonacciHeap() : n_(0), min_node_(nullptr), cmp_(U())
    {};

    /**
     * Inserts node into heap.
     * Time: O(1)
     */
    void insert(T key, Node *node)
    {
        node->key = key;

        if (min_node_ != nullptr)
        {
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

    // O(1)
    Node *top()
    {
        return min_node_;
    }

    /**
     * Removes smallest element.
     * Time: O(log n)
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
                children = new Node*[z->num_children];
                next = x;

                for (int j = 0; j < z->num_children; ++j)
                {
                    children[j] = next;
                    next = next->right;
                }


                for (int j = 0; j < z->num_children; ++j)
                {
                    x = children[j];
                    // insert x to the left of min_node_
                    min_node_->left->right = x;
                    x->left = min_node_->left;
                    min_node_->left = x;
                    x->right = min_node_;
                    x->parent = nullptr;
                }

                delete [] children;
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

    void decreaseKey(Node *x, T k)
    {
        if (!cmp_(x->key, k))
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

    /**
     * Removes x from the child list of y.
     * Time: O(1)
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
            y->left = x->child;
            y->right = x->child->right;
            y->right->left = y;
            x->child->right = y;
        }

        x->num_children += 1;
        y->mark = false;
    }

    void consolidate()
    {
        int arr_size = 1 + (int)floor(logf(n_) * ONE_OVER_LOG_PHI);

        Node *x, *y, *next;
        Node **arr = new Node*[arr_size];
        std::fill_n(arr, arr_size, nullptr);
        int num_roots = 0;

        x = min_node_;

        if (x != nullptr)
        {
            do
            {
                ++num_roots;
                x = x->right;
            }
            while (x != min_node_);
        }

        while (num_roots > 0)
        {
            int d = x->num_children;
            next = x->right;

            while (arr[d] != nullptr)
            {
                y = arr[d];

                if (cmp_(y->key, x->key))
                {
                    std::swap(x, y);
                }

                link(y, x);

                arr[d] = nullptr;
                ++d;
            }

            arr[d] = x;
            x = next;
            --num_roots;
        }

        min_node_ = nullptr;

        for (int j = 0; j < arr_size; ++j)
        {
            y = arr[j];

            if (y == nullptr)
            {
                continue ;
            }

            if (min_node_ != nullptr)
            {
                y->left->right = y->right;
                y->right->left = y->left;

                y->left = min_node_;
                y->right = min_node_->right;

                min_node_->right = y;
                y->right->left = y;

                if (cmp_(y->key, min_node_->key))
                {
                    min_node_ = y;
                }
            }
            else
            {
                min_node_ = y;
            }
        }

        delete [] arr;
    }


    void clear()
    {
        n_ = 0;
        min_node_ = nullptr;
    };

    U cmp_;
    Node *min_node_;
    int n_;

};


#endif
