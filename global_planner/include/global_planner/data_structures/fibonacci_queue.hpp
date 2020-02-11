#ifndef FIBONACCI_QUEUE_H
#define FIBONACCI_QUEUE_H

#include <global_planner/data_structures/fibonacci_heap.hpp>

template<class K, class V>
class FibonacciQueue
{
public:

    typedef FibonacciHeap<K> Heap;
    typedef typename FibonacciHeap<K>::Node Node;

    void insert(K key, V *val)
    {
        Node *n = new Node(reinterpret_cast<void *>(val));
        heap_.insert(key, n);
    };

    V *top()
    {
        return reinterpret_cast<V *>(heap_.top()->payload);
    };

    V *pop()
    {
        if (heap_.n_ <= 0)
        {
            return nullptr;
        }

        Node *n = heap_.pop();
        V *payload = reinterpret_cast<V *>(n->payload);

        delete n;

        return payload;
    };

    bool isEmpty()
    {
        return heap_.n_ <= 0;
    };

    void clear()
    {
        clearSiblings(heap_.min_node_);
        heap_.clear();
    }

protected:
    Heap heap_;

private:
    void clearSiblings(Node *begin)
    {
        Node *x = begin;
        Node *next;

        do
        {
            if (x == nullptr)
            {
                break ;
            }

            next = x->right;

            if (x->child != nullptr && x->num_children)
            {
                clearSiblings(x->child);
            }

            delete x;
            x = next;
        }
        while (x != begin);
    }
};

#endif
