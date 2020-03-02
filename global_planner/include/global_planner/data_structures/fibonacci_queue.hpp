#ifndef FIBONACCI_QUEUE_H
#define FIBONACCI_QUEUE_H

/**
 * Author: Jan Huenermann <jan@huenermann.de>
 */

#include <global_planner/data_structures/fibonacci_heap.hpp>

#include <cassert>

template<class K>
class FibonacciQueue
{
public:

    typedef FibonacciHeap<K> Heap;
    typedef typename FibonacciHeap<K>::Node Node;

    /**
     * Base class for queue elements.
     */
    class Element
    {
    public:
        Node *heap_node;

        Element() : heap_node(nullptr) {}
    };

    /**
     * Actual queue object.
     */
    template<class V>
    class Container
    {
    public:

        /**
         * Inserts an element into the queue.
         * Complexity: O(1)
         * @param Key
         * @param Element
         */
        void insert(K key, V *val)
        {
            auto el = static_cast<Element *>(val);
            assert(el->heap_node == nullptr);
            el->heap_node = new Node(reinterpret_cast<void *>(val));
            heap_.insert(key, el->heap_node);
        };

        /**
         * Decreases the key of an element of the queue.
         * Complexity: O(1) amortized
         * @param Element
         * @param New key
         */
        void decreaseKey(V *val, K key)
        {
            auto el = static_cast<Element *>(val);
            heap_.decreaseKey(el->heap_node, key);
        };

        /**
         * Inserts if not in queue, change key if in queue.
         * Complexity: O(1) amortized
         * @param Key
         * @param Element
         */
        void upsert(K key, V *val, bool increase = false)
        {
            if (has(val))
            {
                auto el = static_cast<Element *>(val);

                if (el->heap_node->key == key)
                {
                    return ;
                }
                else if (increase && el->heap_node->key < key)
                {
                    remove(val);
                    insert(key, val);
                }
                else
                {
                    decreaseKey(val, key);
                }
            }
            else
            {
                insert(key, val);
            }
        }

        /**
         * Checks if a given element is in the queue.
         * Complexity: O(1)
         * @param Element
         * @return True if the element is in the queue.
         */
        inline bool has(V *val)
        {
            auto el = static_cast<Element *>(val);
            return el->heap_node != nullptr;
        }

        /**
         * Removes an element from the queue.
         * Complexity: O(log n)
         * @param
         */
        void remove(V *val)
        {
            auto el = static_cast<Element *>(val);
            heap_.remove(el->heap_node);
            // delete heap node
            delete el->heap_node;
            el->heap_node = nullptr;
        };

        /**
         * Complexity: O(1)
         * @return Element with lowest priority
         */
        V *top()
        {
            return reinterpret_cast<V *>(heap_.top()->payload);
        };

        /**
         * Removes the element with the lowest key from the queue and returns it.
         * Complexity: O(log n)
         * @return Element with the lowest key.
         */
        V *pop()
        {
            if (heap_.size() <= 0)
            {
                return nullptr;
            }

            Node *n = heap_.pop();
            auto el = reinterpret_cast<Element *>(n->payload);
            assert(el->heap_node == n);

            // delete heap node
            el->heap_node = nullptr;
            delete n;

            return static_cast<V *>(el);
        };

        /**
         * Complexity: O(1)
         * @return True if size() is zero.
         */
        inline bool empty()
        {
            return size() <= 0;
        };

        /**
         * Complexity: O(1)
         * @return The number of elements in the queue.
         */
        inline int size()
        {
            return heap_.size();
        };

        /**
         * Removes all elements from the queue.
         * Complexity: O(n)
         */
        void clear()
        {
            clear(heap_.min_node_);
            heap_.clear();
        }

    protected:
        Heap heap_;

        /**
         * Resets the associated elements of each node.
         * @param node to clear
         */
        void clear(Node *n)
        {
            if (!n)
            {
                return ;
            }

            Node *x;
            Node *next = n->right;
            bool done = false;

            // go through all siblings, call recursively, and delete heap node
            do
            {
                x = next;
                next = x->right;

                if (x->child)
                {
                    clear(x->child);
                }

                auto el = reinterpret_cast<Element *>(x->payload);
                assert(el->heap_node == x);
                el->heap_node = nullptr;
                delete x;
            }
            while (x != n);
        }
    };

};

#endif
