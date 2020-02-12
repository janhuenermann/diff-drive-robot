#include <gtest/gtest.h>
#include <global_planner/data_structures/fibonacci_heap.hpp>
#include <global_planner/data_structures/fibonacci_queue.hpp>

TEST(FibonacciHeapTest, testInsert){
    int a = 1;
    int b = 2;
    int c = 3;
    int d = 4;

    FibonacciHeap<double> heap;
    auto node_a = new FibonacciHeap<double>::Node(&a);
    auto node_b = new FibonacciHeap<double>::Node(&b);
    auto node_c = new FibonacciHeap<double>::Node(&c);
    auto node_d = new FibonacciHeap<double>::Node(&d);

    heap.insert(2.0, node_a);
    heap.insert(1.0, node_b);
    heap.insert(-7.0, node_c);
    heap.insert(5.0, node_d);

    FibonacciHeap<double>::Node *ptr;
    EXPECT_EQ(heap.n_, 4);

    ptr = heap.pop();
    EXPECT_EQ(*(int *)(ptr->payload), c);
    EXPECT_EQ(heap.n_, 3);

    ptr = heap.pop();
    EXPECT_EQ(*(int *)(ptr->payload), b);
    EXPECT_EQ(heap.n_, 2);

    ptr = heap.pop();
    EXPECT_EQ(*(int *)(ptr->payload), a);
    EXPECT_EQ(heap.n_, 1);

    ptr = heap.pop();
    EXPECT_EQ(*(int *)(ptr->payload), d);
    EXPECT_EQ(heap.n_, 0);

    delete node_a, node_b, node_c, node_d;
}

TEST(FibonacciQueueTest, testInsert){
    int a = 1;
    int b = 2;
    int c = 3;
    int d = 4;
    int e = 5;

    FibonacciQueue<double, int> q;

    q.insert(-4.0, &b);
    q.insert(5.5, &c);
    q.insert(-10.1, &a);
    q.insert(8.78, &e);

    EXPECT_EQ(*q.pop(), a);
    EXPECT_EQ(*q.pop(), b);

    q.insert(6.0, &d);

    EXPECT_EQ(*q.pop(), c);
    EXPECT_EQ(*q.pop(), d);
    EXPECT_EQ(*q.pop(), e);

    EXPECT_TRUE(q.isEmpty());
}
