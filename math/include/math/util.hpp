#ifndef MATH_UTIL_H
#define MATH_UTIL_H

#include <cmath>
#include <iostream>
#include <execinfo.h>

template<class T>
inline T norm(T ax, T ay, T bx, T by)
{
    T dx = bx-ax;
    T dy = by-ay;
    return std::sqrt(dx*dx + dy*dy);
}

template<class T>
inline T sign(T a)
{
    if (a >= 0)
        return 1;
    else
        return -1;
}

inline void print_trace()
{
    void* callstack[128];
    int i, frames = backtrace(callstack, 128);
    char** strs = backtrace_symbols(callstack, frames);
    
    for (i = 0; i < frames; ++i)
    {
        std::cout << strs[i] << std::endl;
    }

    free(strs);
}

#endif