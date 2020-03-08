#ifndef MATH_UTIL_H
#define MATH_UTIL_H

#include <cmath>
#include <iostream>
#include <chrono>
#include <execinfo.h>

typedef std::chrono::high_resolution_clock high_res_clock;
typedef std::chrono::duration<double, std::milli> high_res_duration;

#define time_now() high_res_clock::now()
#define time_diff(t1, t2, scale) ((std::chrono::duration<double, std::ratio<1, scale>>)(t2 - t1)).count()

struct Profiler
{
    Profiler() : max(0), sum(0), n(0)
    {
        time_last_print = time_now();
    }

    void start()
    {
        time_start = time_now();
    }

    void stop()
    {
        double duration = time_diff(time_start, time_now(), 1000);

        max = std::max(duration, max);
        sum += duration;
        n += 1;
    }

    double getTimeTaken()
    {
        return sum / (double)n;
    }

    void print(std::string name, double every = 20.0)
    {
        if (time_diff(time_last_print, time_now(), 1) > every && n > 0)
        {
            double avg = sum / (double)n;
#ifdef ROS_INFO
            ROS_INFO("%s takes on max (ms): %.4lf, average: (ms) %.4lf", name.c_str(), max, avg);
#else
            std::cout << name << " takes on max (ms): " << max << ", average: (ms) " << avg << std::endl;
#endif
            time_last_print = time_now();
        }
    }

    int n;
    double max;
    double sum;

    high_res_clock::time_point time_start;
    high_res_clock::time_point time_last_print;

};

struct MinMaxTracker
{
    MinMaxTracker() : min(1e100), max(-1e100), n(0)
    {
        time_last_print = time_now();
    }

    void update(double val)
    {
        min = std::min(val, min);
        max = std::max(val, max);
        n++;
    }

    void print(std::string name, std::string unit, double every = 20.0)
    {
        if (time_diff(time_last_print, time_now(), 1) > every && n > 0)
        {
#ifdef ROS_INFO
            ROS_INFO("%s min: %.4lf %s, max: %.4lf %s", name.c_str(), min, unit.c_str(), max, unit.c_str());
#else
            std::cout << name << " min: " << min << " " << unit << ", max: " << max << " " << unit << std::endl;
#endif
            time_last_print = time_now();
        }
    }

    int n;
    double min;
    double max;

    high_res_clock::time_point time_last_print;

};

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