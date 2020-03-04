#ifndef MATH_UTIL_H
#define MATH_UTIL_H

#include <cmath>
#include <iostream>
#include <execinfo.h>

struct Profiler
{
    Profiler() : max(0), sum(0), n(0)
    {
        time_last_print = ros::WallTime::now();
    }

    void start()
    {
        time_start = ros::WallTime::now();
    }

    void stop()
    {
        double time = (ros::WallTime::now() - time_start).toNSec();

        max = std::max(time, max);
        sum += time;
        n += 1;
    }

    void print(std::string name, double every = 20.0)
    {
        if ((ros::WallTime::now() - time_last_print).toSec() > every && n > 0)
        {
            double avg = sum / (double)n * 1e-6;
            ROS_INFO("%s takes on max (ms): %.4lf, average: (ms) %.4lf", name.c_str(), max * 1e-6, avg);
            time_last_print = ros::WallTime::now();
        }
    }

    int n;
    double max;
    double sum;

    ros::WallTime time_start;
    ros::WallTime time_last_print;

};

struct MinMaxTracker
{
    MinMaxTracker() : min(1e100), max(-1e100), n(0)
    {
        time_last_print = ros::WallTime::now();
    }

    void update(double val)
    {
        min = std::min(val, min);
        max = std::max(val, max);
        n++;
    }

    void print(std::string name, std::string unit, double every = 20.0)
    {
        if ((ros::WallTime::now() - time_last_print).toSec() > every && n > 0)
        {
            ROS_INFO("%s min: %.4lf %s, max: %.4lf %s", name.c_str(), min, unit.c_str(), max, unit.c_str());
            time_last_print = ros::WallTime::now();
        }
    }

    int n;
    double min;
    double max;

    ros::WallTime time_last_print;

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