#include <math/line_of_sight.hpp>

bool line_of_sight(const Index2& pA, const Index2& pB, void (*callback)(const Index2&, bool& stop, void *data), void *data)
{
    Index2 d = pB - pA;
    Index2 p = pA, p_;
    bool stop = false, stop_a = false, stop_b = false;

#define LOOP \
    Vector2<int> d_abs = d.abs();\
    Vector2<int> d_sgn = d.sgn();\
    int phi = 2 * SHORT_AXIS(d);\
    int eta = 2 * LONG_AXIS(d_abs) * SHORT_AXIS(d_sgn);\
    int err = 0, lambda = SHORT_AXIS(d_abs) - LONG_AXIS(d_abs), lambda_;\
    while (p != pB && !stop) \
    {\
        LONG_AXIS(p) += LONG_AXIS(d_sgn);\
        err += phi;\
        if (HAS_BIG_ERR())\
        {\
            err -= eta;\
            SHORT_AXIS(p) += SHORT_AXIS(d_sgn);\
            lambda_ = err * SHORT_AXIS(d_sgn);\
            p_ = p;\
            if (lambda_ < lambda)\
            {\
                SHORT_AXIS(p_) -= SHORT_AXIS(d_sgn);\
                callback(p_, stop, data);\
            }\
            else if (lambda_ > lambda)\
            {\
                LONG_AXIS(p_) -= LONG_AXIS(d_sgn);\
                callback(p_, stop, data);\
            }\
            else\
            {\
                LONG_AXIS(p_) -= LONG_AXIS(d_sgn);\
                callback(p_, stop_a, data);\
                if (p_ == pB) { break ; } \
                p_ = p;\
                SHORT_AXIS(p_) -= SHORT_AXIS(d_sgn);\
                callback(p_, stop_b, data);\
                stop = stop_a && stop_b;\
            }\
            if (stop || p_ == pB)\
            {\
                break; \
            }\
        }\
        callback(p, stop, data);\
    }

    if (std::abs(d.x) > std::abs(d.y))
    {
        #define LONG_AXIS(pt) pt.x
        #define SHORT_AXIS(pt) pt.y

        if (SHORT_AXIS(d) >= 0)
        {
            #define HAS_BIG_ERR() (err >= LONG_AXIS(d_abs))
            LOOP;
            #undef HAS_BIG_ERR
        }
        else
        {
            #define HAS_BIG_ERR() (err < -LONG_AXIS(d_abs))
            LOOP;
            #undef HAS_BIG_ERR
        }

        #undef LONG_AXIS
        #undef SHORT_AXIS
    }
    else
    {
        #define LONG_AXIS(pt) pt.y
        #define SHORT_AXIS(pt) pt.x

        if (SHORT_AXIS(d) >= 0)
        {
            #define HAS_BIG_ERR() (err >= LONG_AXIS(d_abs))
            LOOP;
            #undef HAS_BIG_ERR
        }
        else
        {
            #define HAS_BIG_ERR() (err < -LONG_AXIS(d_abs))
            LOOP;
            #undef HAS_BIG_ERR
        }

        #undef LONG_AXIS
        #undef SHORT_AXIS
    }

#undef LOOP

    return stop;

}

template<typename T>
bool line_of_sight(const Vector2<T>& pA, const Vector2<T>& pB, void (*callback)(const Index2&, bool& stop, void *data), void *data)
{
    static_assert(std::is_same<T, double>::value || std::is_same<T, float>::value, "T must be float or double");

    const Vector2<T> d = pB - pA;

    Index2 p = pA.round();
    Index2 p_end = pB.round();
    Index2 p_;

    T lambda_;

    bool stop = false, stop_a = false, stop_b = false;

#define LOOP \
    int sgns = copysign(1, SHORT_AXIS(d));\
    int sgnl = copysign(1, LONG_AXIS(d));\
    T phi = SHORT_AXIS(d) / std::abs(LONG_AXIS(d));\
    T err = SHORT_AXIS(pA) - SHORT_AXIS(p); \
    T lambda = std::abs(SHORT_AXIS(d) / LONG_AXIS(d)) * (0.5 + (LONG_AXIS(pA) - LONG_AXIS(p)) * sgnl) - 0.5; \
    while (p != p_end && !stop) \
    {\
        LONG_AXIS(p) += sgnl;\
        err += phi;\
        if (HAS_BIG_ERR())\
        {\
            err -= sgns;\
            SHORT_AXIS(p) += sgns;\
            lambda_ = err * sgns;\
            p_ = p;\
            if (lambda_ < lambda)\
            {\
                SHORT_AXIS(p_) -= sgns;\
                callback(p_, stop, data);\
            }\
            else if (lambda_ > lambda)\
            {\
                LONG_AXIS(p_) -= sgnl;\
                callback(p_, stop, data);\
            }\
            else\
            {\
                LONG_AXIS(p_) -= sgnl;\
                callback(p_, stop_a, data);\
                if (p_ == p_end) { break ; } \
                p_ = p;\
                SHORT_AXIS(p_) -= sgns;\
                callback(p_, stop_b, data);\
                stop = stop_a && stop_b;\
            }\
            if (stop || p_ == p_end)\
            {\
                break; \
            }\
        }\
        callback(p, stop, data);\
    }

    if (std::abs(d.x) > std::abs(d.y))
    {
        #define LONG_AXIS(v) v.x
        #define SHORT_AXIS(v) v.y

        if (SHORT_AXIS(d) >= 0)
        {
            #define HAS_BIG_ERR() (err >= 0.5)
            LOOP;
            #undef HAS_BIG_ERR
        }
        else
        {
            #define HAS_BIG_ERR() (err < -0.5)
            LOOP;
            #undef HAS_BIG_ERR
        }

        #undef LONG_AXIS
        #undef SHORT_AXIS
    }
    else
    {
        #define LONG_AXIS(v) v.y
        #define SHORT_AXIS(v) v.x

        if (SHORT_AXIS(d) >= 0)
        {
            #define HAS_BIG_ERR() (err >= 0.5)
            LOOP;
            #undef HAS_BIG_ERR
        }
        else
        {
            #define HAS_BIG_ERR() (err < -0.5)
            LOOP;
            #undef HAS_BIG_ERR
        }

        #undef LONG_AIXS
        #undef SHORT_AXIS
    }

    return stop;

}


template bool line_of_sight<double>(const Vector2<double>& pA, const Vector2<double>& pB, void (*callback)(const Index2&, bool& stop, void *data), void *data);
template bool line_of_sight<float>(const Vector2<float>& pA, const Vector2<float>& pB, void (*callback)(const Index2&, bool& stop, void *data), void *data);



