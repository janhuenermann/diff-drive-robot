#ifndef LINE_OF_SIGHT_H
#define LINE_OF_SIGHT_H

#include <math/vector2.hpp>

bool line_of_sight(const Index2& pA, const Index2& pB, void (*callback)(const Index2&, bool& stop, void *data), void *data);

template<typename T>
bool line_of_sight(const Vector2<T>& pA, const Vector2<T>& pB, void (*callback)(const Index2&, bool& stop, void *data), void *data);

#endif