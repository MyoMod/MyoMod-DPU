#pragma once

#include <array>
#include <algorithm>

template<int N>
consteval std::array<char, 10> idArr(const char (&name)[N])
{
    static_assert(N == 11, "Length of id and type must be 10");
    std::array<char, 10> buffer;
    std::copy_n(name, 10, buffer.begin());
    return buffer;
}