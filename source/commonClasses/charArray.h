#pragma once

#include <array>
#include <algorithm>

/**
 * @brief Creates a std::array of length 10 from a string literal
 * 
 * @note string must be 10 + null characters long, eg "1234567890"
 * @param name String literal
 * @return std::array<char, 10> 
 */
template<int N>
consteval std::array<char, 10> idArr(const char (&name)[N])
{
    static_assert(N == 11, "Length of id and type must be 10");
    std::array<char, 10> buffer;
    std::copy_n(name, 10, buffer.begin());
    return buffer;
}