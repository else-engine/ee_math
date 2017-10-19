/**
 * Copyright (c) 2017 Gauthier ARNOULD
 * This file is released under the zlib License (Zlib).
 * See file LICENSE or go to https://opensource.org/licenses/Zlib
 * for full license details.
 */

#pragma once

#include <cstddef>
#include <iostream>
#include <typeinfo>

#include "vec.hpp"

#define EE_MATRIX_COLUMN_MAJOR 1

namespace EE {
namespace math {

inline constexpr auto mat_rc_to_i(const vec<std::size_t, 2>& coords,
                                  const vec<std::size_t, 2>& size) {
#if EE_MATRIX_COLUMN_MAJOR
    return coords(0) + size(0) * coords(1);
#else
    return coords(0) * size(1) + coords(1);
#endif
}

/**
 * Generic matrix template.
 */
template <typename T, std::size_t R, std::size_t C>
struct mat {
    static_assert(std::is_arithmetic<T>::value, "T must be arithmetic type");
    static_assert(R > 0, "R must be at least 1");
    static_assert(C > 0, "C must be at least 1");

    using value_type      = T;
    using reference       = value_type&;
    using const_reference = const value_type&;
    using pointer         = value_type*;
    using const_pointer   = const value_type*;

    constexpr static std::size_t size    = R * C;
    constexpr static std::size_t rows    = R;
    constexpr static std::size_t columns = C;

    T data[size];

    inline constexpr auto& operator()(std::size_t r, std::size_t c) {
        return data[mat_rc_to_i({r, c}, {R, C})];
    }

    inline constexpr const auto& operator()(std::size_t r, std::size_t c) const {
        return data[mat_rc_to_i({r, c}, {R, C})];
    }

    inline constexpr explicit operator bool() const {
        for (auto v : data) {
            if (v) {
                return false;
            }
        }

        return true;
    }
};

template <typename T, std::size_t R, std::size_t C>
std::ostream& operator<<(std::ostream& output, const mat<T, R, C>& m) {
    output << "mat<" << typeid(T).name() << ", " <<
        R << ", " << C << "> {" << std::endl;

    for (std::size_t r = 0; r < R; ++ r) {
        output << "   ";

        for (std::size_t c = 0; c < C; ++ c) {
            output << " " << m(r, c) << (r + 1 == R && c + 1 == C ? "}" : ",");
        }

        if (r + 1 < R) {
            output << std::endl;
        }
    }

    return output;
}

} // namespace math
} // namespace EE
