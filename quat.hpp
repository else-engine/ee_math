/**
 * Copyright (c) 2017 Gauthier ARNOULD
 * This file is released under the zlib License (Zlib).
 * See file LICENSE or go to https://opensource.org/licenses/Zlib
 * for full license details.
 */

#pragma once

#include <iostream>

#include "vec.hpp"

namespace EE {
namespace math {

/**
 * Generic quaternion template.
 */
template <typename T>
struct quat {
    static_assert(std::is_arithmetic<T>::value, "T must be arithmetic type");

    using value_type      = T;
    using reference       = value_type&;
    using const_reference = const value_type&;
    using pointer         = value_type*;
    using const_pointer   = const value_type*;

    constexpr static std::size_t size = 4;

    union {
        T data[size];

        struct { T x, y, z, w; };

        vec<T, 3> xyz;
    };

    inline constexpr auto& operator()(std::size_t i) {
        return data[i];
    }

    inline constexpr const auto& operator()(std::size_t i) const {
        return data[i];
    }

    inline constexpr explicit operator bool() const {
        return x || y || z || w;
    }
};

template <typename T>
std::ostream& operator<<(std::ostream& output, const quat<T>& q) {
    output << "quat<" << typeid(T).name() << "> {"
        << q.x << ", " << q.y << ", " << q.z << ", " << q.w << "}";

    return output;
}

} // namespace math
} // namespace EE
