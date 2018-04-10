/**
 * Copyright (c) 2017-2018 Gauthier ARNOULD
 * This file is released under the zlib License (Zlib).
 * See file LICENSE or go to https://opensource.org/licenses/Zlib
 * for full license details.
 */

#pragma once

#include <iostream>

#include <ee_utils/componentwise.hpp>

#include "vec.hpp"

namespace ee {
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

    inline constexpr reference operator()(std::size_t d) {
        return data[d];
    }

    inline constexpr const_reference operator()(std::size_t d) const {
        return data[d];
    }

    inline constexpr reference operator[](std::size_t index) {
        return data[index];
    }

    inline constexpr const_reference operator[](std::size_t index) const {
        return data[index];
    }

    inline constexpr explicit operator bool() const {
        return x || y || z || w;
    }
};

/**
 * A way to identify quaternions.
 */
namespace detail {

template <typename>
struct is_quat_impl : std::false_type {};

template <typename T>
struct is_quat_impl<quat<T>> : std::true_type {};

} // namespace detail

template <typename T>
constexpr bool is_quat = detail::is_quat_impl<std::decay_t<T>>::value;

/**
 * Output formatting
 */
template <typename T>
std::ostream& operator<<(std::ostream& output, const quat<T>& q) {
    output << "quat<" << typeid(T).name() << "> {"
        << q.x << ", " << q.y << ", " << q.z << ", " << q.w << "}";

    return output;
}

} // namespace math

template <typename VT, typename T>
struct but<math::quat<T>, VT> {
    using type = math::quat<VT>;
};

} // namespace ee

namespace std {

template <typename T>
class tuple_size<::ee::math::quat<T>> : public integral_constant<size_t, 4u> {};

} // namespace std
