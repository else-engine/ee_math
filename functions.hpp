/**
 * Copyright (c) 2017 Gauthier ARNOULD
 * This file is released under the zlib License (Zlib).
 * See file LICENSE or go to https://opensource.org/licenses/Zlib
 * for full license details.
 */

#pragma once

#include <cmath>
#include <algorithm>

#include <ee_utils/templates.hpp>

/**
 * All functions here are instances of unamed structs. It allows direct call
 * like standard functions but also using them as functors for componentwise
 * operations.
 */

namespace ee {
namespace math {

using tutil::eif;

/**
 * Addition.
 */
constexpr struct {
    template <typename T>
    constexpr T operator()(T lhs, T rhs) const {
        return lhs + rhs;
    }
} add;

/**
 * Subtraction.
 */
constexpr struct {
    template <typename T>
    constexpr T operator()(T lhs, T rhs) const {
        return lhs - rhs;
    }
} sub;

/**
 * Multiplication.
 */
constexpr struct {
    template <typename T>
    constexpr T operator()(T lhs, T rhs) const {
        return lhs * rhs;
    }
} mul;

/**
 * Division.
 */
constexpr struct {
    template <typename T>
    constexpr T operator()(T lhs, T rhs) const {
        return lhs / rhs;
    }
} div;

/**
 * Sign function.
 * Returns -1 if v is negative, +1 if it is positive, 0 otherwise.
 */
constexpr struct {
    template <typename T>
    constexpr T operator()(T v) const {
        return (T{0L} < v) - (v < T{0L});
    }
} sgn;

/**
 * Modulo for floating point types.
 */
constexpr struct {
    template <typename T>
    constexpr eif<std::is_floating_point<T>::value, T> operator()(T lhs, T rhs) const {
        return lhs - static_cast<long long>(lhs / rhs) * rhs;
    }
} fmod;

/**
 * Round.
 * Round down to nearest integer or nearest multiple of significance lesser or
 * equal to input.
 */
namespace detail {

template <typename T>
constexpr T round_impl(T value, std::true_type, std::true_type) {
    return static_cast<long long>(value + T{0.5L} * math::sgn(value));
}

template <typename T>
constexpr T round_impl(T value, std::false_type, std::false_type) {
    return value;
}

template <typename T>
constexpr T round_impl(T value, std::false_type, std::true_type) {
    return value;
}

} // namespace detail

constexpr struct {
    template <typename T>
    constexpr T operator()(T value) const {
        return detail::round_impl(value, std::is_floating_point<T>(), std::is_signed<T>());
    }
} round;

/**
 * Floor.
 * Round down to nearest integer or nearest multiple of significance lesser or
 * equal to input.
 */
namespace detail {

template <typename T>
constexpr T floor_impl(T value, std::true_type, std::true_type) {
    if (value < T{0L}) {
        T mod = value - static_cast<long long>(value);
        return value - (mod ? T{1L} + mod : T{0L});
    }

    return value - fmod(value, T{1L});
}

template <typename T>
constexpr T floor_impl(T value, std::false_type, std::false_type) {
    return value;
}

template <typename T>
constexpr T floor_impl(T value, std::false_type, std::true_type) {
    return value;
}

template <typename T>
constexpr T floor_impl(T value, T significance, std::true_type, std::true_type) {
    if (value < T{0L}) {
        T mod = fmod(value, significance);
        return value - (mod ? significance + mod : T{0L});
    }

    return value - fmod(value, significance);
}

template <typename T>
constexpr T floor_impl(T value, T significance, std::false_type, std::false_type) {
    return value - value % significance;
}

template <typename T>
constexpr T floor_impl(T value, T significance, std::false_type, std::true_type) {
    if (value < T{0L}) {
        T value_plus_one = value + T{1L};
        return value_plus_one - significance - value_plus_one % significance;
    }

    return value - value % significance;
}

} // namespace detail

constexpr struct {
    template <typename T>
    constexpr T operator()(T value) const {
        return detail::floor_impl(value, std::is_floating_point<T>(), std::is_signed<T>());
    }

    template <typename T>
    constexpr T operator()(T value, T significance) const {
        return detail::floor_impl(value, significance, std::is_floating_point<T>(), std::is_signed<T>());
    }
} floor;

/**
 * Ceil.
 * Round up to nearest integer or nearest multiple of significance greater or
 * equal to input.
 */
namespace detail {

template <typename T>
constexpr T ceil_impl(T value, std::true_type, std::true_type) {
    if (value > T{0L}) {
        T mod = value - static_cast<long long>(value);
        return value + (mod ? T{1L} - mod : T{0L});
    }

    return value - fmod(value, T{1L});
}

template <typename T>
constexpr T ceil_impl(T value, std::false_type, std::false_type) {
    return value;
}

template <typename T>
constexpr T ceil_impl(T value, std::false_type, std::true_type) {
    return value;
}

template <typename T>
constexpr T ceil_impl(T value, T significance, std::true_type, std::true_type) {
    if (value > T{0L}) {
        T mod = fmod(value, significance);
        return value + (mod ? significance - mod : T{0L});
    }

    return value - fmod(value, significance);
}

template <typename T>
constexpr T ceil_impl(T value, T significance, std::false_type, std::false_type) {
    T value_minus_one = value - T{1L};
    return value_minus_one + significance - value_minus_one % significance;
}

template <typename T>
constexpr T ceil_impl(T value, T significance, std::false_type, std::true_type) {
    if (value > T{0L}) {
        T value_minus_one = value - T{1L};
        return value_minus_one + significance - value_minus_one % significance;
    }

    return value - value % significance;
}

} // namespace detail

constexpr struct {
    template <typename T>
    constexpr T operator()(T value) const {
        return detail::ceil_impl(value, std::is_floating_point<T>(), std::is_signed<T>());
    }

    template <typename T>
    constexpr T operator()(T value, T significance) const {
        return detail::ceil_impl(value, significance, std::is_floating_point<T>(), std::is_signed<T>());
    }
} ceil;

/**
 * Clamp value between minimum and maximum.
 */
constexpr struct {
    template <typename T>
    constexpr T operator()(T value, T minimum, T maximum) const {
        T result = std::max(std::min(value, maximum), minimum);

        return result;
    }
} clamp;

/**
 * Absolute value.
 */
constexpr struct {
    template <typename T>
    constexpr T operator()(T v) const {
        return std::abs(v);
    }
} abs;

/**
 * Minimum of two values.
 */
constexpr struct {
    template <typename T>
    constexpr T operator()(T lhs, T rhs) const {
        return std::min(lhs, rhs);
    }
} min;

/**
 * Maximum of two values.
 */
constexpr struct {
    template <typename T>
    constexpr T operator()(T lhs, T rhs) const {
        return std::max(lhs, rhs);
    }
} max;

/**
 * Return linear interpolation bewteen v1 and v2 using weight.
 */
constexpr struct {
    template <typename T>
    constexpr T operator()(T v1, T v2, T weight) const {
        // Precise method which guarantees result is v2 when weight is 1.
        return v1 * (T{1L} - weight) + v2 * weight;

        // Imprecise method which does not guarantee result is v2 when weight is 1,
        // due to floating-point arithmetic error.
        //return v1 + weight * (v2 - v1);
    }
} lerp;

} // namespace math
} // namespace ee
