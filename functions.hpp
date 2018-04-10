/**
 * Copyright (c) 2017-2018 Gauthier ARNOULD
 * This file is released under the zlib License (Zlib).
 * See file LICENSE or go to https://opensource.org/licenses/Zlib
 * for full license details.
 */

#pragma once

#include <cmath>

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
    template <typename... Ts>
    constexpr auto operator()(Ts... vs) const noexcept {
        return (vs + ...);
    }
} add;

/**
 * Subtraction.
 */
constexpr struct {
    template <typename T>
    constexpr T operator()(T lhs, T rhs) const noexcept {
        return lhs - rhs;
    }
} sub;

/**
 * Multiplication.
 */
constexpr struct {
    template <typename... Ts>
    constexpr auto operator()(Ts... vs) const noexcept {
        return (vs * ...);
    }
} mul;

/**
 * Division.
 */
constexpr struct {
    template <typename T>
    constexpr T operator()(T lhs, T rhs) const noexcept {
        return lhs / rhs;
    }
} div;


/**
 * Opposite.
 */
constexpr struct {
    template <typename T>
    constexpr T operator()(T rhs) const noexcept {
        return - rhs;
    }
} opp;

/**
 * Bitwise left shift.
 */
constexpr struct {
    template <typename T>
    constexpr T operator()(T lhs, T rhs) const noexcept {
        return lhs << rhs;
    }
} lshift;

/**
 * Bitwise right shift.
 */
constexpr struct {
    template <typename T>
    constexpr T operator()(T lhs, T rhs) const noexcept {
        return lhs >> rhs;
    }
} rshift;

/**
 * Sign function.
 * Returns -1 if v is negative, +1 if it is positive, 0 otherwise.
 */
constexpr struct {
    template <typename T>
    constexpr T operator()(T v) const noexcept {
        return (T{0L} < v) - (v < T{0L});
    }
} sgn;

/**
 * Trunc.
 */
constexpr struct {
    template <typename T>
    /*constexpr*/ eif<std::is_floating_point<T>::value, T> operator()(T value) const noexcept {
        return std::trunc(value);
    }

    template <typename T>
    constexpr eif< ! std::is_floating_point<T>::value, T> operator()(T value) const noexcept {
        return value;
    }
} trunc;

/**
 * Modulo.
 */
constexpr struct {
    template <typename T>
    /*constexpr*/ eif<std::is_floating_point<T>::value, T> operator()(T lhs, T rhs) const noexcept {
        return std::remainder(lhs, rhs);
        //return lhs - trunc(lhs / rhs) * rhs;
    }

    template <typename T>
    constexpr eif< ! std::is_floating_point<T>::value, T> operator()(T lhs, T rhs) const noexcept {
        return lhs % rhs;
    }
} mod;

/**
 * Round.
 */
constexpr struct {
    template <typename T>
    /*constexpr*/ eif<std::is_floating_point<T>::value, T> operator()(T value) const noexcept {
        return std::round(value);
        //return trunc(value + T{0.5L} * sgn(value));
    }

    template <typename T>
    constexpr eif< ! std::is_floating_point<T>::value, T> operator()(T value) const noexcept {
        return value;
    }
} round;

/**
 * Floor.
 * Round down to nearest integer or nearest multiple of significance lesser or
 * equal to input.
 */
constexpr struct {
    template <typename T>
    /*constexpr*/ eif<std::is_floating_point<T>::value, T> operator()(T value) const noexcept {
        return std::floor(value);
        //T remainder = value - trunc(value);
        //return value - remainder - (remainder < T{0L} ? T{1L} : T{0L});
    }

    template <typename T>
    constexpr eif< ! std::is_floating_point<T>::value, T> operator()(T value) const noexcept {
        return value;
    }

    template <typename T>
    /*constexpr*/ eif<std::is_floating_point<T>::value, T> operator()(T value, T significance) const noexcept {
        T remainder = mod(value, significance);
        return value - remainder - (remainder < T{0L} ? significance : T{0L});
    }

    template <typename T>
    constexpr eif< ! std::is_floating_point<T>::value, T> operator()(T value, T significance) const noexcept {
#if 0
        T remainder = mod(value, significance);
        return value - remainder - (remainder < T{0L} ? significance : T{0L});
#else
        value += value < T{0L} ? T{1L} - significance : T{0L};
        return value - mod(value, significance);
#endif
    }
} floor;

/**
 * Ceil.
 * Round up to nearest integer or nearest multiple of significance greater or
 * equal to input.
 */
constexpr struct {
    template <typename T>
    /*constexpr*/ eif<std::is_floating_point<T>::value, T> operator()(T value) const noexcept {
        return std::ceil(value);
        //T remainder = value - trunc(value);
        //return value - remainder + (remainder > T{0L} ? T{1L} : T{0L});
    }

    template <typename T>
    constexpr eif< ! std::is_floating_point<T>::value, T> operator()(T value) const noexcept {
        return value;
    }

    template <typename T>
    /*constexpr*/ eif<std::is_floating_point<T>::value, T> operator()(T value, T significance) const noexcept {
        T remainder = mod(value, significance);
        return value - remainder + (remainder > T{0L} ? significance : T{0L});
    }

    template <typename T>
    constexpr eif< ! std::is_floating_point<T>::value, T> operator()(T value, T significance) const noexcept {
#if 0
        T remainder = mod(value, significance);
        return value - remainder + (remainder > T{0L} ? significance : T{0L});
#else
        value += value >= T{0L} ? significance - T{1L} : T{0L};
        return value - mod(value, significance);
#endif
    }
} ceil;

/**
 * Absolute value.
 */
constexpr struct {
    template <typename T>
    /*constexpr*/ eif< ! std::is_unsigned<T>::value, T> operator()(T value) const noexcept {
        return std::abs(value);
    }

    template <typename T>
    constexpr eif<std::is_unsigned<T>::value, T> operator()(T value) const noexcept {
        return value;
    }
} abs;

/**
 * Minimum from a list of values.
 */
constexpr struct {
    template <typename T>
    constexpr const T& operator()(const T& f) const noexcept {
        return f;
    }

    template <typename T, typename... Rs>
    constexpr const T& operator()(const T& f, const T& s, const Rs&... rs) const noexcept {
        return (*this)(s < f ? s : f, rs...);;
    }
} min;

/**
 * Maximum from a list of values.
 */
constexpr struct {
    template <typename T>
    constexpr const T& operator()(const T& f) const noexcept {
        return f;
    }

    template <typename T, typename... Rs>
    constexpr const T& operator()(const T& f, const T& s, const Rs&... rs) const noexcept {
        return (*this)(f < s ? s : f, rs...);
    }
} max;

/**
 * Clamp value between lo and hi.
 */
constexpr struct {
    template <typename T>
    constexpr const T& operator()(const T& v, const T& lo, const T& hi) const noexcept {
        return lo < v ? hi < v ? hi : v : lo;
    }
} clamp;

/**
 * Return linear interpolation bewteen v1 and v2 using weight.
 */
constexpr struct {
    template <typename T>
    constexpr T operator()(T v1, T v2, T weight) const noexcept {
        // Precise method which guarantees result is v2 when weight is 1.
        return v1 * (T{1L} - weight) + v2 * weight;

        // Imprecise method which does not guarantee result is v2 when weight is 1,
        // due to floating-point arithmetic error.
        //return v1 + weight * (v2 - v1);
    }
} lerp;

} // namespace math
} // namespace ee
