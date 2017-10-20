/**
 * Copyright (c) 2017 Gauthier ARNOULD
 * This file is released under the zlib License (Zlib).
 * See file LICENSE or go to https://opensource.org/licenses/Zlib
 * for full license details.
 */

#pragma once

#include "mat.hpp"
#include "vec.hpp"
#include "quat.hpp"
#include "generators.hpp"
#include "iterators.hpp"

namespace ee {
namespace math {

using tutil::eif;
using tutil::is_arithmetic;

/**
 * Comparison operators.
 */

/**
 * Equal.
 * mat == mat
 */
template <typename T, std::size_t R, std::size_t C>
constexpr bool operator==(const mat<T, R, C>& lhs, const mat<T, R, C>& rhs) {
    for (std::size_t r = 0; r < R; ++ r) {
        for (std::size_t c = 0; c < C; ++ c) {
            if (lhs(r, c) != rhs(r, c)) {
                return false;
            }
        }
    }

    return true;
}

/**
 * Equal.
 * vec == vec
 */
template <typename T, std::size_t D>
constexpr bool operator==(const vec<T, D>& lhs, const vec<T, D>& rhs) {
    for (std::size_t d = 0; d < D; ++ d) {
        if (lhs(d) != rhs(d)) {
            return false;
        }
    }

    return true;
}

/**
 * Equal.
 * quat == quat
 */
template <typename T>
constexpr bool operator==(const quat<T>& lhs, const quat<T>& rhs) {
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z && lhs.w == rhs.w;
}

/**
 * Different.
 * mat != mat
 */
template <typename T, std::size_t R, std::size_t C>
constexpr bool operator!=(const mat<T, R, C>& lhs, const mat<T, R, C>& rhs) {
    return ! (lhs == rhs);
}

/**
 * Different.
 * vec != vec
 */
template <typename T, std::size_t D>
constexpr bool operator!=(const vec<T, D>& lhs, const vec<T, D>& rhs) {
    return ! (lhs == rhs);
}

/**
 * Different.
 * quat != quat
 */
template <typename T>
constexpr bool operator!=(const quat<T>& lhs, const quat<T>& rhs) {
    return ! (lhs == rhs);
}

/**
 * Lesser than.
 * mat < mat
 * Lexicographical comparison useful when used with container which needs a way
 * to sort elements.
 */
template <typename T, std::size_t R, std::size_t C>
constexpr bool operator<(const mat<T, R, C>& lhs, const mat<T, R, C>& rhs) {
    for (unsigned int r = 0; r < R; ++ r) {
        for (unsigned int c = 0; c < C; ++ c) {
            if (lhs(r, c) != rhs(r, c)) {
                return lhs(r, c) < rhs(r, c);
            }
        }
    }

    return false;
}

/**
 * Lesser than.
 * vec < vec
 * Lexicographical comparison useful when used with container which needs a way
 * to sort elements.
 */
template <typename T, std::size_t D>
constexpr bool operator<(const vec<T, D>& lhs, const vec<T, D>& rhs) {
    for (unsigned int d = 0; d < D; ++ d) {
        if (lhs(d) != rhs(d)) {
            return lhs(d) < rhs(d);
        }
    }

    return false;
}

/**
 * Greater than.
 * mat > mat
 */
template <typename T, std::size_t R, std::size_t C>
constexpr bool operator>(const mat<T, R, C>& lhs, const mat<T, R, C>& rhs) {
    return operator<(rhs, lhs); // reverse order
}

/**
 * Greater than.
 * vec > vec
 */
template <typename T, std::size_t D>
constexpr bool operator>(const vec<T, D>& lhs, const vec<T, D>& rhs) {
    return operator<(rhs, lhs); // reverse order
}

/**
 * Lesser or equal to.
 * mat <= mat
 */
template <typename T, std::size_t R, std::size_t C>
constexpr bool operator<=(const mat<T, R, C>& lhs, const mat<T, R, C>& rhs) {
    return ! (lhs > rhs);
}

/**
 * Lesser or equal to.
 * vec <= vec
 */
template <typename T, std::size_t D>
constexpr bool operator<=(const vec<T, D>& lhs, const vec<T, D>& rhs) {
    return ! (lhs > rhs);
}

/**
 * Greater or equal to.
 * mat >= mat
 */
template <typename T, std::size_t R, std::size_t C>
constexpr bool operator>=(const mat<T, R, C>& lhs, const mat<T, R, C>& rhs) {
    return ! (lhs < rhs);
}

/**
 * Greater or equal to.
 * vec >= vec
 */
template <typename T, std::size_t D>
constexpr bool operator>=(const vec<T, D>& lhs, const vec<T, D>& rhs) {
    return ! (lhs < rhs);
}

/**
 * Unary arithmetic operators.
 */

namespace detail {

template <typename T, std::size_t... Is>
constexpr auto opposite(const T& t, std::index_sequence<Is...>) {
    return T{ - t.data[Is]...};
}

} // namespace detail

/**
 * Opposite.
 * - mat
 */
template <typename T, std::size_t R, std::size_t C>
constexpr auto operator-(const mat<T, R, C>& rhs) {
    return detail::opposite(rhs, std::make_index_sequence<R * C>());
}

/**
 * Opposite.
 * - vec
 */
template <typename T, std::size_t D>
constexpr auto operator-(const vec<T, D>& rhs) {
    return detail::opposite(rhs, std::make_index_sequence<D>());
}

/**
 * Binary arithmetic operators.
 */

/**
 * Multiplication.
 * mat * mat
 */
template <typename T, std::size_t R, std::size_t LC, std::size_t RC>
constexpr auto operator*(const mat<T, R, LC>& lhs, const mat<T, LC, RC>& rhs) {
    mat<T, R, RC> result{};

    for (std::size_t k = 0; k < RC; ++ k) {
        for (std::size_t j = 0; j < R; ++ j) {
            for (std::size_t i = 0; i < LC; ++ i) {
                // FIXME : when called from constexpr context, gcc 5.4 gives :
                // error: ‘#‘result_decl’ not supported by dump_expr#<expression
                // error>.ee::math::mat<T, R, C>::operator()<float, 4ul, 4ul>(j,
                // k)’ is not a constant expression.
                // Clang works without problem with this.
                result(j, k) += lhs(j, i) * rhs(i, k);
            }
        }
    }

    return result;
}

/**
 * Multiplication.
 * mat * vec
 */
template <typename T, std::size_t R, std::size_t C>
constexpr auto operator*(const mat<T, R, C>& lhs, const vec<T, C>& rhs) {
    vec<T, R> result{};

    for (std::size_t r = 0; r < R; ++ r) {
        for (std::size_t c = 0; c < C; ++ c) {
            // FIXME : when called from constexpr context, gcc 5.4 gives :
            // error: ‘result.ee::math::vec<T, 3ul>::operator()<float>(r)’ is
            // not a constant expression.
            // Clang works without problem with this.
            result(r) += lhs(r, c) * rhs(c);
        }
    }

    return result;
}

/**
 * Multiplication.
 * mat * arithmetic
 */
template <typename LT, std::size_t R, std::size_t C, typename RT, typename = eif<is_arithmetic<RT>>>
constexpr auto operator*(const mat<LT, R, C>& lhs, RT rhs) {
    mat<LT, R, C> result{lhs};

    result *= rhs;

    return result;
}

/**
 * Multiplication.
 * arithmetic * mat
 */
template <typename LT, typename RT, std::size_t R, std::size_t C, typename = eif<is_arithmetic<LT>>>
constexpr auto operator*(LT lhs, const mat<RT, R, C>& rhs) {
    return rhs * lhs; // reverse order
}

/**
 * Multiplication.
 * vec * arithmetic
 */
template <typename LT, std::size_t D, typename RT, typename = eif<is_arithmetic<RT>>>
constexpr auto operator*(const vec<LT, D>& lhs, RT rhs) {
    vec<LT, D> result{lhs};

    result *= rhs;

    return result;
}

/**
 * Multiplication.
 * arithmetic * vec
 */
template <typename LT, typename RT, std::size_t D, typename = eif<is_arithmetic<LT>>>
constexpr auto operator*(LT lhs, const vec<RT, D>& rhs) {
    return rhs * lhs; // reverse order
}

/**
 * Division.
 * mat / arithmetic
 */
template <typename LT, std::size_t R, std::size_t C, typename RT, typename = eif<is_arithmetic<RT>>>
constexpr auto operator/(const mat<LT, R, C>& lhs, RT rhs) {
    mat<LT, R, C> result{lhs};

    result /= rhs;

    return result;
}

/**
 * Division.
 * vec / arithmetic
 */
template <typename LT, std::size_t D, typename RT, typename = eif<is_arithmetic<RT>>>
constexpr auto operator/(const vec<LT, D>& lhs, RT rhs) {
    vec<LT, D> result{lhs};

    result /= rhs;

    return result;
}

/**
 * Addition.
 * mat + mat
 */
template <typename T, std::size_t R, std::size_t C>
constexpr auto operator+(const mat<T, R, C>& lhs, const mat<T, R, C>& rhs) {
    mat<T, R, C> result{lhs};

    result += rhs;

    return result;
}

/**
 * Addition.
 * vec + vec
 */
template <typename T, std::size_t D>
constexpr auto operator+(const vec<T, D>& lhs, const vec<T, D>& rhs) {
    vec<T, D> result{lhs};

    result += rhs;

    return result;
}

/**
 * Subtraction.
 * mat - mat
 */
template <typename T, std::size_t R, std::size_t C>
constexpr auto operator-(const mat<T, R, C>& lhs, const mat<T, R, C>& rhs) {
    mat<T, R, C> result{lhs};

    result -= rhs;

    return result;
}

/**
 * Subtraction.
 * vec - vec
 */
template <typename T, std::size_t D>
constexpr auto operator-(const vec<T, D>& lhs, const vec<T, D>& rhs) {
    vec<T, D> result{lhs};

    result -= rhs;

    return result;
}

/**
 * Arithmetic assignment operators or compound operators.
 */

/**
 * Multiplication.
 * mat *= arithmetic
 */
template <typename LT, std::size_t R, std::size_t C, typename RT, typename = eif<is_arithmetic<RT>>>
constexpr const auto& operator*=(mat<LT, R, C>& lhs, RT rhs) {
    for (std::size_t i = 0; i < R * C; ++ i) {
        lhs.data[i] *= rhs;
    }

    return lhs;
}

/**
 * Multiplication.
 * vec *= arithmetic
 */
template <typename LT, std::size_t D, typename RT, typename = eif<is_arithmetic<RT>>>
constexpr const auto& operator*=(vec<LT, D> & lhs, RT rhs) {
    for (std::size_t i = 0; i < D; ++ i) {
        lhs.data[i] *= rhs;
    }

    return lhs;
}

/**
 * Multiplication.
 * mat *= mat
 */
template <typename T, std::size_t R, std::size_t C>
constexpr const auto& operator*=(mat<T, R, C>& lhs, const mat<T, R, C>& rhs) {
    lhs = lhs * rhs;

    return lhs;
}

/**
 * Division.
 * mat /= arithmetic
 */
template <typename LT, std::size_t R, std::size_t C, typename RT, typename = eif<is_arithmetic<RT>>>
constexpr const auto& operator/=(mat<LT, R, C>& lhs, RT rhs) {
    for (std::size_t i = 0; i < R * C; ++ i) {
        lhs.data[i] /= rhs;
    }

    return lhs;
}

/**
 * Division.
 * vec /= arithmetic
 */
template <typename LT, std::size_t D, typename RT, typename = eif<is_arithmetic<RT>>>
constexpr const auto& operator/=(vec<LT, D>& lhs, RT rhs) {
    for (std::size_t i = 0; i < D; ++ i) {
        lhs.data[i] /= rhs;
    }

    return lhs;
}

/**
 * Addition.
 * mat += mat
 */
template <typename T, std::size_t R, std::size_t C>
constexpr const auto& operator+=(mat<T, R, C>& lhs, const mat<T, R, C>& rhs) {
    for (std::size_t i = 0; i < R * C; ++ i) {
        lhs.data[i] += rhs.data[i];
    }

    return lhs;
}

/**
 * Addition.
 * vec += vec
 */
template <typename T, std::size_t D>
constexpr const auto& operator+=(vec<T, D>& lhs, const vec<T, D>& rhs) {
    for (std::size_t i = 0; i < D; ++ i) {
        lhs.data[i] += rhs.data[i];
    }

    return lhs;
}

/**
 * Subtraction.
 * mat -= mat
 */
template <typename T, std::size_t R, std::size_t C>
constexpr const auto& operator-=(mat<T, R, C>& lhs, const mat<T, R, C>& rhs) {
    for (std::size_t i = 0; i < R * C; ++ i) {
        lhs.data[i] -= rhs.data[i];
    }

    return lhs;
}

/**
 * Subtraction.
 * vec -= vec
 */
template <typename T, std::size_t D>
constexpr const auto& operator-=(vec<T, D>& lhs, const vec<T, D>& rhs) {
    for (std::size_t i = 0; i < D; ++ i) {
        lhs.data[i] -= rhs.data[i];
    }

    return lhs;
}

} // namespace math
} // namespace ee
