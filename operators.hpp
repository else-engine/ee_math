/**
 * Copyright (c) 2017-2018 Gauthier ARNOULD
 * This file is released under the zlib License (Zlib).
 * See file LICENSE or go to https://opensource.org/licenses/Zlib
 * for full license details.
 */

#pragma once

#include <ee_utils/componentwise.hpp>

#include "mat.hpp"
#include "vec.hpp"
#include "quat.hpp"
#include "common.hpp"
#include "functions.hpp"

namespace ee {
namespace math {

using tutil::eif;

/**
 * Comparison operators.
 */

/**
 * Equal.
 * For mat, vec and quat.
 */
template <typename T, typename = eif<is_mat<T> || is_vec<T> || is_quat<T>>>
constexpr bool operator==(const T& lhs, const T& rhs) {
    for (std::size_t i = 0; i < T::size; ++ i) {
        if (lhs.data[i] != rhs.data[i]) {
            return false;
        }
    }

    return true;
}

/**
 * Different.
 * For mat, vec and quat.
 */
template <typename T, typename = eif<is_mat<T> || is_vec<T> || is_quat<T>>>
constexpr bool operator!=(const T& lhs, const T& rhs) {
    return ! (lhs == rhs);
}

/**
 * Lesser.
 * For mat, vec and quat.
 * Lexicographical comparison useful when used with container which needs a way
 * to sort elements.
 */
template <typename T, typename = eif<is_mat<T> || is_vec<T> || is_quat<T>>>
constexpr bool operator<(const T& lhs, const T& rhs) {
    for (std::size_t i = 0; i < T::size; ++ i) {
        if (lhs.data[i] != rhs.data[i]) {
            return lhs.data[i] < rhs.data[i];
        }
    }

    return false;
}

/**
 * Greater.
 * For mat, vec and quat.
 */
template <typename T, typename = eif<is_mat<T> || is_vec<T> || is_quat<T>>>
constexpr bool operator>(const T& lhs, const T& rhs) {
    return operator<(rhs, lhs); // reverse order
}

/**
 * Lesser or equal.
 * For mat, vec and quat.
 */
template <typename T, typename = eif<is_mat<T> || is_vec<T> || is_quat<T>>>
constexpr bool operator<=(const T& lhs, const T& rhs) {
    return ! (lhs > rhs);
}

/**
 * Greater or equal.
 * For mat, vec and quat.
 */
template <typename T, typename = eif<is_mat<T> || is_vec<T> || is_quat<T>>>
constexpr bool operator>=(const T& lhs, const T& rhs) {
    return ! (lhs < rhs);
}

/**
 * Unary arithmetic operators.
 */

/**
 * +.
 * For mat and vec.
 * Yeah, does nothing...
 */
template <typename T, typename = eif<is_mat<T> || is_vec<T>>>
constexpr auto operator+(const T& rhs) {
    return rhs;
}

/**
 * Opposite.
 * For mat and vec.
 */
template <typename T, typename = eif<is_mat<T> || is_vec<T>>>
constexpr auto operator-(const T& rhs) {
    return cwise(opp, rhs);
}

/**
 * Binary arithmetic operators.
 */

/**
 * Matrix-matrix multiplication.
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
 * Matrix-vector multiplication.
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
 * Scalar multiplication.
 * For mat and vec.
 */
template <typename LT, typename RT, typename = eif<(is_mat<LT> || is_vec<LT>) && is_num<RT>>>
constexpr auto operator*(const LT& lhs, RT rhs) {
    return cwise(mul, lhs, rhs);
}

/**
 * Scalar multiplication.
 * For mat and vec.
 */
template <typename LT, typename RT, typename = eif<is_num<LT> && (is_mat<RT> || is_vec<RT>)>>
constexpr auto operator*(LT lhs, const RT& rhs) {
    return cwise(mul, lhs, rhs);
}

/**
 * Scalar division.
 * For mat and vec.
 */
template <typename LT, typename RT, typename = eif<(is_mat<LT> || is_vec<LT>) && is_num<RT>>>
constexpr auto operator/(const LT& lhs, RT rhs) {
    return cwise(div, lhs, rhs);
}

/**
 * Addition.
 * For mat and vec.
 */
template <typename T, typename = eif<is_mat<T> || is_vec<T>>>
constexpr auto operator+(const T& lhs, const T& rhs) {
    return cwise(add, lhs, rhs);
}

/**
 * Subtraction.
 * For mat and vec.
 */
template <typename T, typename = eif<is_mat<T> || is_vec<T>>>
constexpr auto operator-(const T& lhs, const T& rhs) {
    return cwise(sub, lhs, rhs);
}

/**
 * Arithmetic assignment operators or compound operators.
 */

/**
 * Matrix-matrix multiplication.
 */
template <typename T, std::size_t R, std::size_t C>
constexpr const auto& operator*=(mat<T, R, C>& lhs, const mat<T, R, C>& rhs) {
    lhs = lhs * rhs;

    return lhs;
}

/**
 * Scalar multiplication.
 * For mat and vec.
 */
template <typename LT, typename RT, typename = eif<(is_mat<LT> || is_vec<LT>) && is_num<RT>>>
constexpr const auto& operator*=(LT& lhs, RT rhs) {
    return lhs = cwise(mul, lhs, rhs);
}

/**
 * Scalar division.
 * For mat and vec.
 */
template <typename LT, typename RT, typename = eif<(is_mat<LT> || is_vec<LT>) && is_num<RT>>>
constexpr const auto& operator/=(LT& lhs, RT rhs) {
    return lhs = cwise(div, lhs, rhs);
}

/**
 * Addition.
 * For mat and vec.
 */
template <typename T, typename = eif<is_mat<T> || is_vec<T>>>
constexpr const auto& operator+=(T& lhs, const T& rhs) {
    return lhs = cwise(add, lhs, rhs);
}

/**
 * Subtraction.
 * For mat and vec.
 */
template <typename T, typename = eif<is_mat<T> || is_vec<T>>>
constexpr const auto& operator-=(T& lhs, const T& rhs) {
    return lhs = cwise(sub, lhs, rhs);
}

/**
 * Bit shift operators
 */

/**
 * Leftshift assignment.
 * For mat and vec.
 */
template <typename LT, typename RT, typename = eif<(is_mat<LT> || is_vec<LT>) && is_num<RT>>>
constexpr const auto& operator<<=(LT& lhs, RT rhs) {
    return lhs = cwise(lshift, lhs, rhs);
}

/**
 * Leftshift.
 * For mat and vec.
 */
template <typename LT, typename RT, typename = eif<(is_mat<LT> || is_vec<LT>) && is_num<RT>>>
constexpr auto operator<<(const LT& lhs, RT rhs) {
    return cwise(lshift, lhs, rhs);
}

/**
 * Rightshift assignment.
 * For mat and vec.
 */
template <typename LT, typename RT, typename = eif<(is_mat<LT> || is_vec<LT>) && is_num<RT>>>
constexpr const auto& operator>>=(LT& lhs, RT rhs) {
    return lhs = cwise(rshift, lhs, rhs);
}

/**
 * Rightshift.
 * For mat and vec.
 */
template <typename LT, typename RT, typename = eif<(is_mat<LT> || is_vec<LT>) && is_num<RT>>>
constexpr auto operator>>(const LT& lhs, RT rhs) {
    return cwise(rshift, lhs, rhs);
}

} // namespace math
} // namespace ee
