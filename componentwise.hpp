/**
 * Copyright (c) 2017 Gauthier ARNOULD
 * This file is released under the zlib License (Zlib).
 * See file LICENSE or go to https://opensource.org/licenses/Zlib
 * for full license details.
 */

#pragma once

#include <ee_utils/templates.hpp>

#include "common.hpp"
#include "mat.hpp"
#include "vec.hpp"

namespace ee {
namespace cw {

using tutil::eif;
using math::is_num;
using math::mat;
using math::is_mat;
using math::vec;
using math::is_vec;

namespace detail {

template <typename O, std::size_t... Is>
constexpr auto mul(const O& lhs, const O& rhs, std::index_sequence<Is...>) {
    return O{lhs.data[Is] * rhs.data[Is]...};
}

template <typename O, typename T, std::size_t... Is>
constexpr auto mul_scalar(const O& lhs, T rhs, std::index_sequence<Is...>) {
    return O{lhs.data[Is] * rhs...};
}

} // namespace detail

/**
 * Componentwise multiplication.
 * mat * mat
 * vec * vec
 */
template <typename T, typename = eif<is_mat<T> || is_vec<T>>>
constexpr auto mul(const T& lhs, const T& rhs) {
    return detail::mul(lhs, rhs, std::make_index_sequence<T::size>());
}

/**
 * Componentwise multiplication.
 * scalar * mat
 * scalar * vec
 */
template <typename LT, typename RT, typename = eif<is_num<LT> && (is_mat<RT> || is_vec<RT>)>>
constexpr auto mul(LT lhs, const RT& rhs) {
    return detail::mul_scalar(rhs, lhs, std::make_index_sequence<RT::size>());
}

/**
 * Componentwise multiplication.
 * mat * scalar
 * vec * scalar
 */
template <typename LT, typename RT, typename = eif<(is_mat<LT> || is_vec<LT>) && is_num<RT>>>
constexpr auto mul(const LT& lhs, RT rhs) {
    return detail::mul_scalar(lhs, rhs, std::make_index_sequence<LT::size>());
}

namespace detail {

template <typename O, std::size_t... Is>
constexpr auto div(const O& lhs, const O& rhs, std::index_sequence<Is...>) {
    return O{lhs.data[Is] / rhs.data[Is]...};
}

template <typename O, typename T, std::size_t... Is, typename = eif<is_num<T>>>
constexpr auto div_scalar(const O& lhs, T rhs, std::index_sequence<Is...>) {
    return O{lhs.data[Is] / rhs...};
}

template <typename T, typename O, std::size_t... Is, typename = eif<is_num<T>>>
constexpr auto div_scalar(T lhs, const O& rhs, std::index_sequence<Is...>) {
    return O{lhs / rhs.data[Is]...};
}

} // namespace detail

/**
 * Componentwise division.
 * mat / mat
 * vec / vec
 */
template <typename T, typename = eif<is_mat<T> || is_vec<T>>>
constexpr auto div(const T& lhs, const T& rhs) {
    return detail::div(lhs, rhs, std::make_index_sequence<T::size>());
}

/**
 * Componentwise division.
 * mat / scalar
 * vec / scalar
 */
template <typename LT, typename RT, typename = eif<(is_mat<LT> || is_vec<LT>) && is_num<RT>>>
constexpr auto div(const LT& lhs, RT rhs) {
    return detail::div_scalar(lhs, rhs, std::make_index_sequence<LT::size>());
}

/**
 * Componentwise division.
 * scalar / mat
 * scalar / vec
 */
template <typename LT, typename RT, typename = eif<is_num<LT> && (is_mat<RT> || is_vec<RT>)>>
constexpr auto div(LT lhs, const RT& rhs) {
    return detail::div_scalar(lhs, rhs, std::make_index_sequence<RT::size>());
}

namespace detail {

template <typename O, std::size_t... Is>
constexpr auto add(const O& lhs, const O& rhs, std::index_sequence<Is...>) {
    return O{lhs.data[Is] + rhs.data[Is]...};
}

template <typename O, typename T, std::size_t... Is>
constexpr auto add_scalar(const O& lhs, T rhs, std::index_sequence<Is...>) {
    return O{lhs.data[Is] + rhs...};
}

} // namespace detail

/**
 * Componentwise addition.
 * mat + mat
 * vec + vec
 */
template <typename T, typename = eif<is_mat<T> || is_vec<T>>>
constexpr auto add(const T& lhs, const T& rhs) {
    return detail::add(lhs, rhs, std::make_index_sequence<T::size>());
}

/**
 * Componentwise addition.
 * scalar + mat
 * scalar + vec
 */
template <typename LT, typename RT, typename = eif<is_num<LT> && (is_mat<RT> || is_vec<RT>)>>
constexpr auto add(LT lhs, const RT& rhs) {
    return detail::add_scalar(rhs, lhs, std::make_index_sequence<RT::size>());
}

/**
 * Componentwise addition.
 * mat + scalar
 * vec + scalar
 */
template <typename LT, typename RT, typename = eif<(is_mat<LT> || is_vec<LT>) && is_num<RT>>>
constexpr auto add(const LT& lhs, RT rhs) {
    return detail::add_scalar(lhs, rhs, std::make_index_sequence<LT::size>());
}

/**
 * Componentwise addition in place.
 * mat += scalar
 * vec += scalar
 */
template <typename LT, typename RT, typename = eif<(is_mat<LT> || is_vec<LT>) && is_num<RT>>>
constexpr void add(LT* lhs, RT rhs) {
    for (std::size_t i = 0; i < LT::size; ++ i) {
        lhs->data[i] += rhs;
    }
}

namespace detail {

template <typename O, std::size_t... Is>
constexpr auto sub(const O& lhs, const O& rhs, std::index_sequence<Is...>) {
    return O{lhs.data[Is] - rhs.data[Is]...};
}

template <typename O, typename T, std::size_t... Is, typename = eif<is_num<T>>>
constexpr auto sub_scalar(const O& lhs, T rhs, std::index_sequence<Is...>) {
    return O{lhs.data[Is] - rhs...};
}

template <typename T, typename O, std::size_t... Is, typename = eif<is_num<T>>>
constexpr auto sub_scalar(T lhs, const O& rhs, std::index_sequence<Is...>) {
    return O{lhs - rhs.data[Is]...};
}

} // namespace detail

/**
 * Componentwise subtraction.
 * mat - mat
 * vec - vec
 */
template <typename T, typename = eif<is_mat<T> || is_vec<T>>>
constexpr auto sub(const T& lhs, const T& rhs) {
    return detail::sub(lhs, rhs, std::make_index_sequence<T::size>());
}

/**
 * Componentwise subtraction.
 * mat - scalar
 * vec - scalar
 */
template <typename LT, typename RT, typename = eif<(is_mat<LT> || is_vec<LT>) && is_num<RT>>>
constexpr auto sub(const LT& lhs, RT rhs) {
    return detail::sub_scalar(lhs, rhs, std::make_index_sequence<LT::size>());
}

/**
 * Componentwise subtraction.
 * scalar - mat
 * scalar - vec
 */
template <typename LT, typename RT, typename = eif<is_num<LT> && (is_mat<RT> || is_vec<RT>)>>
constexpr auto sub(LT lhs, const RT& rhs) {
    return detail::sub_scalar(lhs, rhs, std::make_index_sequence<RT::size>());
}

} // namespace cw
} // namespace ee
