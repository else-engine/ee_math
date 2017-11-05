/**
 * Copyright (c) 2017 Gauthier ARNOULD
 * This file is released under the zlib License (Zlib).
 * See file LICENSE or go to https://opensource.org/licenses/Zlib
 * for full license details.
 */

#pragma once

#include <utility>

#include "mat.hpp"
#include "quat.hpp"

namespace ee {
namespace math {

/**
 * Defines constants as templates of the required type.
 * Constant name always start with "c_".
 */

#define EE_CONSTANT(n, v)\
template <typename T>\
constexpr static T n = v

EE_CONSTANT(c_pi  ,   3.1415926535897932384626433832795028841971693993751L);
// gcc allows this but the standard says acos (among others) is not constexpr.
//EE_CONSTANT(c_pi, std::acos(-1.0L));

EE_CONSTANT(c_two_pi     ,   T{2L} * c_pi<T>);
EE_CONSTANT(c_half_pi    , T{0.5L} * c_pi<T>);
EE_CONSTANT(c_pi_over_180, c_pi<T> / T{180L});
EE_CONSTANT(c_180_over_pi, T{180L} / c_pi<T>);

#undef EE_CONSTANT

/**
 * Identities
 */
namespace detail {

template <typename T, std::size_t R, std::size_t C, std::size_t... Is>
constexpr auto identity(std::index_sequence<Is...>) {
    return mat<T, R, C>{(mat_i_to_r(Is, {R, C}) == mat_i_to_c(Is, {R, C}) ? T{1L} : T{0L})...};
}

} // namespace detail

// Generic identity
// Cannot be instantiated, only there to allow specializations below.
template <typename T>
constexpr std::enable_if_t<false, T> c_identity{};

// Matrix identity specialization
template <typename T, std::size_t R, std::size_t C>
constexpr auto c_identity<mat<T, R, C>> =
detail::identity<T, R, C>(std::make_index_sequence<R * C>());

// Quaternion identity specialization
template <typename T>
constexpr auto c_identity<quat<T>> =
math::quat<T>{T{0L}, T{0L}, T{0L}, T{1L}};

} // namespace math
} // namespace ee