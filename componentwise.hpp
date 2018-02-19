/**
 * Copyright (c) 2017 Gauthier ARNOULD
 * This file is released under the zlib License (Zlib).
 * See file LICENSE or go to https://opensource.org/licenses/Zlib
 * for full license details.
 */

#pragma once

#include <type_traits>

#include <ee_utils/templates.hpp>

#include "common.hpp"
#include "mat.hpp"
#include "vec.hpp"
#include "quat.hpp"

namespace ee {

using tutil::eif;

namespace math {

/**
 * is_tuple. in the mathematical meaning.
 * To be considered as tuple a class needs :
 *  - a constexpr static member "size" reporting the number of components.
 *  - a type member "value_type", the type of the components.
 *  - a subscript operator which returns value_type references from 0 to size-1.
 *  - TODO to be an aggregate (needs std::is_aggregate)
 */
namespace detail {

template <typename T, typename = void>
struct is_tuple_impl : std::false_type {};

template <typename T>
struct is_tuple_impl<T, eif<
tutil::has_subscript_operator<typename T::reference (T::*)(std::size_t)> &&
tutil::has_subscript_operator<typename T::const_reference (T::*)(std::size_t) const> &&
T::size * sizeof(typename T::value_type) == sizeof(T)
>> : std::true_type {};

} // namespace detail

template <typename T>
constexpr bool is_tuple = detail::is_tuple_impl<T>::value;

/**
 * Allow calling functions componentwise.
 */
namespace detail {

/* 1 parameter */
// tuple
template <typename F, typename T, std::size_t... Is>
constexpr eif<is_tuple<T>, T> cwise(F&& f, const T& t1, std::index_sequence<Is...>) {
    return T{std::forward<F>(f)(t1[Is])...};
}

/* 2 parameters */
// tuple, tuple
template <typename F, typename T, std::size_t... Is>
constexpr eif<is_tuple<T>, T> cwise(F&& f, const T& t1, const T& t2, std::index_sequence<Is...>) {
    return T{std::forward<F>(f)(t1[Is], t2[Is])...};
}

// tuple, scalar
template <typename F, typename T, typename S, std::size_t... Is>
constexpr eif<is_tuple<T> && is_num<S>, T> cwise(F&& f, const T& t1, S s2, std::index_sequence<Is...>) {
    return {std::forward<F>(f)(t1[Is], s2)...};
}

// scalar, tuple
template <typename F, typename T, typename S, std::size_t... Is>
constexpr eif<is_tuple<T> && is_num<S>, T> cwise(F&& f, S s1, const T& t2, std::index_sequence<Is...>) {
    return {std::forward<F>(f)(s1, t2[Is])...};
}

/* 3 parameters */
// tuple, tuple, tuple
template <typename F, typename T, std::size_t... Is>
constexpr eif<is_tuple<T>, T> cwise(F&& f, const T& t1, const T& t2, const T& t3, std::index_sequence<Is...>) {
    return T{std::forward<F>(f)(t1[Is], t2[Is], t3[Is])...};
}

// tuple, tuple, scalar
template <typename F, typename T, typename S, std::size_t... Is>
constexpr eif<is_tuple<T> && is_num<S>, T> cwise(F&& f, const T& t1, const T& t2, S s3, std::index_sequence<Is...>) {
    return {std::forward<F>(f)(t1[Is], t2[Is], s3)...};
}

// tuple, scalar, tuple
template <typename F, typename T, typename S, std::size_t... Is>
constexpr eif<is_tuple<T> && is_num<S>, T> cwise(F&& f, const T& t1, S s2, const T& t3, std::index_sequence<Is...>) {
    return {std::forward<F>(f)(t1[Is], s2, t3[Is])...};
}

// scalar, tuple, tuple
template <typename F, typename T, typename S, std::size_t... Is>
constexpr eif<is_tuple<T> && is_num<S>, T> cwise(F&& f, S s1, const T& t2, const T& t3, std::index_sequence<Is...>) {
    return {std::forward<F>(f)(s1, t2[Is], t3[Is])...};
}

// tuple, scalar, scalar
template <typename F, typename T, typename S, std::size_t... Is>
constexpr eif<is_tuple<T> && is_num<S>, T> cwise(F&& f, const T& t1, S s2, S s3, std::index_sequence<Is...>) {
    return {std::forward<F>(f)(t1[Is], s2, s3)...};
}

// scalar, tuple, scalar
template <typename F, typename T, typename S, std::size_t... Is>
constexpr eif<is_tuple<T> && is_num<S>, T> cwise(F&& f, S s1, const T& t2, S s3, std::index_sequence<Is...>) {
    return {std::forward<F>(f)(s1, t2[Is], s3)...};
}

// scalar, scalar, tuple
template <typename F, typename T, typename S, std::size_t... Is>
constexpr eif<is_tuple<T> && is_num<S>, T> cwise(F&& f, S s1, S s2, const T& t3, std::index_sequence<Is...>) {
    return {std::forward<F>(f)(s1, s2, t3[Is])...};
}

} // namespace detail

/* 1 parameter */
template <typename F, typename T>
constexpr eif<is_tuple<T>, T> cwise(F&& f, const T& t) {
    using tuple = T;
    return detail::cwise(std::forward<F>(f), t, std::make_index_sequence<tuple::size>());
}

/* 2 parameters */
template <typename F, typename T1, typename T2>
constexpr eif<is_tuple<T1> || is_tuple<T2>, std::conditional_t<is_tuple<T1>, T1, T2>> cwise(F&& f, const T1& t1, const T2& t2) {
    using tuple = std::conditional_t<is_tuple<T1>, T1, T2>;
    return detail::cwise(std::forward<F>(f), t1, t2, std::make_index_sequence<tuple::size>());
}

/* 3 parameters */
template <typename F, typename T1, typename T2, typename T3>
constexpr eif<is_tuple<T1> || is_tuple<T2> || is_tuple<T3>, std::conditional_t<is_tuple<T1>, T1, std::conditional_t<is_tuple<T2>, T2, T3>>> cwise(F&& f, const T1& t1, const T2& t2, const T3& t3) {
    using tuple = std::conditional_t<is_tuple<T1>, T1, std::conditional_t<is_tuple<T2>, T2, T3>>;
    return detail::cwise(std::forward<F>(f), t1, t2, t3, std::make_index_sequence<tuple::size>());
}

/**
 * Use tuple elements as distinct parameters with any callable.
 */
namespace detail {

template <typename F, typename T, std::size_t... Is>
constexpr decltype(auto) split(F&& f, const T& t, std::index_sequence<Is...>) {
    return std::forward<F>(f)(t[Is]...);
}

} // namespace detail

template <typename F, typename T, typename = eif<is_tuple<T>>>
constexpr decltype(auto) split(F&& f, const T& t) {
    return detail::split(std::forward<F>(f), t,
                         std::make_index_sequence<T::size>());
}

} // namespace math
} // namespace ee
