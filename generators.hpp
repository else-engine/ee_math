/**
 * Copyright (c) 2017 Gauthier ARNOULD
 * This file is released under the zlib License (Zlib).
 * See file LICENSE or go to https://opensource.org/licenses/Zlib
 * for full license details.
 */

#pragma once

#include <cstddef>
#include <utility>

#include <ee_utils/templates.hpp>

#include "common.hpp"
#include "vec.hpp"
#include "mat.hpp"

namespace ee {
namespace math {

using tutil::eif;
using tutil::all_same;

/**
 * Creates a vec object, deducing type and size from arguments.
 */
template <typename F, typename... Rs,
         typename = eif<all_same<std::decay_t<F>, std::decay_t<Rs>...>>>
constexpr auto make_vec(F&& f, Rs&&... rs) {
    return vec<std::decay_t<F>, sizeof...(rs) + 1>{
        std::forward<F>(f), std::forward<Rs>(rs)...};
}

/**
 * Returns a mat, vec or quat filled with input value.
 */
namespace detail {

template <typename O, std::size_t... Is>
constexpr auto fill(typename O::value_type v,
                    std::index_sequence<Is...>) {
    return O{(static_cast<void>(Is), v)...};
}

} // namespace detail

template <typename O>
constexpr auto fill(typename O::value_type v) {
    return detail::fill<O>(v, std::make_index_sequence<O::size>{});
}

/**
 * Returns a mat, vec or quat, depending of what O is, with as many values as
 * required from a given pointer.
 */
namespace detail {

template <typename O, typename T, std::size_t... Is>
constexpr auto as_ptr(const T* const ptr,
                      std::index_sequence<Is...>) {
    return O{static_cast<typename O::value_type>(ptr[Is])...};
}

} // namespace detail

template <typename O, typename T>
constexpr auto as_ptr(const T* const ptr) {
    return detail::as_ptr<O>(ptr, std::make_index_sequence<O::size>());
}

/**
 * Returns a mat, vec or quat, depending of what O is, based on a list composed
 * of mat, vec and/or arithmetic elements.
 */
namespace detail {

template <typename O, std::size_t I, typename... Ts, typename = eif<I == 0>, typename = eif<sizeof...(Ts) == O::size>>
constexpr auto as(Ts&&... ts) {
    return O{ts...};
}

template <typename, std::size_t I, typename T, typename... Rs, typename = eif<I != 0 && (is_mat<T> || is_vec<T>)>>
constexpr auto as(const T&, Rs&&...);

template <typename O, std::size_t I, typename F, typename... Rs, typename = eif<I != 0>, typename = eif<is_num<F>>>
constexpr auto as(F&& f, Rs&&... rs) {
    return as<O, I - 1>(std::forward<Rs>(rs)..., static_cast<typename O::value_type>(f));
}

template <typename O, std::size_t I, std::size_t... Is, typename F, typename... Rs>
constexpr auto as(std::index_sequence<Is...>, const F* const f, Rs&&... rs) {
    return as<O, I - 1>(std::forward<Rs>(rs)..., static_cast<typename O::value_type>(f[Is])...);
}

template <typename O, std::size_t I, typename T, typename... Rs, typename>
constexpr auto as(const T& f, Rs&&... rs) {
    return as<O, I>(std::make_index_sequence<T::size>{}, f.data, std::forward<Rs>(rs)...);
}

} // namespace detail

template <typename O, typename F, typename... Rs>
constexpr auto as(F&& f, Rs&&... ts) {
    return detail::as<O, sizeof...(Rs) + 1>(std::forward<F>(f), std::forward<Rs>(ts)...);
}

} // namespace math
} // namespace ee