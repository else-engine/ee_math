/**
 * Copyright (c) 2017 Gauthier ARNOULD
 * This file is released under the zlib License (Zlib).
 * See file LICENSE or go to https://opensource.org/licenses/Zlib
 * for full license details.
 */

#pragma once

namespace ee {
namespace math {

/**
 * Set of begin/end, cbegin/cend functions allowing manipulations through
 * iterators and for range-based loop.
 * Works with mat, vec and quat.
 */
template <typename T>
constexpr auto begin(T& d) noexcept {
    return &d.data[0];
}

template <typename T>
constexpr const auto cbegin(const T& d) noexcept {
    return &d.data[0];
}

template <typename T>
constexpr auto end(T& d) noexcept {
    return &d.data[T::size];
}

template <typename T>
constexpr const auto cend(const T& d) noexcept {
    return &d.data[T::size];
}

} // namespace math
} // namespace ee