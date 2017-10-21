/**
 * Copyright (c) 2017 Gauthier ARNOULD
 * This file is released under the zlib License (Zlib).
 * See file LICENSE or go to https://opensource.org/licenses/Zlib
 * for full license details.
 */

#pragma once

#include <type_traits>

namespace ee {
namespace math {

/**
 * A way to identify scalar numbers.
 */
template <typename T>
constexpr bool is_num = std::is_arithmetic<std::decay_t<T>>::value;

} // namespace math
} // namespace ee