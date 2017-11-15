/**
 * Copyright (c) 2017 Gauthier ARNOULD
 * This file is released under the zlib License (Zlib).
 * See file LICENSE or go to https://opensource.org/licenses/Zlib
 * for full license details.
 */

#pragma once

#include "vec.hpp"

namespace ee {
namespace math {

/**
 * Represent a rotation by an ordered pair (axis, angle).
 * axis should be a unit vector.
 */
template <typename T>
struct axis_angle {
    vec<T, 3> axis;
    T angle;
};

} // namespace math
} // namespace ee
