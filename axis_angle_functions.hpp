/**
 * Copyright (c) 2017 Gauthier ARNOULD
 * This file is released under the zlib License (Zlib).
 * See file LICENSE or go to https://opensource.org/licenses/Zlib
 * for full license details.
 */

#pragma once

#include <cmath>

#include "axis_angle.hpp"

#include "mat.hpp"
#include "quat.hpp"
#include "generators.hpp"

namespace ee {
namespace math {

/**
 * Return the matrix describing axis_angle rotation.
 * Based on Rodrigues' rotation formula.
 */
template <typename T>
mat<T, 4 ,4> mat_from(const axis_angle<T>& aa) {
    const T cos_theta = std::cos(aa.angle);
    const T sin_theta = std::sin(aa.angle);

    const T one_minus_cos_theta = T{1L} - cos_theta;

    const T xx_1_minus_cos_t = aa.axis(0) * aa.axis(0) * one_minus_cos_theta;
    const T xy_1_minus_cos_t = aa.axis(0) * aa.axis(1) * one_minus_cos_theta;
    const T yy_1_minus_cos_t = aa.axis(1) * aa.axis(1) * one_minus_cos_theta;
    const T xz_1_minus_cos_t = aa.axis(0) * aa.axis(2) * one_minus_cos_theta;
    const T yz_1_minus_cos_t = aa.axis(1) * aa.axis(2) * one_minus_cos_theta;
    const T zz_1_minus_cos_t = aa.axis(2) * aa.axis(2) * one_minus_cos_theta;

    const T x_sin_t = aa.axis(0) * sin_theta;
    const T y_sin_t = aa.axis(1) * sin_theta;
    const T z_sin_t = aa.axis(2) * sin_theta;

    return {
        // X axis
        xx_1_minus_cos_t + cos_theta,
        xy_1_minus_cos_t + z_sin_t,
        xz_1_minus_cos_t - y_sin_t,
        T{0L},

        // Y axis
        xy_1_minus_cos_t - z_sin_t,
        yy_1_minus_cos_t + cos_theta,
        yz_1_minus_cos_t + x_sin_t,
        T{0L},

        // Z axis
        xz_1_minus_cos_t + y_sin_t,
        yz_1_minus_cos_t - x_sin_t,
        zz_1_minus_cos_t + cos_theta,
        T{0L},

        T{0L}, T{0L}, T{0L}, T{1L}};
}

/**
 * Return the quaternion describing axis_angle rotation.
 */
template <typename T>
quat<T> quat_from(const axis_angle<T>& aa) {
    const T ha = aa.angle * T{0.5L};

    return as<quat<T>>(aa.axis * std::sin(ha), std::cos(ha));
}

/**
 * Rotate v based on axis_angle input using Rodrigues' rotation formula.
 * To rotate several points, better creating a rotation matrix or a quaternion.
 * For right handed basis.
 */
template <typename T>
vec<T, 3> rotate(const axis_angle<T>& aa, const vec<T, 3>& v) {
    const T cos_a = std::cos(aa.angle);
    const T sin_a = std::sin(aa.angle);

    return
        v * cos_a +
        cross(aa.axis, v) * sin_a +
        aa.axis * (dot(aa.axis, v) * (T{1L} - cos_a));
}

} // namespace math
} // namespace ee
