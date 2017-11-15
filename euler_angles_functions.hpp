/**
 * Copyright (c) 2017 Gauthier ARNOULD
 * This file is released under the zlib License (Zlib).
 * See file LICENSE or go to https://opensource.org/licenses/Zlib
 * for full license details.
 */

#pragma once

#include <cmath>

#include "euler_angles.hpp"

#include "mat.hpp"
#include "quat.hpp"

namespace ee {
namespace math {

/**
 * Return matrix describing rotation from Euler angles.
 */
template <typename T, typename B>
mat<T, 4, 4> mat_from(const euler_angles<T, B>& ea) {
    const T cos_a = std::cos(ea.alpha);
    const T sin_a = std::sin(ea.alpha);

    const T cos_b = std::cos(ea.beta);
    const T sin_b = std::sin(ea.beta);

    const T cos_g = std::cos(ea.gamma);
    const T sin_g = std::sin(ea.gamma);

    const T cos_a_cos_g = cos_a * cos_g;
    const T sin_a_sin_g = sin_a * sin_g;
    const T cos_g_sin_a = cos_g * sin_a;
    const T cos_a_sin_g = cos_a * sin_g;

    return from_basis<B>(mat<T, 4, 4>{
          cos_a_cos_g - cos_b * sin_a_sin_g,
          cos_g_sin_a + cos_a_sin_g * cos_b,
          sin_b * sin_g,
          T{0L},

        - cos_a_sin_g - cos_b * cos_g_sin_a,
          cos_a_cos_g * cos_b - sin_a_sin_g,
          cos_g * sin_b,
          T{0L},

          sin_a * sin_b,
        - cos_a * sin_b,
          cos_b,
          T{0L},

          T{0L}, T{0L}, T{0L}, T{1L}});
}

/**
 * Return matrix describing rotation from Tait-Bryan angles.
 */
template <typename T, typename B>
mat<T, 4, 4> mat_from(const tait_bryan_angles<T, B>& ea) {
    const T cos_a = std::cos(ea.alpha);
    const T sin_a = std::sin(ea.alpha);

    const T cos_b = std::cos(ea.beta);
    const T sin_b = std::sin(ea.beta);

    const T cos_g = std::cos(ea.gamma);
    const T sin_g = std::sin(ea.gamma);

    const T cos_a_cos_g = cos_a * cos_g;
    const T sin_b_sin_g = sin_b * sin_g;
    const T cos_g_sin_a = cos_g * sin_a;

    return from_basis<B>(mat<T, 4, 4>{
          cos_a * cos_b,
          cos_b * sin_a,
        - sin_b,
          T{0L},

          cos_a * sin_b_sin_g - cos_g_sin_a,
          cos_a_cos_g + sin_a * sin_b_sin_g,
          cos_b * sin_g,
          T{0L},

          sin_a * sin_g + cos_a_cos_g * sin_b,
          cos_g_sin_a * sin_b - cos_a * sin_g,
          cos_b * cos_g,
          T{0L},

          T{0L}, T{0L}, T{0L}, T{1L}});
}

/**
 * Return quaternion describing rotation from Euler angles.
 */
template <typename T, typename B>
quat<T> quat_from(const euler_angles<T, B>& ea) {
    const T     a = T{0.5L} * ea.alpha;
    const T cos_a = std::cos(a);
    const T sin_a = std::sin(a);

    const T     b = T{0.5L} * ea.beta;
    const T cos_b = std::cos(b);
    const T sin_b = std::sin(b);

    const T     g = T{0.5L} * ea.gamma;
    const T cos_g = std::cos(g);
    const T sin_g = std::sin(g);

    return from_basis<B>(quat<T>{
        cos_a * sin_b * cos_g + sin_a * sin_b * sin_g,
        sin_a * sin_b * cos_g - cos_a * sin_b * sin_g,
        sin_a * cos_b * cos_g + cos_a * cos_b * sin_g,
        cos_a * cos_b * cos_g - sin_a * cos_b * sin_g});
}

/**
 * Return quaternion describing rotation from Tait-Bryan angles.
 */
template <typename T, typename B>
quat<T> quat_from(const tait_bryan_angles<T, B>& tba) {
    const T     a = T{0.5L} * tba.alpha;
    const T cos_a = std::cos(a);
    const T sin_a = std::sin(a);

    const T     b = T{0.5L} * tba.beta;
    const T cos_b = std::cos(b);
    const T sin_b = std::sin(b);

    const T     g = T{0.5L} * tba.gamma;
    const T cos_g = std::cos(g);
    const T sin_g = std::sin(g);

    return from_basis<B>(quat<T>{
        cos_a * cos_b * sin_g - sin_a * sin_b * cos_g,
        cos_a * sin_b * cos_g + sin_a * cos_b * sin_g,
        sin_a * cos_b * cos_g - cos_a * sin_b * sin_g,
        cos_a * cos_b * cos_g + sin_a * sin_b * sin_g});
}

} // namespace math
} // namespace ee
