/**
 * Copyright (c) 2017 Gauthier ARNOULD
 * This file is released under the zlib License (Zlib).
 * See file LICENSE or go to https://opensource.org/licenses/Zlib
 * for full license details.
 */

#pragma once

#include <cmath>

#include "basis.hpp"
#include "mat.hpp"

namespace ee {
namespace math {

/**
 * Standard perspective projection.
 */
template <typename T>
constexpr mat<T, 4, 4> perspective(T fovy, T aspect, T near, T far) {
    const T d = T{1L} / std::tan(fovy * T{0.5L});
    const T near_m_far = near - far;

#if 1
    // OpenGL
    return {
        d / aspect, T{0L},             T{0L}              ,   T{0L},
          T{0L}   ,   d  ,             T{0L}              ,   T{0L},
          T{0L}   , T{0L},    (near + far) / near_m_far   , - T{1L},
          T{0L}   , T{0L}, T{2L} * near * far / near_m_far,   T{0L}};
#else
    // DirectX
    return {
        d / aspect, T{0L},       T{0L}       ,          T{0L}           ,
          T{0L}   ,   d  ,       T{0L}       ,          T{0L}           ,
          T{0L}   , T{0L}, far / (far - near), near * far / (near - far),
          T{0L}   , T{0L},       T{1L}       ,          T{0L}           };
#endif
}

/**
 * Inverse of standard perspective projection.
 * To avoid computing inverse.
 * Found empirically.
 */
template <typename T>
constexpr mat<T, 4, 4> perspective_inverse(T fovy, T aspect, T near, T far) {
    const T rcp_d = std::tan(fovy * T{0.5L});

#if 1
    // OpenGL
    const T _2nf = T{2L} * near * far;

    return {
        aspect * rcp_d, T{0L},   T{0L},        T{0L}       ,
            T{0L}     , rcp_d,   T{0L},        T{0L}       ,
            T{0L}     , T{0L},   T{0L}, (near - far) / _2nf,
            T{0L}     , T{0L}, - T{1L}, (near + far) / _2nf};
#else
    // DirectX
    const T nf    = near * far;

    return {
        aspect * rcp_d, T{0L},        T{0L}     ,   T{0L} ,
            T{0L}     , rcp_d,        T{0L}     ,   T{0L} ,
            T{0L}     , T{0L},        T{0L}     ,   T{1L} ,
            T{0L}     , T{0L}, (near - far) / nf, far / nf};
#endif
}

/**
 * Oblique perspective projection.
 */
template <typename T>
constexpr mat<T, 4, 4> perspective(T right, T left, T top, T bottom, T near, T far) {
    const T _2near       = T{2L} * near;
    const T right_m_left = right - left;
    const T top_m_bottom = top - bottom;
    const T near_m_far   = near - far;

    return {
            _2near / right_m_left    ,              T{0L}           ,            T{0L}         ,   T{0L},
                    T{0L}            ,     _2near / top_m_bottom    ,            T{0L}         ,   T{0L},
        (right + left) / right_m_left, (top + bottom) / top_m_bottom, (near + far) / near_m_far, - T{1L},
                    T{0L}            ,              T{0L}           , _2near * far / near_m_far,   T{0L}};
}

/**
 * Infinite perspective projection.
 * http://chaosinmotion.com/blog/?p=555
 */
template <typename T>
constexpr mat<T, 4, 4> perspective(T fovy, T aspect, T near) {
    const T d = T{1L} / std::tan(fovy * T{0.5L});

#if 1
    return {
        d / aspect, T{0L},      T{0L}    ,   T{0L},
          T{0L}   ,   d  ,      T{0L}    ,   T{0L},
          T{0L}   , T{0L},    - T{1L}    , - T{1L},
          T{0L}   , T{0L}, - T{2L} * near,   T{0L}};
#else
    // TODO test !
    return {
        d / aspect, T{0L},   T{0L},   T{0L},
          T{0L}   ,   d  ,   T{0L},   T{0L},
          T{0L}   , T{0L},   T{0L}, - T{1L},
          T{0L}   , T{0L}, - near ,   T{0L}};
#endif
}

/**
 * Orthographic projection.
 */
template <typename T>
constexpr mat<T, 4, 4> orthographic(T left, T right, T bottom, T top, T near, T far) {
    const T near_m_far = near - far;

    return {
             T{2L} / (right - left)    ,               T{0L}            ,           T{0L}          , T{0L},
                     T{0L}             ,       T{2L} / (top - bottom)   ,           T{0L}          , T{0L},
                     T{0L}             ,               T{0L}            ,    T{2L} / near_m_far    , T{0L},
        (left + right) / (left - right), (bottom + top) / (bottom - top), (near + far) / near_m_far, T{1L}};
}

template <typename T>
constexpr mat<T, 4, 4> orthographic(T width, T height, T near, T far) {
    const T near_m_far = near - far;

    return {
        T{2L} / width,      T{0L}    ,           T{0L}          , T{0L},
             T{0L}   , T{2L} / height,           T{0L}          , T{0L},
             T{0L}   ,      T{0L}    ,    T{2L} / near_m_far    , T{0L},
             T{0L}   ,      T{0L}    , (near + far) / near_m_far, T{1L}};
}

template <typename T>
constexpr mat<T, 4, 4> orthographic(T width, T height, T depth) {
    return {
        T{2L} / width,      T{0L}    ,      T{0L}   , T{0L},
             T{0L}   , T{2L} / height,      T{0L}   , T{0L},
             T{0L}   ,      T{0L}    , T{2L} / depth, T{0L},
             T{0L}   ,      T{0L}    ,      T{0L}   , T{1L}};
}

/**
 * Inverse of orthographic projection.
 * To avoid computing inverse.
 */
template <typename T>
constexpr mat<T, 4, 4> orthographic_inverse(T left, T right, T bottom, T top, T near, T far) {
    return {
        (right - left) * T{0.5L},          T{0L}          ,          T{0L}          , T{0L},
                 T{0L}          , (top - bottom) * T{0.5L},          T{0L}          , T{0L},
                 T{0L}          ,          T{0L}          ,   (near - far) * T{0.5L}, T{0L},
        (left + right) * T{0.5L}, (bottom + top) * T{0.5L}, - (near + far) * T{0.5L}, T{1L}};
}

/**
 * Viewport transformation matrix.
 */
template <typename T>
constexpr mat<T, 4, 4> viewport(const uint2& lower_left, const uint2& size, T near, T far) {
    auto half_size = size * T{0.5L};

    return {
                half_size.w       ,            T{0L}          ,         T{0L}         , T{0L},
                   T{0L}          ,         half_size.h       ,         T{0L}         , T{0L},
                   T{0L}          ,            T{0L}          , (far - near) * T{0.5L}, T{0L},
        lower_left.x + half_size.w, lower_left.y + half_size.h, (far + near) * T{0.5L}, T{1L}};
}

/**
 * Viewport transformation matrix when lower left corner is (0, 0).
 */
template <typename T>
constexpr mat<T, 4, 4> viewport(const uint2& size, T near, T far) {
    auto half_size = size * T{0.5L};

    return {
        half_size.w,    T{0L}   ,         T{0L}         , T{0L},
           T{0L}   , half_size.h,         T{0L}         , T{0L},
           T{0L}   ,    T{0L}   , (far - near) * T{0.5L}, T{0L},
        half_size.w, half_size.h, (far + near) * T{0.5L}, T{1L}};
}

} // namespace math
} // namespace ee
