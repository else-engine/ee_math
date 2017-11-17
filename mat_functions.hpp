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

namespace detail {

template <typename T, std::size_t R, std::size_t C, std::size_t... Is>
constexpr auto transpose(const mat<T, R, C>& M, std::index_sequence<Is...>) {
    return mat<T, C, R>{
        M.data[Is / C + R * (Is % C)]...
    };
}

} // namespace detail

/**
 * Return the transpose of a matrix.
 */
template <typename T, std::size_t R, std::size_t C>
constexpr auto transpose(const mat<T, R, C>& M) {
    return detail::transpose(M, std::make_index_sequence<R * C>());
}

/**
 * The trace of an n-by-n square matrix A is defined to be the sum of the
 * elements on the main diagonal.
 * https://en.wikipedia.org/wiki/Trace_%28linear_algebra%29
 */
template <typename T, std::size_t D>
constexpr auto trace(const mat<T, D, D>& m) {
    T sum{};

    for (std::size_t d = 0ul; d < D; ++ d) {
        sum += m(d, d);
    }

    return sum;
}

/**
 * Return determinant of a given matrix.
 */
template <typename T, std::size_t D>
constexpr auto det(const mat<T, D, D>& M) {
    T result{T{0L}};

    for (std::size_t d = 0ul; d < D; ++ d) {
        result +=
            ((d & 1) ? - T{1L} : T{1L})
            * M(0, d)
            * det(cut(M, {0, d}));
    }

    return result;
}

/**
 * Return determinant of a square matrix.
 * This overload is mandatory to avoid generic det() to instantiate a mat0x0.
 */
template <typename T>
constexpr auto det(const mat<T, 1, 1>& M) {
    return M(0, 0);
}

/**
 * Return determinant of a square matrix.
 * Adding mat2x2 overload use near half less arithmetic operations.
 */
template <typename T>
constexpr auto det(const mat<T, 2, 2>& M) {
    return M(0, 0) * M(1, 1) - M(0, 1) * M(1, 0);
}

#if 0
/**
 * Return determinant of a square matrix.
 * Adding mat3x3 overlad still decrease addition count and let multiplication
 * count unchanged.
 * Now by checking binary result it seems that up to mat3x3 overload is better
 * (less instructions) than mat2x2 with debug build but becomes worse with a
 * release optimized build.
 */
template <typename T>
constexpr auto det(const mat<T, 3, 3>& M) {
    return
        M(0, 0) * M(1, 1) * M(2, 2) +
        M(0, 1) * M(1, 2) * M(2, 0) +
        M(0, 2) * M(1, 0) * M(2, 1) -
        M(0, 2) * M(1, 1) * M(2, 0) -
        M(0, 1) * M(1, 0) * M(2, 2) -
        M(0, 0) * M(1, 2) * M(2, 1);
}
#endif

#if 0
/**
 * Return determinant of a square matrix.
 * Adding mat4x4 overload deacrease addition just a little more but increase
 * multiplication near what was obtained at mat1x1 only overload.
 */
template <typename T>
constexpr auto det(const mat<T, 4, 4>& M) {
    return
        M(0, 0) * M(1, 1) * M(2, 2) * M(3, 3) -
        M(0, 0) * M(1, 1) * M(2, 3) * M(3, 2) +
        M(0, 0) * M(1, 2) * M(2, 3) * M(3, 1) -
        M(0, 0) * M(1, 2) * M(2, 1) * M(3, 3) +
        M(0, 0) * M(1, 3) * M(2, 1) * M(3, 2) -
        M(0, 0) * M(1, 3) * M(2, 2) * M(3, 1) -
        M(0, 1) * M(1, 2) * M(2, 3) * M(3, 0) +
        M(0, 1) * M(1, 2) * M(2, 0) * M(3, 3) -
        M(0, 1) * M(1, 3) * M(2, 0) * M(3, 2) +
        M(0, 1) * M(1, 3) * M(2, 2) * M(3, 0) -
        M(0, 1) * M(1, 0) * M(2, 2) * M(3, 3) +
        M(0, 1) * M(1, 0) * M(2, 3) * M(3, 2) +
        M(0, 2) * M(1, 3) * M(2, 0) * M(3, 1) -
        M(0, 2) * M(1, 3) * M(2, 1) * M(3, 0) +
        M(0, 2) * M(1, 0) * M(2, 1) * M(3, 3) -
        M(0, 2) * M(1, 0) * M(2, 3) * M(3, 1) +
        M(0, 2) * M(1, 1) * M(2, 3) * M(3, 0) -
        M(0, 2) * M(1, 1) * M(2, 0) * M(3, 3) -
        M(0, 3) * M(1, 0) * M(2, 1) * M(3, 2) +
        M(0, 3) * M(1, 0) * M(2, 2) * M(3, 1) -
        M(0, 3) * M(1, 1) * M(2, 2) * M(3, 0) +
        M(0, 3) * M(1, 1) * M(2, 0) * M(3, 2) -
        M(0, 3) * M(1, 2) * M(2, 0) * M(3, 1) +
        M(0, 3) * M(1, 2) * M(2, 1) * M(3, 0);
}
#endif

namespace detail {

template <typename T, std::size_t D, std::size_t... Is>
constexpr auto inv(const mat<T, D, D>& M, T rcp_d, std::index_sequence<Is...>) {
    return mat<T, D, D>{
        ((Is % D + Is / D) & std::size_t{1L} ? - T{1L} : T{1L})
            * det(cut(M, {Is / D, Is % D}))...} * rcp_d;
}

} // namespace detail

/**
 * Return inverse of an invertible square matrix.
 * No test is done for determinant equals zero, input matrix must be invertible.
 */
template <typename T, std::size_t D>
constexpr auto inv(const mat<T, D, D>& M) {
    const T d = det(M);

    const T rcp_d = T{1L} / d;

    return detail::inv(M, rcp_d, std::make_index_sequence<D * D>());
}

/**
 * Return inverse of an invertible square matrix.
 * Overload for mat3x3.
 */
template <typename T>
constexpr auto inv(const mat<T, 3, 3>& M) {
    T d = det(M);

    T rcp_d = T{1L} / d;

    return mat<T, 3, 3>{
        M(1, 1) * M(2, 2) - M(2, 1) * M(1, 2),
        M(1, 2) * M(2, 0) - M(1, 0) * M(2, 2),
        M(1, 0) * M(2, 1) - M(2, 0) * M(1, 1),

        M(0, 2) * M(2, 1) - M(0, 1) * M(2, 2),
        M(0, 0) * M(2, 2) - M(0, 2) * M(2, 0),
        M(2, 0) * M(0, 1) - M(0, 0) * M(2, 1),

        M(0, 1) * M(1, 2) - M(0, 2) * M(1, 1),
        M(1, 0) * M(0, 2) - M(0, 0) * M(1, 2),
        M(0, 0) * M(1, 1) - M(1, 0) * M(0, 1)} * rcp_d;
}

/**
 * Return inverse of an invertible square matrix.
 * Overload for mat4x4.
 */
template <typename T>
constexpr auto inv(const mat<T, 4, 4>& M) {
    T d = det(M);

    T rcp_d = T{1L} / d;

    return mat<T, 4, 4>{
        (M(1, 1) * (M(2, 2) * M(3, 3) - M(2, 3) * M(3, 2)) +
         M(1, 2) * (M(2, 3) * M(3, 1) - M(2, 1) * M(3, 3)) +
         M(1, 3) * (M(2, 1) * M(3, 2) - M(2, 2) * M(3, 1))),
        ( - (M(1, 0) * (M(2, 2) * M(3, 3) - M(2, 3) * M(3, 2)) +
             M(1, 2) * (M(2, 3) * M(3, 0) - M(2, 0) * M(3, 3)) +
             M(1, 3) * (M(2, 0) * M(3, 2) - M(2, 2) * M(3, 0)))),
        (M(1, 0) * (M(2, 1) * M(3, 3) - M(2, 3) * M(3, 1)) +
         M(1, 1) * (M(2, 3) * M(3, 0) - M(2, 0) * M(3, 3)) +
         M(1, 3) * (M(2, 0) * M(3, 1) - M(2, 1) * M(3, 0))),
        ( - (M(1, 0) * (M(2, 1) * M(3, 2) - M(2, 2) * M(3, 1)) +
             M(1, 1) * (M(2, 2) * M(3, 0) - M(2, 0) * M(3, 2)) +
             M(1, 2) * (M(2, 0) * M(3, 1) - M(2, 1) * M(3, 0)))),

        ( - (M(0, 1) * (M(2, 2) * M(3, 3) - M(2, 3) * M(3, 2)) +
             M(0, 2) * (M(2, 3) * M(3, 1) - M(2, 1) * M(3, 3)) +
             M(0, 3) * (M(2, 1) * M(3, 2) - M(2, 2) * M(3, 1)))),
        (M(0, 0) * (M(2, 2) * M(3, 3) - M(2, 3) * M(3, 2)) +
         M(0, 2) * (M(2, 3) * M(3, 0) - M(2, 0) * M(3, 3)) +
         M(0, 3) * (M(2, 0) * M(3, 2) - M(2, 2) * M(3, 0))),
        ( - (M(0, 0) * (M(2, 1) * M(3, 3) - M(2, 3) * M(3, 1)) +
             M(0, 1) * (M(2, 3) * M(3, 0) - M(2, 0) * M(3, 3)) +
             M(0, 3) * (M(2, 0) * M(3, 1) - M(2, 1) * M(3, 0)))),
        (M(0, 0) * (M(2, 1) * M(3, 2) - M(2, 2) * M(3, 1)) +
         M(0, 1) * (M(2, 2) * M(3, 0) - M(2, 0) * M(3, 2)) +
         M(0, 2) * (M(2, 0) * M(3, 1) - M(2, 1) * M(3, 0))),

        (M(0, 1) * (M(1, 2) * M(3, 3) - M(1, 3) * M(3, 2)) +
         M(0, 2) * (M(1, 3) * M(3, 1) - M(1, 1) * M(3, 3)) +
         M(0, 3) * (M(1, 1) * M(3, 2) - M(1, 2) * M(3, 1))),
        ( - (M(0, 0) * (M(1, 2) * M(3, 3) - M(1, 3) * M(3, 2)) +
             M(0, 2) * (M(1, 3) * M(3, 0) - M(1, 0) * M(3, 3)) +
             M(0, 3) * (M(1, 0) * M(3, 2) - M(1, 2) * M(3, 0)))),
        (M(0, 0) * (M(1, 1) * M(3, 3) - M(1, 3) * M(3, 1)) +
         M(0, 1) * (M(1, 3) * M(3, 0) - M(1, 0) * M(3, 3)) +
         M(0, 3) * (M(1, 0) * M(3, 1) - M(1, 1) * M(3, 0))),
        ( - (M(0, 0) * (M(1, 1) * M(3, 2) - M(1, 2) * M(3, 1)) +
             M(0, 1) * (M(1, 2) * M(3, 0) - M(1, 0) * M(3, 2)) +
             M(0, 2) * (M(1, 0) * M(3, 1) - M(1, 1) * M(3, 0)))),

        ( - (M(0, 1) * (M(1, 2) * M(2, 3) - M(1, 3) * M(2, 2)) +
             M(0, 2) * (M(1, 3) * M(2, 1) - M(1, 1) * M(2, 3)) +
             M(0, 3) * (M(1, 1) * M(2, 2) - M(1, 2) * M(2, 1)))),
        (M(0, 0) * (M(1, 2) * M(2, 3) - M(1, 3) * M(2, 2)) +
         M(0, 2) * (M(1, 3) * M(2, 0) - M(1, 0) * M(2, 3)) +
         M(0, 3) * (M(1, 0) * M(2, 2) - M(1, 2) * M(2, 0))),
        ( - (M(0, 0) * (M(1, 1) * M(2, 3) - M(1, 3) * M(2, 1)) +
             M(0, 1) * (M(1, 3) * M(2, 0) - M(1, 0) * M(2, 3)) +
             M(0, 3) * (M(1, 0) * M(2, 1) - M(1, 1) * M(2, 0)))),
        (M(0, 0) * (M(1, 1) * M(2, 2) - M(1, 2) * M(2, 1)) +
         M(0, 1) * (M(1, 2) * M(2, 0) - M(1, 0) * M(2, 2)) +
         M(0, 2) * (M(1, 0) * M(2, 1) - M(1, 1) * M(2, 0)))} * rcp_d;
}

} // namespace math
} // namespace ee
