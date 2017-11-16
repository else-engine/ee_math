/**
 * Copyright (c) 2017 Gauthier ARNOULD
 * This file is released under the zlib License (Zlib).
 * See file LICENSE or go to https://opensource.org/licenses/Zlib
 * for full license details.
 */

#pragma once

#include "basis.hpp"

namespace ee {
namespace math {

/**
 * Express 3D cartesian coordinates in B.
 * Input cartesian coordinates must come from basis<xpos, ypos, zpos>.
 * Presented with a transformation matrix, it is same as returning {x', y', z'}
 * from input {x, y, z} with :
 *
 *  │x'│   │Ix Iy Iz│   │x│
 *  │y'│ = │Jx Jy Jz│ ∙ │y│
 *  │z'│   │Kx Ky Kz│   │z│
 *
 * But (relying on I, J, K constraints) coded in a more constant expression way.
 */
template <typename B, typename T>
constexpr vec<T, 3> to_basis(const vec<T, 3>& v) {
    constexpr auto i = B::i::template v<int>;
    constexpr auto j = B::j::template v<int>;
    constexpr auto k = B::k::template v<int>;

    constexpr int i_index  = (i(0) ? 0 : 0) + (i(1) ? 1 : 0) + (i(2) ? 2 : 0);
    constexpr int i_switch =  i(0)      +      i(1)      +      i(2);

    constexpr int j_index  = (j(0) ? 0 : 0) + (j(1) ? 1 : 0) + (j(2) ? 2 : 0);
    constexpr int j_switch =  j(0)      +      j(1)      +      j(2);

    constexpr int k_index  = (k(0) ? 0 : 0) + (k(1) ? 1 : 0) + (k(2) ? 2 : 0);
    constexpr int k_switch =  k(0)      +      k(1)      +      k(2);

    return {
        i_switch * v(i_index),
        j_switch * v(j_index),
        k_switch * v(k_index)};
}

/**
 * Express transformation matrix in B.
 * Input matrix must come from basis<xpos, ypos, zpos>.
 * Same as returning M' from input M with :
 *
 *       │Ix Iy Iz 0│       │Ix Jx Kx 0│
 *  M' = │Jx Jy Jz 0│ ∙ M ∙ │Iy Jy Ky 0│
 *       │Kx Ky Kz 0│       │Iz Jz Kz 0│
 *       │ 0  0  0 1│       │ 0  0  0 1│
 *
 * But (relying on I, J, K constraints) coded in a more constant expression way.
 *
 * So, when we do :
 *
 * p' = M' * p
 *
 * it is equivalent to :
 *
 * p' = to_basis< B >( M * from_basis< B >( p ) )
 */
template <typename B, typename T>
constexpr mat<T, 4, 4> to_basis(const mat<T, 4, 4>& m) {
    constexpr auto i = B::i::template v<int>;
    constexpr auto j = B::j::template v<int>;
    constexpr auto k = B::k::template v<int>;

    constexpr int i_index  = (i(0) ? 0 : 0) + (i(1) ? 1 : 0) + (i(2) ? 2 : 0);
    constexpr int i_switch =  i(0)      +      i(1)      +      i(2);

    constexpr int j_index  = (j(0) ? 0 : 0) + (j(1) ? 1 : 0) + (j(2) ? 2 : 0);
    constexpr int j_switch =  j(0)      +      j(1)      +      j(2);

    constexpr int k_index  = (k(0) ? 0 : 0) + (k(1) ? 1 : 0) + (k(2) ? 2 : 0);
    constexpr int k_switch =  k(0)      +      k(1)      +      k(2);

    return {
        i_switch * i_switch * m(i_index, i_index),
        i_switch * j_switch * m(j_index, i_index),
        i_switch * k_switch * m(k_index, i_index),
        i_switch            * m(   3   , i_index),

        j_switch * i_switch * m(i_index, j_index),
        j_switch * j_switch * m(j_index, j_index),
        j_switch * k_switch * m(k_index, j_index),
        j_switch            * m(   3   , j_index),

        k_switch * i_switch * m(i_index, k_index),
        k_switch * j_switch * m(j_index, k_index),
        k_switch * k_switch * m(k_index, k_index),
        k_switch            * m(   3   , k_index),

                   i_switch * m(i_index,    3   ),
                   j_switch * m(j_index,    3   ),
                   k_switch * m(k_index,    3   ),
                              m(   3   ,    3   )};
}

/**
 * Express quaternion in B.
 * Input quaternion's components must come from basis<xpos, ypos, zpos>.
 * Basically, it treats the vector part as the vector overload do and switch
 * angle sign when handedness are different between B and basis<xpos, ypos,
 * zpos> to make sure we describe the same resulting rotation.
 */
template <typename B, typename T>
constexpr quat<T> to_basis(const quat<T>& q) {
    constexpr auto i = B::i::template v<int>;
    constexpr auto j = B::j::template v<int>;
    constexpr auto k = B::k::template v<int>;

    constexpr int angle_switch =
        initial_basis::is_right_handed == B::is_right_handed ? 1 : -1;

    constexpr int i_index  = (i(0) ? 0 : 0) + (i(1) ? 1 : 0) + (i(2) ? 2 : 0);
    constexpr int i_switch = (i(0)      +      i(1)      +      i(2)) * angle_switch;

    constexpr int j_index  = (j(0) ? 0 : 0) + (j(1) ? 1 : 0) + (j(2) ? 2 : 0);
    constexpr int j_switch = (j(0)      +      j(1)      +      j(2)) * angle_switch;

    constexpr int k_index  = (k(0) ? 0 : 0) + (k(1) ? 1 : 0) + (k(2) ? 2 : 0);
    constexpr int k_switch = (k(0)      +      k(1)      +      k(2)) * angle_switch;

    return {
        i_switch * q(i_index),
        j_switch * q(j_index),
        k_switch * q(k_index),
        q(3)};
}

/**
 * Express 3D cartesian coordinates in basis<xpos, ypos, zpos>.
 * Input coordinates must come from B.
 * Presented with a transformation matrix, it is same as returning {x', y', z'}
 * from input {x, y, z} with :
 *
 *  │x'│   │Ix Jx Kx│   │x│
 *  │y'│ = │Iy Jy Ky│ ∙ │y│
 *  │z'│   │Iz Jz Kz│   │z│
 *
 * But (relying on I, J, K constraints) coded in a more constant expression way.
 */
template <typename B, typename T>
constexpr vec<T, 3> from_basis(const vec<T, 3>& v) {
    constexpr auto i = B::i::template v<int>;
    constexpr auto j = B::j::template v<int>;
    constexpr auto k = B::k::template v<int>;

    constexpr int x_index  = (i(0) ? 0 : 0) + (j(0) ? 1 : 0) + (k(0) ? 2 : 0);
    constexpr int x_switch =  i(0)      +      j(0)      +      k(0);

    constexpr int y_index  = (i(1) ? 0 : 0) + (j(1) ? 1 : 0) + (k(1) ? 2 : 0);
    constexpr int y_switch =  i(1)      +      j(1)      +      k(1);

    constexpr int z_index  = (i(2) ? 0 : 0) + (j(2) ? 1 : 0) + (k(2) ? 2 : 0);
    constexpr int z_switch =  i(2)      +      j(2)      +      k(2);

    return {
        x_switch * v(x_index),
        y_switch * v(y_index),
        z_switch * v(z_index)};
}

/**
 * Express transformation matrix in basis<xpos, ypos, zpos>.
 * Input matrix must come from B.
 * Same as returning M' from input M with :
 *
 *       │Ix Jx Kx 0│       │Ix Iy Iz 0│
 *  M' = │Iy Jy Ky 0│ ∙ M ∙ │Jx Jy Jz 0│
 *       │Iz Jz Kz 0│       │Kx Ky Kz 0│
 *       │ 0  0  0 1│       │ 0  0  0 1│
 *
 * But (relying on I, J, K constraints) coded in a more constant expression way.
 *
 * So, when we do :
 *
 * p' = M' * p
 *
 * it is equivalent to :
 *
 * p' = from_basis< B >( M * to_basis< B >( p ) )
 */
template <typename B, typename T>
constexpr mat<T, 4, 4> from_basis(const mat<T, 4, 4>& m) {
    constexpr auto i = B::i::template v<int>;
    constexpr auto j = B::j::template v<int>;
    constexpr auto k = B::k::template v<int>;

    constexpr int x_index  = (i(0) ? 0 : 0) + (j(0) ? 1 : 0) + (k(0) ? 2 : 0);
    constexpr int x_switch =  i(0)      +      j(0)      +      k(0);

    constexpr int y_index  = (i(1) ? 0 : 0) + (j(1) ? 1 : 0) + (k(1) ? 2 : 0);
    constexpr int y_switch =  i(1)      +      j(1)      +      k(1);

    constexpr int z_index  = (i(2) ? 0 : 0) + (j(2) ? 1 : 0) + (k(2) ? 2 : 0);
    constexpr int z_switch =  i(2)      +      j(2)      +      k(2);

    return {
        x_switch * x_switch * m(x_index, x_index),
        x_switch * y_switch * m(y_index, x_index),
        x_switch * z_switch * m(z_index, x_index),
        x_switch            * m(   3   , x_index),

        y_switch * x_switch * m(x_index, y_index),
        y_switch * y_switch * m(y_index, y_index),
        y_switch * z_switch * m(z_index, y_index),
        y_switch            * m(   3   , y_index),

        z_switch * x_switch * m(x_index, z_index),
        z_switch * y_switch * m(y_index, z_index),
        z_switch * z_switch * m(z_index, z_index),
        z_switch            * m(   3   , z_index),

                   x_switch * m(x_index,    3   ),
                   y_switch * m(y_index,    3   ),
                   z_switch * m(z_index,    3   ),
                              m(   3   ,    3   )};
}

/**
 * Express quaternion in basis<xpos, ypos, zpos>.
 * Input quaternion's components must come from B.
 * Basically, it treats the vector part as the vector overload do and switch
 * angle sign when handedness are different between B and basis<xpos, ypos,
 * zpos> to make sure we describe the same resulting rotation.
 */
template <typename B, typename T>
constexpr quat<T> from_basis(const quat<T>& q) {
    constexpr auto i = B::i::template v<int>;
    constexpr auto j = B::j::template v<int>;
    constexpr auto k = B::k::template v<int>;

    constexpr int angle_switch =
        initial_basis::is_right_handed == B::is_right_handed ? 1 : -1;

    constexpr int x_index  = (i(0) ? 0 : 0) + (j(0) ? 1 : 0) + (k(0) ? 2 : 0);
    constexpr int x_switch = (i(0)      +      j(0)      +      k(0)) * angle_switch;

    constexpr int y_index  = (i(1) ? 0 : 0) + (j(1) ? 1 : 0) + (k(1) ? 2 : 0);
    constexpr int y_switch = (i(1)      +      j(1)      +      k(1)) * angle_switch;

    constexpr int z_index  = (i(2) ? 0 : 0) + (j(2) ? 1 : 0) + (k(2) ? 2 : 0);
    constexpr int z_switch = (i(2)      +      j(2)      +      k(2)) * angle_switch;

    return {
        x_switch * q(x_index),
        y_switch * q(y_index),
        z_switch * q(z_index),
        q(3)};
}

/**
 * Return the matrix describing rotation of angle a around xpos axis.
 */
template <typename T>
mat<T, 4, 4> mat_from(T a, xpos) {
    const T cos_a = std::cos(a);
    const T sin_a = std::sin(a);

    return {
        T{1L},   T{0L}, T{0L}, T{0L},
        T{0L},   cos_a, sin_a, T{0L},
        T{0L}, - sin_a, cos_a, T{0L},
        T{0L},   T{0L}, T{0L}, T{1L}};
}

/**
 * Return the matrix describing rotation of angle a around xneg axis.
 */
template <typename T>
mat<T, 4, 4> mat_from(T a, xneg) {
    const T cos_a = std::cos(a);
    const T sin_a = std::sin(a);

    return {
        T{1L}, T{0L},   T{0L}, T{0L},
        T{0L}, cos_a, - sin_a, T{0L},
        T{0L}, sin_a,   cos_a, T{0L},
        T{0L}, T{0L},   T{0L}, T{1L}};
}

/**
 * Return the matrix describing rotation of angle a around ypos axis.
 */
template <typename T>
mat<T, 4, 4> mat_from(T a, ypos) {
    const T cos_a = std::cos(a);
    const T sin_a = std::sin(a);

    return {
        cos_a, T{0L}, - sin_a, T{0L},
        T{0L}, T{1L},   T{0L}, T{0L},
        sin_a, T{0L},   cos_a, T{0L},
        T{0L}, T{0L},   T{0L}, T{1L}};
}

/**
 * Return the matrix describing rotation of angle a around yneg axis.
 */
template <typename T>
mat<T, 4, 4> mat_from(T a, yneg) {
    const T cos_a = std::cos(a);
    const T sin_a = std::sin(a);

    return {
          cos_a, T{0L}, sin_a, T{0L},
          T{0L}, T{1L}, T{0L}, T{0L},
        - sin_a, T{0L}, cos_a, T{0L},
          T{0L}, T{0L}, T{0L}, T{1L}};
}

/**
 * Return the matrix describing rotation of angle a around zpos axis.
 */
template <typename T>
mat<T, 4, 4> mat_from(T a, zpos) {
    const T cos_a = std::cos(a);
    const T sin_a = std::sin(a);

    return {
          cos_a, sin_a, T{0L}, T{0L},
        - sin_a, cos_a, T{0L}, T{0L},
          T{0L}, T{0L}, T{1L}, T{0L},
          T{0L}, T{0L}, T{0L}, T{1L}};
}

/**
 * Return the matrix describing rotation of angle a around zneg axis.
 */
template <typename T>
mat<T, 4, 4> mat_from(T a, zneg) {
    const T cos_a = std::cos(a);
    const T sin_a = std::sin(a);

    return {
        cos_a, - sin_a, T{0L}, T{0L},
        sin_a,   cos_a, T{0L}, T{0L},
        T{0L},   T{0L}, T{1L}, T{0L},
        T{0L},   T{0L}, T{0L}, T{1L}};
}

/**
 * Return the quaternion describing rotation of angle a around xpos axis.
 */
template <typename T>
quat<T> quat_from(T a, xpos) {
    a *= T{0.5L};

    return {std::sin(a), T{0L}, T{0L}, std::cos(a)};
}

/**
 * Return the quaternion describing rotation of angle a around xneg axis.
 */
template <typename T>
quat<T> quat_from(T a, xneg) {
    a *= T{0.5L};

    return { - std::sin(a), T{0L}, T{0L}, std::cos(a)};
}

/**
 * Return the quaternion describing rotation of angle a around ypos axis.
 */
template <typename T>
quat<T> quat_from(T a, ypos) {
    a *= T{0.5L};

    return {T{0L}, std::sin(a), T{0L}, std::cos(a)};
}

/**
 * Return the quaternion describing rotation of angle a around yneg axis.
 */
template <typename T>
quat<T> quat_from(T a, yneg) {
    a *= T{0.5L};

    return {T{0L}, - std::sin(a), T{0L}, std::cos(a)};
}

/**
 * Return the quaternion describing rotation of angle a around zpos axis.
 */
template <typename T>
quat<T> quat_from(T a, zpos) {
    a *= T{0.5L};

    return quat<T>{T{0L}, T{0L}, std::sin(a), std::cos(a)};
}

/**
 * Return the quaternion describing rotation of angle a around zneg axis.
 */
template <typename T>
quat<T> quat_from(T a, zneg) {
    a *= T{0.5L};

    return quat<T>{T{0L}, T{0L}, - std::sin(a), std::cos(a)};
}

/**
 * Return the 1st vector of the basis described by a given quaternion.
 * Input quaternion must be normalized.
 */
template <typename T>
constexpr vec<T, 3> basis_vector(const quat<T>& q, xpos) {
    return {
        T{1L} - T{2L} * (q.y * q.y + q.z * q.z),
                T{2L} * (q.x * q.y + q.w * q.z),
                T{2L} * (q.x * q.z - q.w * q.y)};
}

/**
 * Return the opposite of the 1st vector of the basis described by a given
 * quaternion.
 * Input quaternion must be normalized.
 */
template <typename T>
constexpr vec<T, 3> basis_vector(const quat<T>& q, xneg) {
    return {
          T{2L} * (q.y * q.y + q.z * q.z) - T{1L},
        - T{2L} * (q.x * q.y + q.w * q.z),
        - T{2L} * (q.x * q.z - q.w * q.y)};
}

/**
 * Return the 2nd vector of the basis described by a given quaternion.
 * Input quaternion must be normalized.
 */
template <typename T>
constexpr vec<T, 3> basis_vector(const quat<T>& q, ypos) {
    return {
                T{2L} * (q.x * q.y - q.w * q.z),
        T{1L} - T{2L} * (q.x * q.x + q.z * q.z),
                T{2L} * (q.y * q.z + q.w * q.x)};
}

/**
 * Return the opposite of the 2nd vector of the basis described by a given
 * quaternion.
 * Input quaternion must be normalized.
 */
template <typename T>
constexpr vec<T, 3> basis_vector(const quat<T>& q, yneg) {
    return {
        - T{2L} * (q.x * q.y - q.w * q.z),
          T{2L} * (q.x * q.x + q.z * q.z) - T{1L},
        - T{2L} * (q.y * q.z + q.w * q.x)};
}

/**
 * Return the 3rd vector of the basis described by a given quaternion.
 * Input quaternion must be normalized.
 */
template <typename T>
constexpr vec<T, 3> basis_vector(const quat<T>& q, zpos) {
    return {
                T{2L} * (q.x * q.z + q.w * q.y),
                T{2L} * (q.y * q.z - q.w * q.x),
        T{1L} - T{2L} * (q.x * q.x + q.y * q.y)};
}

/**
 * Return the opposite of the 3rd vector of the basis described by a given
 * quaternion.
 * Input quaternion must be normalized.
 */
template <typename T>
constexpr vec<T, 3> basis_vector(const quat<T>& q, zneg) {
    return {
        - T{2L} * (q.x * q.z + q.w * q.y),
        - T{2L} * (q.y * q.z - q.w * q.x),
          T{2L} * (q.x * q.x + q.y * q.y) - T{1L}};
}

} // namespace math
} // namespace ee
