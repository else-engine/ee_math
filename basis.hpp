/**
 * Copyright (c) 2017 Gauthier ARNOULD
 * This file is released under the zlib License (Zlib).
 * See file LICENSE or go to https://opensource.org/licenses/Zlib
 * for full license details.
 */

#pragma once

#include <iostream>

#include "vec.hpp"
#include "quat.hpp"
#include "mat.hpp"
#include "operators.hpp" // vec == vec
#include "Vec_operators.hpp" // for cross

// gcc   : reguire gcc 6.1
// clang : require clang 3.5
// Variable template cannot be used as dependent name
// https://gcc.gnu.org/bugzilla/show_bug.cgi?id=67248
// https://bugs.llvm.org/show_bug.cgi?id=24473

namespace ee {
namespace math {

struct xpos {
    template <typename T>
    constexpr static vec<T, 3> v = { T{1L}, T{0L}, T{0L} };
};

template <typename T>
constexpr vec<T, 3> xpos::v;

struct ypos {
    template <typename T>
    constexpr static vec<T, 3> v = { T{0L}, T{1L}, T{0L} };
};

template <typename T>
constexpr vec<T, 3> ypos::v;

struct zpos {
    template <typename T>
    constexpr static vec<T, 3> v = { T{0L}, T{0L}, T{1L} };
};

template <typename T>
constexpr vec<T, 3> zpos::v;

struct xneg {
    template <typename T>
    constexpr static vec<T, 3> v = { T{ - 1L}, T{0L}, T{0L} };
};

template <typename T>
constexpr vec<T, 3> xneg::v;

struct yneg {
    template <typename T>
    constexpr static vec<T, 3> v = { T{0L}, T{ - 1L}, T{0L} };
};

template <typename T>
constexpr vec<T, 3> yneg::v;

struct zneg {
    template <typename T>
    constexpr static vec<T, 3> v = { T{0L}, T{0L}, T{ - 1L} };
};

template <typename T>
constexpr vec<T, 3> zneg::v;

/**
 * Describe the basis formed by the (I, J, K) triplet.
 * Constraints : I, J and K must be linearly independent vectors chosen from the
 * list xpos, ypos, zpos, xneg, yneg, zneg.
 * Used to view data (point, quaternion, scoords...) from a different basis to
 * make it matches what a formula requires as input or what a user means.
 */
template <typename I, typename J, typename K>
struct basis {
    using i = I;
    using j = J;
    using k = K;

    constexpr static bool is_right_handed =
        cross(i::template v<int>, j::template v<int>) == k::template v<int>;
    constexpr static bool is_left_handed  = ! is_right_handed;
};

// The case where described basis is the initial basis.
using initial_basis = basis<xpos, ypos, zpos>;

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
 * Output formatting
 */
inline std::ostream& operator<<(std::ostream& output, xpos) {
    output << "xpos";

    return output;
}

inline std::ostream& operator<<(std::ostream& output, ypos) {
    output << "ypos";

    return output;
}

inline std::ostream& operator<<(std::ostream& output, zpos) {
    output << "zpos";

    return output;
}

inline std::ostream& operator<<(std::ostream& output, xneg) {
    output << "xneg";

    return output;
}

inline std::ostream& operator<<(std::ostream& output, yneg) {
    output << "yneg";

    return output;
}

inline std::ostream& operator<<(std::ostream& output, zneg) {
    output << "zneg";

    return output;
}

template <typename I, typename J, typename K>
std::ostream& operator<<(std::ostream& output, basis<I, J, K>) {
    output << "basis<" << I{} << ", " << J{} << ", " << K{} << ">";

    return output;
}

} // namespace math
} // namespace ee
