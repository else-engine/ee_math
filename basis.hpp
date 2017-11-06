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

    constexpr int x_index  = (i(0) ? 0 : 0) + (i(1) ? 1 : 0) + (i(2) ? 2 : 0);
    constexpr int x_switch =  i(0)      +      i(1)      +      i(2);

    constexpr int y_index  = (j(0) ? 0 : 0) + (j(1) ? 1 : 0) + (j(2) ? 2 : 0);
    constexpr int y_switch =  j(0)      +      j(1)      +      j(2);

    constexpr int z_index  = (k(0) ? 0 : 0) + (k(1) ? 1 : 0) + (k(2) ? 2 : 0);
    constexpr int z_switch =  k(0)      +      k(1)      +      k(2);

    return {
        x_switch * v(x_index),
        y_switch * v(y_index),
        z_switch * v(z_index)};
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

    constexpr int x_index  = (i(0) ? 0 : 0) + (i(1) ? 1 : 0) + (i(2) ? 2 : 0);
    constexpr int x_switch = (i(0)      +      i(1)      +      i(2)) * angle_switch;

    constexpr int y_index  = (j(0) ? 0 : 0) + (j(1) ? 1 : 0) + (j(2) ? 2 : 0);
    constexpr int y_switch = (j(0)      +      j(1)      +      j(2)) * angle_switch;

    constexpr int z_index  = (k(0) ? 0 : 0) + (k(1) ? 1 : 0) + (k(2) ? 2 : 0);
    constexpr int z_switch = (k(0)      +      k(1)      +      k(2)) * angle_switch;

    return {
        x_switch * q(x_index),
        y_switch * q(y_index),
        z_switch * q(z_index),
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
