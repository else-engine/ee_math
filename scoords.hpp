/**
 * Copyright (c) 2017 Gauthier ARNOULD
 * This file is released under the zlib License (Zlib).
 * See file LICENSE or go to https://opensource.org/licenses/Zlib
 * for full license details.
 */

#pragma once

#include <iostream>

#include "basis.hpp"

namespace ee {
namespace math {

/**
 * Spherical coordinates.
 * Can be used to represent an orientation.
 *
 * Convention :
 *  - Azimuth reference is basis::i
 *  - Positive azimuthal angle is from basis::i to basis::j
 *  - Zenith is basis::k
 *
 *      z│
 *       │     ● P
 *       │ φ  ╱¦
 *       │╲  ╱ ¦
 *      k│ ╲╱  ¦
 *       │ ╱   ¦
 *       │╱  j ¦    y
 *      O● ────¦─────
 *      ╱ ╲    ¦
 *    i╱───╲   ¦
 *    ╱  θ  ╲  ¦
 *  x╱       ╲ ¦
 *  ╱         ╲¦
 *             ● P'
 *
 *  - O is origin
 *  - Reference plane's equation is y = 0
 *  - P is the described point
 *  - P' is the orthogonal projection of P on reference plane
 *  - theta (θ) is the azimuthal angle between azimuth reference and OP'
 *  - phi (φ) is the polar angle between zenith and OP
 */

/**
 * Orientation part of spherical coordinates.
 */
template <typename T, typename B = initial_basis>
struct scoords_usphere {
    using basis = B;

    union {
        struct { T theta    , phi  ; };
        struct { T azimuthal, polar; };
    };
};

/**
 * Spherical coordinates.
 */
template <typename T, typename B = initial_basis>
struct scoords {
    using basis = B;

    union {
        T r;
        T radius;
    };

    scoords_usphere<T, B> usphere;
};

/**
 * Output formatting
 */
template <typename T, typename B>
std::ostream& operator<<(std::ostream& output, const scoords_usphere<T, B>& scu) {
    output << "scoords_usphere<" << typeid(T).name() << ", " << B{} << "> {" <<
        scu.theta << ", " << scu.phi << "}";

    return output;
}

template <typename T, typename B>
std::ostream& operator<<(std::ostream& output, const scoords<T, B>& sc) {
    output << "scoords<" << typeid(T).name() << ", B> {" <<
        sc.r << ", {" << sc.usphere.theta << ", " << sc.usphere.phi << "}}";

    return output;
}

} // namespace math
} // namespace ee
