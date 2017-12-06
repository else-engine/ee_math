/**
 * Copyright (c) 2017 Gauthier ARNOULD
 * This file is released under the zlib License (Zlib).
 * See file LICENSE or go to https://opensource.org/licenses/Zlib
 * for full license details.
 */

#pragma once

#include <cmath>

#include "vec.hpp"

namespace ee {
namespace math {

/**
 * Return the squared magnitude of a given vector.
 * Works with vec as well as quat.
 */
template <typename V>
constexpr typename V::value_type mag2(const V& v) {
    typename V::value_type ms{};

    for (auto e : v) {
        ms += e * e;
    }

    return ms;
}

/**
 * Return the magnitude of a given vector.
 * Works with vec as well as quat.
 */
template <typename V>
typename V::value_type mag(const V& v) {
    return std::sqrt(mag2(v));
}

/**
 * Return the normalized vector relative to the given one.
 * Works with vec as well as quat.
 */
template <typename V>
V normalize(const V& v) {
    typename V::value_type m = mag(v);

    V r = v / m;

    return r;
}

/**
 * Return the dot product of v1 and v2.
 */
template <typename T, std::size_t D>
constexpr T dot(const vec<T, D>& v1, const vec<T, D>& v2) {
    T ms{};

    for (std::size_t i = 0; i < D; ++ i) {
        ms += v1(i) * v2(i);
    }

    return ms;
}

/**
 * Return the cross product of v1 and v2.
 */
template <typename T>
constexpr vec<T, 3> cross(const vec<T, 3>& v1, const vec<T, 3>& v2) {
    return {v1(1) * v2(2) - v1(2) * v2(1),
            v1(2) * v2(0) - v1(0) * v2(2),
            v1(0) * v2(1) - v1(1) * v2(0)};
}


/**
 * Gram-Schmidt orthonormalize.
 * Only i has to be normalized before calling this function. i and almost_j must
 * not be collinear, but are not required to be orthogonal. Returned vector will
 * be on the common plan i and almost_j already lay on and will be orthogonal to
 * i, pointing to the same half plane (splitted by i) as almost_j is pointing.
 */
template <typename T>
vec<T, 3> orthonormalize(const vec<T, 3>& i, const vec<T, 3>& almost_j) {
    return normalize(almost_j - dot(i, almost_j) * i);
}

/**
 * Generate orthonormal vectors.
 * Only i has to be normalized before calling this function.
 * i represent one of the POSITIVE unit vectors composing the requested basis
 * (in other words, when working with forward as -z, provide backward to this
 * function).
 * i and almost_j must not be collinear, but are not required to be orthogonal.
 * They will be kept on the common plan they already lay on, j will be
 * orthogonal to i, poiting in the same half plane has almost_j. Also almost_j
 * and j can refer to the same variable.
 * k is not required to be initialized.
 */
template <typename T>
void orthonormal_basis(const vec<T, 3>& i, const vec<T, 3>& almost_j,
                       vec<T, 3>* j, vec<T, 3>* k) {
    // i and almost_j are not necessarily orthogonal, normalizing k is required.
    *k = normalize(cross(almost_j, i));

    // i and k are for sure orthonormal so normalizing j is not required.
    *j = cross(i, *k);
}

} // namespace math
} // namespace ee
