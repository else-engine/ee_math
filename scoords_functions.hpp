/**
 * Copyright (c) 2017 Gauthier ARNOULD
 * This file is released under the zlib License (Zlib).
 * See file LICENSE or go to https://opensource.org/licenses/Zlib
 * for full license details.
 */

#pragma once

#include <cmath>

#include "scoords.hpp"

#include "vec.hpp"
#include "quat.hpp"

namespace ee {
namespace math {

/**
 * Return unit vector pointing in same direction than input.
 */
template <typename T, typename B>
vec<T, 3> vec_from(const scoords_usphere<T, B>& scu) {
    const T cos_theta = std::cos(scu.theta);
    const T sin_theta = std::sin(scu.theta);
    const T cos_phi   = std::cos(scu.phi);
    const T sin_phi   = std::sin(scu.phi);

    return from_basis<B>(
        vec<T, 3>{cos_theta * sin_phi, sin_theta * sin_phi, cos_phi});
}

/**
 * Return cartesian coordinates from spherical coordinates.
 */
template <typename T, typename B>
vec<T, 3> vec_from(const scoords<T, B>& sc) {
    return sc.r * vec_from(sc.usphere);
}

/**
 * Return the quaternion describing rotation required to transform scu's
 * azimuth reference into scu's direction vector.
 */
template <typename T, typename B>
quat<T> quat_from(const scoords_usphere<T, B>& scu) {
    const T     t = T{0.5L} * scu.theta;
    const T cos_t = std::cos(t);
    const T sin_t = std::sin(t);

    // Phi is angle from zenith CCW, we are not at zenith but on reference plane
    // so we need Pi/2 - Phi, but we will have to rotate CW so Phi - Pi/2
    const T     p = T{0.5L} * (scu.phi - c_half_pi<T>);
    const T cos_p = std::cos(p);
    const T sin_p = std::sin(p);

    return from_basis<B>(quat<T>{
        - sin_t * sin_p,
          cos_t * sin_p,
          sin_t * cos_p,
          cos_t * cos_p});
}

namespace detail {

template <typename B, typename T>
scoords_usphere<T, B> scoords_usphere_from(const vec<T, 3>& p) {
    const T theta = std::atan2(p.y, p.x);
    const T phi   = std::acos(p.z);

    return {theta, phi};
}

} // namespace detail

/**
 * Return scoords_usphere from any cartesian coordinates of the unit sphere.
 * Input vector must be normalized.
 */
template <typename B = initial_basis, typename T>
scoords_usphere<T, B> scoords_usphere_from(const vec<T, 3>& xyz) {
    const vec<T, 3> p = to_basis<B>(xyz);

    return detail::scoords_usphere_from<B>(p);
}

/**
 * Return scoords from cartesian coordinates.
 */
template <typename B = initial_basis, typename T>
scoords<T, B> scoords_from(const vec<T, 3>& xyz) {
    const vec<T, 3> p = to_basis<B>(xyz);

    const T m = mag(p);

    // we only divide z since x/y is equal to (x/l)/(z/l) so atan2 gives same
    // result anyway.
    const scoords_usphere<T, B> scu =
        detail::scoords_usphere_from<B>(vec<T, 3>{p.x, p.y, p.z / m});

    return {m, scu};
}

} // namespace math
} // namespace ee
