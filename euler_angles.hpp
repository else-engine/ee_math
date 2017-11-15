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
 * Euler angles.
 * Proper Euler angles definition. See tait_bryan_angles for other sequences.
 *
 *  - Rotation of angle alpha (α) around z
 *  - Rotation of angle beta  (β) around x'
 *  - Rotation of angle gamma (γ) around z"
 *
 * So, it always works as zx'z" intrinsic sequence. The way to use another
 * intrinsic sequence is to change basis B making another type of euler_angles
 * template instanciated.
 *
 *       │z
 * ╲z"   │     ╱y"
 *  ╲    │    ╱
 *   ╲ β╱│   ╱
 *    ╲╱ │  ╱
 *     ╲k│ ╱
 *      ╲│╱ j
 *       ● ──────────
 *     i╱│╲         y
 *     ╱ │ ╲
 *    ╱  │ ╱╲
 *   ╱───│╱γ ╲
 * x╱  α │    ╲
 * ╱     │   x"╲
 *     x'│
 *
 *          │z
 *     ╲z"  │    ╱x"
 *      ╲ β │   ╱
 *       ╲──│  ╱
 *        ╲ │ ╱│
 * x       ╲│╱ │
 * ─────────●  │
 *        ╲ │╲╱ γ
 *       α ╲│╱╲
 *          │  ╲
 *          │  y╲
 *        x'│
 *
 */
template <typename T, typename B = basis<xpos, ypos, zpos>>
struct euler_angles {
    using basis = B;

    union {
        struct { T alpha, beta , gamma; };
        struct { T phi  , theta, psi  ; };
    };
};

/**
 * Tait-Bryan angles.
 * Usual convention for aerospace.
 *
 *  - Rotation of angle alpha (α) around z  (yaw)
 *  - Rotation of angle beta  (β) around y' (pitch)
 *  - Rotation of angle gamma (γ) around x" (roll)
 */
template <typename T, typename B = basis<zpos, xpos, ypos>>
struct tait_bryan_angles {
    using basis = B;

    union {
        struct { T alpha, beta , gamma; };
        struct { T phi  , theta, psi  ; };
    };
};

} // namespace math
} // namespace ee
