/**
 * Copyright (c) 2017-2018 Gauthier ARNOULD
 * This file is released under the zlib License (Zlib).
 * See file LICENSE or go to https://opensource.org/licenses/Zlib
 * for full license details.
 */

#pragma once

#include <iostream>

#include "vec.hpp"
#include "quat.hpp"
#include "mat.hpp"
#include "operators.hpp"
#include "vec_functions.hpp"

// gcc   : reguire gcc 6.1
// clang : require clang 3.5
// Variable template cannot be used as dependent name
// https://gcc.gnu.org/bugzilla/show_bug.cgi?id=67248
// https://bugs.llvm.org/show_bug.cgi?id=24473

namespace ee {
namespace math {

struct xpos {
    template <typename T>
    constexpr static vec<T, 3> v { T{1L}, T{0L}, T{0L} };
};

struct ypos {
    template <typename T>
    constexpr static vec<T, 3> v { T{0L}, T{1L}, T{0L} };
};

struct zpos {
    template <typename T>
    constexpr static vec<T, 3> v { T{0L}, T{0L}, T{1L} };
};

struct xneg {
    template <typename T>
    constexpr static vec<T, 3> v { T{ - 1L}, T{0L}, T{0L} };
};

struct yneg {
    template <typename T>
    constexpr static vec<T, 3> v { T{0L}, T{ - 1L}, T{0L} };
};

struct zneg {
    template <typename T>
    constexpr static vec<T, 3> v { T{0L}, T{0L}, T{ - 1L} };
};

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
