/**
 * Copyright (c) 2017 Gauthier ARNOULD
 * This file is released under the zlib License (Zlib).
 * See file LICENSE or go to https://opensource.org/licenses/Zlib
 * for full license details.
 */

#pragma once

#include <cstddef>
#include <iostream>
#include <typeinfo>

namespace ee {
namespace math {

/**
 * Generic vector template.
 */
template <typename T, std::size_t D>
struct vec {
    static_assert(std::is_arithmetic<T>::value, "T must be arithmetic type");
    static_assert(D > 0, "D must be at least 1");

    using value_type      = T;
    using reference       = value_type&;
    using const_reference = const value_type&;
    using pointer         = value_type*;
    using const_pointer   = const value_type*;

    constexpr static std::size_t size = D;

    T data[size];

    inline constexpr auto& operator()(std::size_t d) {
        return data[d];
    }

    inline constexpr const auto& operator()(std::size_t d) const {
        return data[d];
    }

    inline constexpr explicit operator bool() const {
        for (auto v : data) {
            if (v) {
                return false;
            }
        }

        return true;
    }
};

/**
 * Note on union used in below specializations.
 *
 * C++14, section 9.2 §18 :
 *
 * If a standard-layout union contains two or more standard-layout structs that
 * share a common initial sequence, and if the standard-layout union object
 * currently contains one of these standard-layout structs, it is permitted to
 * inspect the common initial part of any of them. Two standard-layout structs
 * share a common initial sequence if corresponding members have
 * layout-compatible types and either neither member is a bit-field or both are
 * bit-fields with the same width for a sequence of one or more initial members.
 */

/**
 * 1D vector specialization.
 */
template <typename T>
struct vec<T, 1> {
    static_assert(std::is_arithmetic<T>::value, "T must be arithmetic type");

    using value_type      = T;
    using reference       = value_type&;
    using const_reference = const value_type&;
    using pointer         = value_type*;
    using const_pointer   = const value_type*;

    constexpr static std::size_t size = 1;

    union {
        T data[size];

        struct { T x; };

        struct { T r; };

        struct { T s; };

        struct { T i; };
    };

    inline constexpr auto& operator()(std::size_t d) {
        return data[d];
    }

    inline constexpr const auto& operator()(std::size_t d) const {
        return data[d];
    }

    inline constexpr explicit operator bool() const {
        return x;
    }
};

/**
 * 2D vector specialization.
 */
template <typename T>
struct vec<T, 2> {
    static_assert(std::is_arithmetic<T>::value, "T must be arithmetic type");

    using value_type      = T;
    using reference       = value_type&;
    using const_reference = const value_type&;
    using pointer         = value_type*;
    using const_pointer   = const value_type*;

    constexpr static std::size_t size = 2;

    union {
        T data[size];

        struct { T x, y; };

        struct { T r, g; };

        struct { T s, t; };

        struct { T i, j; };

        struct { T w, h; };
    };

    inline constexpr auto& operator()(std::size_t d) {
        return data[d];
    }

    inline constexpr const auto& operator()(std::size_t d) const {
        return data[d];
    }

    inline constexpr explicit operator bool() const {
        return x || y;
    }
};

/**
 * 3D vector specialization.
 */
template <typename T>
struct vec<T, 3> {
    static_assert(std::is_arithmetic<T>::value, "T must be arithmetic type");

    using value_type      = T;
    using reference       = value_type&;
    using const_reference = const value_type&;
    using pointer         = value_type*;
    using const_pointer   = const value_type*;

    constexpr static std::size_t size = 3;

    union {
        T data[size];

        struct { T x, y, z; };
        vec<T, 2> xy;

        struct { T r, g, b; };
        vec<T, 2> rg;

        struct { T s, t, p; };
        vec<T, 2> st;

        struct { T i, j, k; };
        vec<T, 2> ij;
    };

    inline constexpr auto& operator()(std::size_t d) {
        return data[d];
    }

    inline constexpr const auto& operator()(std::size_t d) const {
        return data[d];
    }

    inline constexpr explicit operator bool() const {
        return x || y || z;
    }
};

/**
 * 4D vector specialization.
 */
template <typename T>
struct vec<T, 4> {
    static_assert(std::is_arithmetic<T>::value, "T must be arithmetic type");

    using value_type      = T;
    using reference       = value_type&;
    using const_reference = const value_type&;
    using pointer         = value_type*;
    using const_pointer   = const value_type*;

    constexpr static std::size_t size = 4;

    union {
        T data[size];

        struct { T x, y, z, w; };
        vec<T, 3> xyz;
        struct { vec<T, 2> xy, zw; };

        struct { T r, g, b, a; };
        vec<T, 3> rgb;
        struct { vec<T, 2> rg, ba; };

        struct { T s, t, p, q; };
        vec<T, 3> stp;
        struct { vec<T, 2> st, pq; };

        struct { T i, j, k, l; };
        vec<T, 3> ijk;
        struct { vec<T, 2> ij, kl; };
    };

    inline constexpr auto& operator()(std::size_t d) {
        return data[d];
    }

    inline constexpr const auto& operator()(std::size_t d) const {
        return data[d];
    }

    inline constexpr explicit operator bool() const {
        return x || y || z || w;
    }
};

/**
 * A way to identify vectors.
 */
namespace detail {

template <typename>
struct is_vec_impl : std::false_type {};

template <typename T, std::size_t D>
struct is_vec_impl<vec<T, D>> : std::true_type {};

} // namespace detail

template <typename T>
constexpr bool is_vec = detail::is_vec_impl<std::decay_t<T>>::value;

/**
 * Output formatting
 */
template <typename T, std::size_t D>
std::ostream& operator<<(std::ostream& output, const vec<T, D>& v) {
    output << "vec<" << typeid(T).name() << ", " << D << "> {";

    for (std::size_t d = 0; d < D; ++ d) {
        output << v(d) << (d + 1 == D ? "}" : ", ");
    }

    return output;
}

} // namespace math
} // namespace ee