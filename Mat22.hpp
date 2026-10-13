#pragma once
// ============================================================================
// Mat22 - 2x2 matrix, row-major:   | m00 m01 |
//                                  | m10 m11 |
//
// Why we need it:
//  * Rotation matrix R(theta) maps body-local coordinates to world space.
//    World vertex = position + R * localVertex.
//  * R is orthogonal (R^-1 = R^T, det = +1), so "inverse rotation" is a free
//    transpose - never call the general inverse() for rotations.
//  * The general inverse()/solve() is for things like the 2x2 point-to-point
//    (revolute) joint constraint in Phase 5: K * lambda = -Cdot.
// ============================================================================
#include "Vec2.hpp"
#include <cassert>

namespace phys {

struct Mat22 {
    real m00 = 1, m01 = 0;
    real m10 = 0, m11 = 1;

    static Mat22 rotation(real angle) {
        const real c = std::cos(angle), s = std::sin(angle);
        return {c, -s, s, c};        // columns are the rotated basis vectors
    }

    constexpr Vec2 operator*(Vec2 v) const {
        return {m00 * v.x + m01 * v.y, m10 * v.x + m11 * v.y};
    }

    constexpr Mat22 operator*(const Mat22& o) const {
        return {m00 * o.m00 + m01 * o.m10, m00 * o.m01 + m01 * o.m11,
                m10 * o.m00 + m11 * o.m10, m10 * o.m01 + m11 * o.m11};
    }

    constexpr Mat22 transposed() const { return {m00, m10, m01, m11}; }
    constexpr real determinant() const { return m00 * m11 - m01 * m10; }

    // Adjugate / determinant. Asserts on singular matrices.
    Mat22 inverse() const {
        const real det = determinant();
        assert(std::abs(det) > kEps && "singular Mat22");
        const real inv = real(1) / det;
        return { m11 * inv, -m01 * inv,
                -m10 * inv,  m00 * inv};
    }

    // Solve A x = b via Cramer's rule (cheaper & more accurate than inverse()*b).
    Vec2 solve(Vec2 b) const {
        real det = determinant();
        assert(std::abs(det) > kEps && "singular Mat22");
        det = real(1) / det;
        return {det * (m11 * b.x - m01 * b.y), det * (m00 * b.y - m10 * b.x)};
    }
};

} // namespace phys
