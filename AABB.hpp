#pragma once
// A shape lies inside its AABB: disjoint bounds imply disjoint shapes, but
// overlapping bounds do NOT imply contact. Use inclusive overlap so touching
// bounds cannot discard a contact accepted by the narrowphase.
#include "RigidBody.hpp"
#include <limits>

namespace phys {

struct AABB {
    Vec2 min, max;

    bool overlaps(const AABB& b) const {
        return min.x <= b.max.x && max.x >= b.min.x
            && min.y <= b.max.y && max.y >= b.min.y;
    }
    bool contains(const AABB& b) const {
        return min.x <= b.min.x && min.y <= b.min.y
            && max.x >= b.max.x && max.y >= b.max.y;
    }
    AABB fatten(real margin) const {
        const Vec2 padding{margin, margin};
        return {min - padding, max + padding};
    }
    // 2D surface-area heuristic uses boundary length, not area:
    // P = 2(width + height). Larger enclosing bounds tend to attract more
    // queries; minimizing growth of P is a cheap spatial locality heuristic.
    real perimeter() const { return 2 * ((max.x - min.x) + (max.y - min.y)); }
    static AABB unite(const AABB& a, const AABB& b) {
        return {{std::min(a.min.x, b.min.x), std::min(a.min.y, b.min.y)},
                {std::max(a.max.x, b.max.x), std::max(a.max.y, b.max.y)}};
    }
};

inline AABB computeAABB(const RigidBody& body) {
    if (body.shape.type == ShapeType::Circle) {
        const Vec2 r{body.shape.radius, body.shape.radius};
        return {body.position - r, body.position + r};
    }
    const real inf = std::numeric_limits<real>::infinity();
    AABB bounds{{inf, inf}, {-inf, -inf}};
    for (std::size_t i = 0; i < body.shape.verts.size(); ++i) {
        const Vec2 p = body.worldVertex(i);
        bounds.min.x = std::min(bounds.min.x, p.x);
        bounds.min.y = std::min(bounds.min.y, p.y);
        bounds.max.x = std::max(bounds.max.x, p.x);
        bounds.max.y = std::max(bounds.max.y, p.y);
    }
    return bounds;
}

} // namespace phys
