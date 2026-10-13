#pragma once
#include "RigidBody.hpp"
#include <array>
#include <cstdint>

namespace phys {

// One contact point plus the solver's cached per-point data.
struct ContactPoint {
    ContactPoint() = default;
    ContactPoint(Vec2 p, real d) : position(p), depth(d) {}

    Vec2 position;           // world space
    std::uint64_t featureId = 0; // geometric identity, never a world-point hash
    real depth = 0;          // penetration depth at THIS point (>= 0)

    // ---- filled by ContactSolver::prepare() ----
    Vec2 rA, rB;             // lever arms from each COM to the contact
    real massN = 0;          // 1 / K_n  (effective mass along the normal)
    real massT = 0;          // 1 / K_t  (effective mass along the tangent)
    real targetVn = 0;       // desired post-solve normal velocity (bounce + Baumgarte)
    real accN = 0;           // accumulated normal impulse   (clamped >= 0)
    real accT = 0;           // accumulated tangent impulse  (clamped |.| <= mu*accN)
};

struct ContactManifold {
    RigidBody* bodyA = nullptr;
    RigidBody* bodyB = nullptr;
    Vec2 normal;                          // unit, points from A toward B
    real penetration = 0;                 // deepest point's depth
    std::array<ContactPoint, 2> contactPoints;
    std::size_t indexA = 0, indexB = 0; // stable within a synchronized step
    int count = 0;                        // 1 or 2 valid points

    real friction = 0;                    // mixed: sqrt(muA * muB)
    real restitution = 0;                 // mixed: max(eA, eB)
};

} // namespace phys
