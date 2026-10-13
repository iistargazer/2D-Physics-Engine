#pragma once
#include "Solver.hpp"

namespace phys {

// One record per manifold, bounded by World's contact capacity. Impulses remain
// in ContactPoint; this transient geometry is not a second persistent cache.
struct TemporalPoint {
    Vec2 localA, localB;
    real separation = 0, bounce = 0;
};
struct TemporalContact {
    std::array<TemporalPoint, 2> points;
};
static_assert(sizeof(TemporalPoint) == 48 && alignof(TemporalPoint) == 8);
static_assert(offsetof(TemporalPoint, separation) == 32 && offsetof(TemporalPoint, bounce) == 40);
static_assert(sizeof(TemporalContact) == 96 && alignof(TemporalContact) == 8);

inline void prepareTemporal(std::vector<ContactManifold>& contacts,
                            std::vector<TemporalContact>& rows, const SolverConfig& cfg) {
    rows.resize(contacts.size());
    for (std::size_t i = 0; i < contacts.size(); ++i) {
        auto& m = contacts[i]; const auto& A = *m.bodyA; const auto& B = *m.bodyB;
        m.friction = std::sqrt(A.friction * B.friction);
        m.restitution = std::max(A.restitution, B.restitution);
        for (int p = 0; p < m.count; ++p) {
            auto& c = m.contactPoints[p]; auto& row = rows[i].points[p];
            row.localA = A.rot.transposed() * (c.position - A.position);
            row.localB = B.rot.transposed() * (c.position - B.position);
            row.separation = -c.depth;
            const real vn = dot(B.velocityAt(c.position - B.position)
                              - A.velocityAt(c.position - A.position), m.normal);
            row.bounce = vn < -cfg.restitutionThreshold ? -m.restitution * vn : 0;
        }
    }
}

struct TemporalSoftness { real gamma, beta; };
inline TemporalSoftness temporalSoftness(const SolverConfig& cfg, real h) {
    if (cfg.compliance == 0) return {0, cfg.baumgarte / cfg.substeps};
    const real denominator = h + cfg.damping * cfg.compliance;
    return {cfg.compliance / (h * denominator), h / denominator};
}

// Accumulators represent one substep's impulse. The preceding substep estimate
// is reapplied once, not once per iteration; this is a seed, not an extra force.
inline void refreshTemporal(std::vector<ContactManifold>& contacts,
                            const std::vector<TemporalContact>& rows,
                            const SolverConfig& cfg, real h, bool first) {
    const auto [gamma, beta] = temporalSoftness(cfg, h);
    for (std::size_t i = 0; i < contacts.size(); ++i) {
        auto& m = contacts[i]; auto& A = *m.bodyA; auto& B = *m.bodyB;
        if (!rowIsLive(A, B)) continue;
        const Vec2 n = m.normal, t{n.y, -n.x};
        for (int p = 0; p < m.count; ++p) {
            auto& c = m.contactPoints[p]; const auto& row = rows[i].points[p];
            c.rA = A.rot * row.localA; c.rB = B.rot * row.localB;
            const real separation = row.separation
                + dot(B.position + c.rB - (A.position + c.rA), n);
            c.depth = -separation;
            const real rnA = cross(c.rA, n), rnB = cross(c.rB, n);
            const real rtA = cross(c.rA, t), rtB = cross(c.rB, t);
            const real kn = A.invMass + B.invMass + A.invInertia * rnA * rnA + B.invInertia * rnB * rnB;
            const real kt = A.invMass + B.invMass + A.invInertia * rtA * rtA + B.invInertia * rtB * rtB;
            c.massN = kn > 0 ? 1 / (kn + gamma) : 0;
            c.massT = kt > 0 ? 1 / kt : 0;
            // Positive separation is speculative: allow closing only up to the gap.
            const real push = separation > 0 ? -separation / h
                : beta / h * std::max(-separation - cfg.slop, real(0));
            c.targetVn = first && row.bounce > 0 ? std::max(push, row.bounce) : push;
            if (!cfg.warmStarting) c.accN = c.accT = 0;
            c.accN = std::max(c.accN, real(0));
            c.accT = clamp(c.accT, -m.friction * c.accN, m.friction * c.accN);
            const Vec2 impulse = n * c.accN + t * c.accT;
            A.applyImpulse(-impulse, c.rA); B.applyImpulse(impulse, c.rB);
        }
    }
}

inline void solveTemporal(std::vector<ContactManifold>& contacts,
                          const SolverConfig& cfg, real h) {
    const real gamma = temporalSoftness(cfg, h).gamma;
    for (auto& m : contacts) {
        auto& A = *m.bodyA; auto& B = *m.bodyB;
        if (!rowIsLive(A, B)) continue;
        const Vec2 n = m.normal, t{n.y, -n.x};
        bool blockDone = false;
        if (cfg.blockSolve && m.count == 2 && m.contactPoints[0].massN > 0 && m.contactPoints[1].massN > 0) {
            auto& a = m.contactPoints[0]; auto& b = m.contactPoints[1];
            const real k11 = 1 / a.massN, k22 = 1 / b.massN;
            const real k12 = A.invMass + B.invMass
                + A.invInertia * cross(a.rA, n) * cross(b.rA, n)
                + B.invInertia * cross(a.rB, n) * cross(b.rB, n);
            const Mat22 K{k11, k12, k12, k22};
            if (K.determinant() > kEps && K.determinant() > 1e-6 * k11 * k22) {
                const Vec2 old{a.accN, b.accN};
                const Vec2 rhs = Vec2{
                    dot(B.velocityAt(a.rB) - A.velocityAt(a.rA), n) - a.targetVn + gamma * a.accN,
                    dot(B.velocityAt(b.rB) - A.velocityAt(b.rA), n) - b.targetVn + gamma * b.accN} - K * old;
                Vec2 x = K.solve(-rhs);
                blockDone = x.x >= 0 && x.y >= 0;
                if (!blockDone) { x = {-rhs.x / k11, 0}; blockDone = x.x >= 0 && k12 * x.x + rhs.y >= 0; }
                if (!blockDone) { x = {0, -rhs.y / k22}; blockDone = x.y >= 0 && k12 * x.y + rhs.x >= 0; }
                if (!blockDone) { x = {}; blockDone = rhs.x >= 0 && rhs.y >= 0; }
                if (blockDone) {
                    const Vec2 delta = x - old;
                    A.applyImpulse(-n * delta.x, a.rA); B.applyImpulse(n * delta.x, a.rB);
                    A.applyImpulse(-n * delta.y, b.rA); B.applyImpulse(n * delta.y, b.rB);
                    a.accN = x.x; b.accN = x.y;
                }
            }
        }
        for (int p = 0; !blockDone && p < m.count; ++p) {
            auto& c = m.contactPoints[p];
            const real vn = dot(B.velocityAt(c.rB) - A.velocityAt(c.rA), n);
            const real old = c.accN;
            c.accN = std::max(real(0), old + c.massN * (c.targetVn - vn - gamma * old));
            A.applyImpulse(-n * (c.accN - old), c.rA);
            B.applyImpulse(n * (c.accN - old), c.rB);
        }
        // Friction follows normals so its cone uses current substep support.
        for (int p = 0; p < m.count; ++p) {
            auto& c = m.contactPoints[p];
            const real old = c.accT;
            c.accT = clamp(old - c.massT * dot(B.velocityAt(c.rB) - A.velocityAt(c.rA), t),
                           -m.friction * c.accN, m.friction * c.accN);
            A.applyImpulse(-t * (c.accT - old), c.rA);
            B.applyImpulse(t * (c.accT - old), c.rB);
        }
    }
}

} // namespace phys
