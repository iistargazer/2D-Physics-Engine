#pragma once
// ============================================================================
// Sequential Impulse contact solver (velocity level, Gauss-Seidel / PGS).
//
// Notation: n = contact normal (A -> B), t = tangent, rA/rB = lever arms.
//   Contact-point velocity:  v_P = v + w x r
//   Relative velocity:       vrel = (vB + wB x rB) - (vA + wA x rA)
//   Normal speed:            vn   = vrel . n    (vn < 0  => approaching)
//
// ---- Impulse derivation ----------------------------------------------------
// Apply impulse P = lambda * n to B and -P to A. Then
//   vB' = vB + mB^-1 P            wB' = wB + IB^-1 (rB x P)
//   vA' = vA - mA^-1 P            wA' = wA - IA^-1 (rA x P)
// so  vn' = vn + lambda * K,   with the scalar "effective inverse mass"
//   K = mA^-1 + mB^-1 + IA^-1 (rA x n)^2 + IB^-1 (rB x n)^2.
// Demanding the post-impulse normal speed equals a target (Newton restitution:
// vn' = -e vn) gives
//   lambda = (target - vn) / K = -(1+e) vn / K        (classic J formula).
//
// Sequential impulses solve the contacts ONE AT A TIME, iterating N times.
// Each solve changes velocities that other contacts see, so the iteration is
// Gauss-Seidel on a Linear Complementarity Problem (lambda >= 0, vn' >= 0,
// lambda * vn' = 0). The trick that makes it converge to the LCP solution is
// ACCUMULATION: clamp the TOTAL impulse, not each increment:
//   new = max(acc + lambda, 0);  lambda = new - acc;  acc = new;
//
// ---- Friction (Coulomb) ----------------------------------------------------
// Same recipe along the tangent t = (n.y, -n.x):
//   lambda_t = -vt / K_t,   |acc_t| <= mu * acc_n   (friction cone, box-clamped)
// Friction is solved BEFORE normal so the non-penetration constraint wins.
//
// ---- Baumgarte stabilisation ------------------------------------------------
// Velocity-level solving ignores position error C = -depth, so bodies drift
// into each other. Demand C' = -beta/dt * C (error decays exponentially):
//   target vn >= (beta/dt) * max(depth - slop, 0)
// - beta in [0.1, 0.3]; too big => jitter/energy injection.
// - slop (~0.01) lets shapes keep a small overlap so contacts persist
//   frame to frame (otherwise resting contacts flicker on/off).
// We combine with restitution via max(): bounce and push-out are not additive.
// ============================================================================
#include "SolverRows.hpp"
#include <vector>

namespace phys {

enum class SolverMode { PGS, TGS };

struct SolverConfig {
    SolverMode mode = SolverMode::PGS;
    int substeps = 8;
    real compliance = 0;            // per contact point, m/N; zero selects rigid contact
    real damping = 0;               // per contact point, N*s/m
    int iterations = 10;
    bool warmStarting = true;
    bool blockSolve = true;
    bool positionCorrection = false; // opt-in NGS; retains Baumgarte comparison
    int positionIterations = 10;
    real positionBeta = 0.3;
    real baumgarte = 0.2;           // beta
    real slop = 0.01;               // allowed penetration
    real restitutionThreshold = 1;  // below this closing speed, no bounce (kills jitter)
};

class ContactSolver {
public:
    explicit ContactSolver(SolverConfig cfg = {}) : cfg_(cfg) {}

    // Precompute everything that stays constant across iterations.
    // MUST be called after integrateForces() so vn0 includes gravity.
    void prepare(std::vector<ContactManifold>& manifolds, real dt) const {
        for (auto& m : manifolds) {
            RigidBody& A = *m.bodyA;
            RigidBody& B = *m.bodyB;
            // A contact between two sleeping (or static) bodies needs no work this
            // frame, but it MUST stay in the manifold list: it is the edge of the
            // island graph used to wake whole islands later. `rowIsLive` is the one
            // definition of "this block does work", shared with RowSet and TGS.
            if (!rowIsLive(A, B)) continue;
            const Vec2 n = m.normal;
            const Vec2 t{n.y, -n.x};

            m.friction = std::sqrt(A.friction * B.friction);       // geometric mean
            m.restitution = std::max(A.restitution, B.restitution);

            for (int i = 0; i < m.count; ++i) {
                ContactPoint& c = m.contactPoints[i];
                c.rA = c.position - A.position;
                c.rB = c.position - B.position;

                const real rnA = cross(c.rA, n), rnB = cross(c.rB, n);
                const real kN = A.invMass + B.invMass
                              + A.invInertia * rnA * rnA + B.invInertia * rnB * rnB;
                c.massN = kN > 0 ? 1 / kN : 0;

                const real rtA = cross(c.rA, t), rtB = cross(c.rB, t);
                const real kT = A.invMass + B.invMass
                              + A.invInertia * rtA * rtA + B.invInertia * rtB * rtB;
                c.massT = kT > 0 ? 1 / kT : 0;

                // Restitution target uses the PRE-solve closing speed.
                const real vn0 = dot(B.velocityAt(c.rB) - A.velocityAt(c.rA), n);
                const real bounce = vn0 < -cfg_.restitutionThreshold ? -m.restitution * vn0 : 0;
                const real push = (cfg_.positionCorrection ? 0 : cfg_.baumgarte / dt) * std::max(c.depth - cfg_.slop, real(0));
                c.targetVn = std::max(bounce, push);

                if (!cfg_.warmStarting) c.accN = c.accT = 0;
                c.accN = std::max(c.accN, real(0));
                c.accT = clamp(c.accT, -m.friction * c.accN, m.friction * c.accN);
            }
        }
        // Apply only AFTER every restitution target has been computed: seeding
        // one contact must not contaminate another contact's pre-solve vn0.
        // J^T lambda changes momenta by equal/opposite impulses. The next
        // PGS sweeps solve for increments around this previous-frame estimate.
        for (auto& m : manifolds) {
            // Same sleeping guard as above: a sleeping pair kept its cached
            // impulses, but `rA`/`rB` and `massN` were not recomputed for it this
            // frame, so re-applying them would inject stale impulses and jitter the
            // island awake forever.
            if (!rowIsLive(*m.bodyA, *m.bodyB)) continue;
            const Vec2 t{m.normal.y, -m.normal.x};
            for (int i = 0; i < m.count; ++i) {
                auto& c = m.contactPoints[i];
                const Vec2 p = m.normal * c.accN + t * c.accT;
                m.bodyA->applyImpulse(-p, c.rA);
                m.bodyB->applyImpulse(p, c.rB);
            }
        }
    }

    // N Gauss-Seidel sweeps.
    void solve(std::vector<ContactManifold>& manifolds) const {
        for (int it = 0; it < cfg_.iterations; ++it) solveIteration(manifolds);
    }

    void solveIteration(std::vector<ContactManifold>& manifolds) const {
            for (auto& m : manifolds) {
                RigidBody& A = *m.bodyA;
                RigidBody& B = *m.bodyB;
                if (!rowIsLive(A, B)) continue;
                const Vec2 n = m.normal;
                const Vec2 t{n.y, -n.x};

                // ---- friction (tangential) ----
                for (int i = 0; i < m.count; ++i) {
                    ContactPoint& c = m.contactPoints[i];
                    const Vec2 vrel = B.velocityAt(c.rB) - A.velocityAt(c.rA);
                    real lambda = -dot(vrel, t) * c.massT;

                    const real maxF = m.friction * c.accN;               // Coulomb cone
                    const real newAcc = clamp(c.accT + lambda, -maxF, maxF);
                    lambda = newAcc - c.accT;
                    c.accT = newAcc;

                    const Vec2 P = t * lambda;
                    A.applyImpulse(-P, c.rA);
                    B.applyImpulse(P, c.rB);
                }

                // ---- normal (non-penetration + bounce) ----
                // Two supports share translational/angular velocities. Solve
                // their 2x2 LCP together: w = K*x + b >= 0, x >= 0,
                // x_i*w_i = 0. Enumerate both active, each single active, none.
                // b = current vn - target - K*old removes the old accumulated
                // contribution before solving for the new TOTAL impulse.
                bool blockDone = false;
                if (cfg_.blockSolve && m.count == 2) {
                    auto& c1 = m.contactPoints[0]; auto& c2 = m.contactPoints[1];
                    const real k11 = 1 / c1.massN, k22 = 1 / c2.massN;
                    const real k12 = A.invMass + B.invMass
                        + A.invInertia * cross(c1.rA, n) * cross(c2.rA, n)
                        + B.invInertia * cross(c1.rB, n) * cross(c2.rB, n);
                    const Mat22 K{k11, k12, k12, k22};
                    if (K.determinant() > 1e-6 * k11 * k22) {
                        const Vec2 old{c1.accN, c2.accN};
                        const Vec2 rhs = Vec2{
                            dot(B.velocityAt(c1.rB) - A.velocityAt(c1.rA), n) - c1.targetVn,
                            dot(B.velocityAt(c2.rB) - A.velocityAt(c2.rA), n) - c2.targetVn} - K * old;
                        Vec2 x = K.solve(-rhs);
                        if (x.x >= 0 && x.y >= 0) blockDone = true;
                        if (!blockDone) {
                            x = {-rhs.x / k11, 0};
                            blockDone = x.x >= 0 && k12 * x.x + rhs.y >= 0;
                        }
                        if (!blockDone) {
                            x = {0, -rhs.y / k22};
                            blockDone = x.y >= 0 && k12 * x.y + rhs.x >= 0;
                        }
                        if (!blockDone) { x = {}; blockDone = rhs.x >= 0 && rhs.y >= 0; }
                        if (blockDone) {
                            const Vec2 delta = x - old;
                            A.applyImpulse(-n * delta.x, c1.rA); B.applyImpulse(n * delta.x, c1.rB);
                            A.applyImpulse(-n * delta.y, c2.rA); B.applyImpulse(n * delta.y, c2.rB);
                            c1.accN = x.x; c2.accN = x.y;
                        }
                    }
                }
                if (blockDone) continue;
                for (int i = 0; i < m.count; ++i) {
                    ContactPoint& c = m.contactPoints[i];
                    const Vec2 vrel = B.velocityAt(c.rB) - A.velocityAt(c.rA);
                    real lambda = (c.targetVn - dot(vrel, n)) * c.massN;

                    const real newAcc = std::max(c.accN + lambda, real(0));  // push only
                    lambda = newAcc - c.accN;
                    c.accN = newAcc;

                    const Vec2 P = n * lambda;
                    A.applyImpulse(-P, c.rA);
                    B.applyImpulse(P, c.rB);
                }
            }
    }

    SolverConfig& config() { return cfg_; }

private:
    SolverConfig cfg_;
};

} // namespace phys
