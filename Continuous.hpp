#pragma once
// ============================================================================
// Conservative advancement (CCD) for fast, thin "bullet" bodies.
//
// ---- Why discrete steps tunnel -------------------------------------------------
// A discrete step samples shapes at x(t) and x(t+dt) only. A 0.02 m thick wall and
// a body travelling 1000 m/s * (1/60) s = 16.7 m per step can cross each other
// with no sample inside the overlap: the pair is never tested while touching.
//
// ---- Conservative advancement ---------------------------------------------------
// Let d(A,B) be the true shape distance and g a LOWER bound on it (we use the
// pairwise separation maximum, which is <= d). Every point of a rigid body moves at
// speed <= |v| + |w| * r, where r is the farthest vertex radius, so the relative
// closing speed is bounded by
//        V = |vB - vA| + |wA| rA + |wB| rB.
// Advancing by g / V therefore cannot step past first contact. Repeating at the new
// pose re-selects a separating axis and shrinks the interval. This is the standard
// conservative-advancement argument: progress is guaranteed because g >= 0 and V is
// constant over the constant-velocity window.
//
// ---- Touching is a contact, not a miss ------------------------------------------
// At g = 0 the advance 0/V = 0 stalls, so g <= 0 is treated as the CONTACT EVENT.
// Declaring it "separated" is exactly what lets a body tunnel. Two further rules
// keep the sweep physical:
//   * impact impulses are one-sided (lambda >= 0): CCD can push but never pull;
//   * a pair already touching at t = 0 belongs to the discrete/narrowphase solver
//     this step, so CCD skips it (counted in `skipped`) rather than freezing.
//
// ---- Impacts are resolved at the centre of mass ----------------------------------
// The sweep's impulse is applied with r = 0: it removes the relative normal motion,
// and deliberately does NOT add angular velocity. Conservative advancement bounds
// *when* a pair touches; the pose it stops at is only accurate to the separation
// bound, so the contact point - and with it the lever arm - is not known to
// sub-step accuracy. Injecting a one-step torque from an approximate lever arm is
// catastrophic for thin bodies: a 3 mm plate tilted 0.12 rad against a wall picked
// up ~8e3 rad/s from a 0.006 m offset, and then held a motionless contact point
// while its centre kept drifting into the wall at 72 m/s. Stopping the relative
// normal velocity is the sweep's job; the lever arm is the discrete solver's, and
// it gets the real manifold on the steps that follow.
//
// ---- Bounded work, honest reporting ---------------------------------------------
// Work is capped by an event budget. Two fallbacks are used, both PER BODY and both
// reported, never silent:
//   * motion clamp - a bullet that is still in contact with something while still
//     moving does not get its remaining motion advanced this step (`clamped`). Its
//     velocity is untouched; the discrete solver resolves the contact next step.
//   * positional safety net - any bullet left overlapping is separated
//     positionally (`pushedOut`) with velocities untouched.
// Unrelated bodies always integrate normally: there is no global freeze.
// ============================================================================
#include "Collision.hpp"
#include "Integrator.hpp"
#include <stdexcept>
#include <type_traits>

namespace phys {

// Contact tolerance in world units. Below this the pair counts as touching. It is
// far smaller than any shape in the test suite but large enough that the
// advancement loop terminates instead of grinding toward 1e-17.
inline constexpr real kContactTolerance = 1e-7;

// Speed below which a bullet is considered stopped for clamping purposes.
inline constexpr real kBulletRestSpeed = 0.5;

// Farthest distance from the centre of mass to any material point: the bounding
// circle radius, used for the angular part of the speed bound.
inline real sweepRadius(const RigidBody& b) {
    if (b.shape.type == ShapeType::Circle) return b.shape.radius;
    real radius = 0;
    for (const auto& v : b.shape.verts) radius = std::max(radius, v.length());
    return radius;
}

struct Separation { real gap = -1e30; Vec2 normal{1, 0}; };

// A cast borrows geometry and samples only the pose; never copy owning Shape vectors.
struct SweepPose {
    Vec2 position;
    Mat22 rot;
};
static_assert(std::is_standard_layout_v<SweepPose>);
static_assert(sizeof(real) == 8 && alignof(real) == 8);
static_assert(sizeof(SweepPose) == 48 && alignof(SweepPose) == 8);
static_assert(offsetof(SweepPose, position) == 0 && offsetof(SweepPose, rot) == 16);

inline std::pair<real, real> projection(const RigidBody& b, const SweepPose& pose, Vec2 n) {
    if (b.shape.type == ShapeType::Circle) {
        const real p = dot(pose.position, n);
        return {p - b.shape.radius, p + b.shape.radius};
    }
    real lo = 1e30, hi = -1e30;
    for (std::size_t i = 0; i < b.shape.verts.size(); ++i) {
        const real p = dot(pose.position + pose.rot * b.shape.verts[i], n);
        lo = std::min(lo, p); hi = std::max(hi, p);
    }
    return {lo, hi};
}

inline std::pair<real, real> projection(const RigidBody& b, Vec2 n) {
    return projection(b, {b.position, b.rot}, n);
}

// Each unit-axis gap is <= true distance; the maximum remains a lower bound.
inline Separation separatingGap(const RigidBody& A, const RigidBody& B,
                                const SweepPose& poseA, const SweepPose& poseB) {
    Separation result;
    const auto test = [&](Vec2 axis) {
        if (axis.lengthSq() < kEps) return;
        axis = axis.normalized();
        const auto pa = projection(A, poseA, axis), pb = projection(B, poseB, axis);
        const real forward = pb.first - pa.second, backward = pa.first - pb.second;
        const real gap = std::max(forward, backward);
        if (gap > result.gap) { result.gap = gap; result.normal = forward >= backward ? axis : -axis; }
    };
    if (A.shape.type == ShapeType::Polygon)
        for (std::size_t i = 0; i < A.shape.verts.size(); ++i) test(poseA.rot * A.shape.normals[i]);
    if (B.shape.type == ShapeType::Polygon)
        for (std::size_t i = 0; i < B.shape.verts.size(); ++i) test(poseB.rot * B.shape.normals[i]);
    if (A.shape.type == ShapeType::Circle && B.shape.type == ShapeType::Circle) test(poseB.position - poseA.position);
    if (A.shape.type == ShapeType::Circle && B.shape.type == ShapeType::Polygon)
        for (std::size_t i = 0; i < B.shape.verts.size(); ++i) test(poseB.position + poseB.rot * B.shape.verts[i] - poseA.position);
    if (B.shape.type == ShapeType::Circle && A.shape.type == ShapeType::Polygon)
        for (std::size_t i = 0; i < A.shape.verts.size(); ++i) test(poseB.position - (poseA.position + poseA.rot * A.shape.verts[i]));
    if (result.gap == -1e30) result.gap = -(sweepRadius(A) + sweepRadius(B));
    return result;
}

inline Separation separatingGap(const RigidBody& A, const RigidBody& B) {
    return separatingGap(A, B, {A.position, A.rot}, {B.position, B.rot});
}

inline real closingSpeedBound(const RigidBody& A, const RigidBody& B) {
    return (B.velocity - A.velocity).length()
         + std::abs(A.angularVelocity) * sweepRadius(A)
         + std::abs(B.angularVelocity) * sweepRadius(B);
}

struct TOI {
    bool hit = false;              // contact reached inside the window
    bool converged = true;         // false: iteration budget exhausted
    bool initiallyTouching = false;// cannot be cast; discrete solver's job
    real time = 0;                 // hit time within the window
    Separation separation;
    int iterations = 0;
};

// First contact time of a pair moving at constant linear/angular velocity.
inline TOI timeOfImpact(const RigidBody& originalA, const RigidBody& originalB, real duration) {
    TOI result;
    const Separation start = separatingGap(originalA, originalB);
    if (start.gap <= kContactTolerance) {
        // Overlapping or touching at t = 0: conservative advancement cannot start
        // from a zero gap, and this is a resting/discrete contact, not a swept event.
        result.initiallyTouching = true;
        result.separation = start;
        return result;
    }
    const real speed = closingSpeedBound(originalA, originalB);
    if (speed < kEps) return result;   // no relative motion => no future contact

    real t = 0;
    for (int iteration = 0; iteration < 128; ++iteration) {
        result.iterations = iteration + 1;
        const SweepPose A{originalA.position + originalA.velocity * t,
                          Mat22::rotation(originalA.angle + originalA.angularVelocity * t)};
        const SweepPose B{originalB.position + originalB.velocity * t,
                          Mat22::rotation(originalB.angle + originalB.angularVelocity * t)};
        result.separation = separatingGap(originalA, originalB, A, B);
        if (result.separation.gap <= kContactTolerance) {   // contact: the event
            result.hit = true;
            result.time = t;
            return result;
        }
        const real advance = result.separation.gap / speed;
        if (t + advance > duration) return result;          // provably clear in window
        t += advance;
    }
    result.converged = false;
    return result;
}

struct CCDStats {
    int events = 0;      // impacts resolved by the sweep
    int casts = 0;       // conservative-advancement calls
    int skipped = 0;     // pairs already touching at t = 0
    int unconverged = 0; // casts that hit the iteration cap
    int pushedOut = 0;   // positional safety-net separations
    int clamped = 0;     // bullets whose remaining motion was withheld
    bool capped = false; // event budget exhausted
};

// Advance every awake body for dt, stopping at bullet impacts.
inline CCDStats advanceContinuous(std::vector<RigidBody>& bodies, real dt,
                                  std::vector<char>& clamped, int maxEvents = 64, bool clearForces = true) {
    CCDStats stats;
    real remaining = dt;
    clamped.assign(bodies.size(), 0);

    const auto advanceAll = [&](real time) {
        if (time <= 0) return;
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            auto& b = bodies[i];
            if (b.isStatic() || !b.awake || clamped[i]) continue;
            b.position += b.velocity * time;
            b.setAngle(b.angle + b.angularVelocity * time);
        }
    };

    // Apply a one-sided impact impulse at the centre of mass (r = 0). The normal
    // comes from the true narrowphase manifold when the pair already overlaps, so
    // the direction is exact even though the point is not; see the note above for
    // why an approximate lever arm is worse than no lever arm here.
    const auto resolveImpact = [&](RigidBody& A, RigidBody& B, const Separation& sep) {
        Vec2 n = sep.normal;
        ContactManifold m;
        if (collide(A, B, m) && m.count > 0) n = m.normal;
        const real kN = A.invMass + B.invMass;
        if (kN <= 0) return;
        const real vn = dot(B.velocity - A.velocity, n);
        const real restitution = std::max(A.restitution, B.restitution);
        const real target = vn < -1.0 ? -restitution * vn : 0;   // threshold kills jitter
        const real lambda = (target - vn) / kN;
        if (lambda <= 0) return;                                 // push only
        const Vec2 t{n.y, -n.x};
        const real mu = std::sqrt(A.friction * B.friction);
        const real lambdaT = clamp(-dot(B.velocity - A.velocity, t) / kN, -mu * lambda, mu * lambda);
        A.applyImpulse(-(n * lambda + t * lambdaT), {});
        B.applyImpulse(n * lambda + t * lambdaT, {});
    };

    while (remaining > 1e-12 && stats.events < maxEvents) {
        TOI earliest;
        earliest.time = remaining;
        std::size_t ia = 0, ib = 0;
        bool found = false;
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            for (std::size_t j = i + 1; j < bodies.size(); ++j) {
                const auto& A = bodies[i];
                const auto& B = bodies[j];
                if (!(A.bullet || B.bullet) || (A.isStatic() && B.isStatic())) continue;
                // Bounding-circle travel envelope rejects distant pairs cheaply.
                const real reach = sweepRadius(A) + sweepRadius(B)
                                 + (B.velocity - A.velocity).length() * remaining;
                if ((B.position - A.position).lengthSq() > reach * reach) continue;
                ++stats.casts;
                const TOI impact = timeOfImpact(A, B, remaining);
                if (impact.initiallyTouching) { ++stats.skipped; continue; }
                if (!impact.converged) { ++stats.unconverged; continue; }
                if (impact.hit && impact.time <= earliest.time) {
                    earliest = impact; ia = i; ib = j; found = true;
                }
            }
        }
        if (!found) break;                       // nothing ahead: integrate the rest

        if (earliest.time > 0) advanceAll(earliest.time);
        remaining -= earliest.time;
        resolveImpact(bodies[ia], bodies[ib], earliest.separation);
        ++stats.events;
        if (earliest.time <= 1e-9) {             // zero-length event: force progress
            const real nudge = std::min(remaining, dt * 1e-3);
            advanceAll(nudge);
            remaining -= nudge;
        }
    }
    if (stats.events >= maxEvents) stats.capped = true;

    // ---- per-bullet motion clamp -------------------------------------------------
    // A bullet that is still touching something while still moving would otherwise
    // be integrated straight through the obstacle. Withhold only that body's
    // remaining motion; its velocity is preserved and the discrete solver resolves
    // the contact next step. Related bodies keep moving (no world freeze).
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        auto& A = bodies[i];
        if (!A.bullet || A.isStatic()) continue;
        if (A.velocity.length() < kBulletRestSpeed) continue;
        bool inContact = false;
        for (std::size_t j = 0; j < bodies.size() && !inContact; ++j) {
            if (i == j) continue;
            ContactManifold m;
            if (collide(A, bodies[j], m) && m.count > 0) inContact = true;
            else if (separatingGap(A, bodies[j]).gap <= kContactTolerance) inContact = true;
        }
        if (inContact) { clamped[i] = 1; ++stats.clamped; }
    }

    advanceAll(remaining);

    // ---- positional safety net ---------------------------------------------------
    // Any bullet still overlapping something is separated positionally: velocities
    // are untouched and unrelated bodies already integrated normally.
    for (int pass = 0; pass < 2; ++pass) {
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            if (!bodies[i].bullet || bodies[i].isStatic()) continue;
            for (std::size_t j = 0; j < bodies.size(); ++j) {
                if (i == j) continue;
                RigidBody& A = bodies[i];
                RigidBody& B = bodies[j];
                if (A.isStatic() && B.isStatic()) continue;
                ContactManifold m;
                if (!collide(A, B, m)) continue;
                const real depth = std::max(m.penetration - 0.005, real(0));
                if (depth <= 0) continue;
                const real k = A.invMass + B.invMass;
                if (k <= 0) continue;
                const real push = std::min(depth, real(0.2)) / k;
                A.position -= m.normal * (A.invMass * push);
                B.position += m.normal * (B.invMass * push);
                ++stats.pushedOut;
            }
        }
    }

    if (clearForces) for (auto& b : bodies) { b.force = {}; b.torque = 0; }
    return stats;
}

// Standalone convenience retains the existing interface; World supplies its scratch.
inline CCDStats advanceContinuous(std::vector<RigidBody>& bodies, real dt, int maxEvents = 64) {
    std::vector<char> scratch;
    return advanceContinuous(bodies, dt, scratch, maxEvents);
}

} // namespace phys
