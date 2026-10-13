#pragma once
// ============================================================================
// Semi-implicit (symplectic) Euler.
//
//     v_{n+1} = v_n + a(x_n, v_n) dt        <- velocity FIRST
//     x_{n+1} = x_n + v_{n+1} dt            <- position uses the NEW velocity
//
// Explicit Euler (x uses v_n) adds energy every step -> oscillators spiral
// outward. Swapping the order makes the map symplectic: it preserves phase-space
// area (det of the Jacobian = 1) and conserves a "shadow Hamiltonian" H~ = H + O(dt),
// so energy error stays BOUNDED instead of drifting. Same cost, first order.
//
// We split the step in two because the sequential-impulse solver must run
// BETWEEN them (it works on velocities, then positions advance with the
// corrected velocities):
//
//     integrateForces()  ->  solver.prepare()/solve()  ->  integrateVelocities()
// ============================================================================
#include "RigidBody.hpp"

namespace phys {

// Step 1: apply forces/gravity to velocities.
inline void integrateForces(RigidBody& b, Vec2 gravity, real dt) {
    if (b.isStatic()) return;
    // Gravity is an acceleration, so it must NOT be scaled by invMass.
    b.velocity += (b.force * b.invMass + gravity) * dt;
    b.angularVelocity += (b.torque + b.driveTorque) * b.invInertia * dt;

    // Implicit (unconditionally stable) damping: v /= (1 + c dt)
    b.velocity *= real(1) / (1 + dt * b.linearDamping);
    b.angularVelocity *= real(1) / (1 + dt * b.angularDamping);
}

// Step 2: advance pose with the (solver-corrected) velocities, clear accumulators.
inline void integrateVelocities(RigidBody& b, real dt) {
    if (!b.isStatic()) {
        b.position += b.velocity * dt;
        b.setAngle(b.angle + b.angularVelocity * dt);    // refreshes cached R
    }
    b.force = {};
    b.torque = 0;
}

// Convenience for standalone use (no collision solve in between).
inline void symplecticEuler(RigidBody& b, Vec2 gravity, real dt) {
    integrateForces(b, gravity, dt);
    integrateVelocities(b, dt);
}

} // namespace phys
