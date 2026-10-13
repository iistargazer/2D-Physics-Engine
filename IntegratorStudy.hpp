#pragma once
#include "Vec2.hpp"

namespace phys {
struct ParticleState { Vec2 position, velocity; };
struct SymplecticPolicy {
    template<class Acceleration>
    static void step(ParticleState& s, real t, real dt, Acceleration acceleration) {
        s.velocity += acceleration(s.position, t) * dt;
        s.position += s.velocity * dt;
    }
};
struct VerletPolicy {
    // x1=x0+v0*h+(h^2/2)*a0; v1=v0+(h/2)*(a0+a1).
    // Two force evaluations; second-order, time-reversible for position-only
    // forces. This interface deliberately excludes velocity-dependent forces.
    template<class Acceleration>
    static void step(ParticleState& s, real t, real dt, Acceleration acceleration) {
        const Vec2 a0=acceleration(s.position,t);
        s.position += s.velocity*dt + a0*(0.5*dt*dt);
        s.velocity += (a0+acceleration(s.position,t+dt))*(0.5*dt);
    }
};
struct RK4Policy {
    // First-order system y'=(v,a(x,t)). Evaluate slopes at start, two midpoint
    // predictions and endpoint, then combine (k1+2k2+2k3+k4)/6. Fourth-order
    // local accuracy is NOT symplecticity: long-time energy can still drift.
    template<class Acceleration>
    static void step(ParticleState& s, real t, real dt, Acceleration acceleration) {
        const Vec2 x1=s.velocity,v1=acceleration(s.position,t);
        const Vec2 x2=s.velocity+v1*(dt/2),v2=acceleration(s.position+x1*(dt/2),t+dt/2);
        const Vec2 x3=s.velocity+v2*(dt/2),v3=acceleration(s.position+x2*(dt/2),t+dt/2);
        const Vec2 x4=s.velocity+v3*dt,v4=acceleration(s.position+x3*dt,t+dt);
        s.position += (x1+2*x2+2*x3+x4)*(dt/6);
        s.velocity += (v1+2*v2+2*v3+v4)*(dt/6);
    }
};
} // namespace phys
