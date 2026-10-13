#pragma once
// ============================================================================
// RigidBody + Shape
//
// State is split into LINEAR (x, v, F, m) and ANGULAR (theta, w, tau, I).
// We store INVERSE mass / inertia: static bodies get 0, which makes every
// impulse formula "just work" with no special cases (a static body cannot be
// accelerated because 1/m = 0).
//
// All polygon vertices are stored in BODY-LOCAL space with the CENTROID at the
// origin and CCW winding. That makes "position" the centre of mass, so
// F = m a and tau = I alpha hold without parallel-axis corrections.
// ============================================================================
#include "Mat22.hpp"
#include <algorithm>
#include <vector>
#include <cstddef>

namespace phys {

enum class ShapeType { Circle, Polygon };

struct Shape {
    ShapeType type = ShapeType::Circle;
    real radius = 0;                 // Circle
    std::vector<Vec2> verts;         // Polygon, local, CCW, centroid at origin
    std::vector<Vec2> normals;       // Polygon, local, outward unit face normals
};

struct RigidBody {
    // ---- linear -----------------------------------------------------------
    Vec2 position;                   // centre of mass, world space
    Vec2 velocity;
    Vec2 force;                      // accumulated, cleared every step
    real mass = 0, invMass = 0;

    // ---- angular ----------------------------------------------------------
    real angle = 0;                  // radians, CCW
    real angularVelocity = 0;
    real torque = 0;
    // Persistent angular drive (a motor that does not need re-applying each step).
    // Unlike `torque`, this is NOT cleared by integrateVelocities.
    real driveTorque = 0;
    real inertia = 0, invInertia = 0;   // about the centre of mass

    // ---- material ---------------------------------------------------------
    real friction = 0.4;             // Coulomb mu
    real restitution = 0.2;          // e in [0,1]
    real linearDamping = 0, angularDamping = 0;

    bool bullet = false; // opt-in continuous cast against other bodies
    bool awake = true;
    real sleepTime = 0;
    void wake() { awake = true; sleepTime = 0; }

    Mat22 rot;                       // cached R(angle)
    Shape shape;

    // invMass == 0 is the single definition of an immovable body; `setStatic` and
    // both factories clear invInertia together with invMass, so one test covers
    // every write below. `isStatic` names the policy; the guards are what make it
    // hold. Integrator already skips immovable bodies - the solver paths now agree.
    bool isStatic() const { return invMass == 0; }

    void setAngle(real a) { angle = a; rot = Mat22::rotation(a); }

    Vec2 worldVertex(std::size_t i) const { return position + rot * shape.verts[i]; }
    Vec2 worldNormal(std::size_t i) const { return rot * shape.normals[i]; }

    // F at world point p => linear force F and torque (p-x) x F.
    void applyForce(Vec2 f, Vec2 worldPoint) {
        if (f.lengthSq() > 0) wake();
        force += f;
        torque += cross(worldPoint - position, f);
    }

    // Instantaneous impulse P at lever arm r (world-oriented, relative to COM):
    //   dv = P/m,   dw = (r x P)/I
    //
    // Skipping the write for an immovable body is load-bearing, not an
    // optimisation: it keeps inf/NaN impulses out of its state, and it is what
    // lets many rows sharing one floor belong to the same parallel colour
    // (see docs/PARALLEL.md). Writing a zero delta would still be a race.
    void applyImpulse(Vec2 p, Vec2 r) {
        if (isStatic()) return;
        velocity += p * invMass;
        angularVelocity += invInertia * cross(r, p);
    }

    // Angular-only row write (z impulse), same immovable guard.
    void applyAngularImpulse(real p) {
        if (isStatic()) return;
        angularVelocity += invInertia * p;
    }

    // Position-level (NGS) row write, same immovable guard.
    void translate(Vec2 d) {
        if (isStatic()) return;
        position += d;
    }

    // Velocity of the material point at lever arm r:  v + w x r
    Vec2 velocityAt(Vec2 r) const { return velocity + cross(angularVelocity, r); }

    void setStatic() { mass = inertia = invMass = invInertia = 0; velocity = {}; angularVelocity = 0; }

    // ---- factories --------------------------------------------------------
    // mass <= 0 => static body.
    static RigidBody makeCircle(real r, real m, Vec2 pos) {
        RigidBody b;
        b.shape.type = ShapeType::Circle;
        b.shape.radius = r;
        b.position = pos;
        b.setAngle(0);
        if (m > 0) {
            b.mass = m; b.invMass = 1 / m;
            b.inertia = real(0.5) * m * r * r;      // solid disc: (1/2) m r^2
            b.invInertia = 1 / b.inertia;
        }
        return b;
    }

    // Oriented bounding box = a 4-gon rotated by `angle`.
    static RigidBody makeBox(real w, real h, real m, Vec2 pos, real angle = 0) {
        const real hx = w / 2, hy = h / 2;
        RigidBody b = makePolygon({{-hx, -hy}, {hx, -hy}, {hx, hy}, {-hx, hy}}, m, pos);
        b.setAngle(angle);
        return b;
    }

    // Input must be a CONVEX polygon (either winding). Mass properties are
    // computed by a triangle fan from the origin:
    //   area     A  = 1/2 sum cross(p_i, p_i+1)
    //   centroid c  = (1/(6A)) sum cross(p_i,p_i+1) (p_i + p_i+1)
    //   inertia  I0 = (rho/12) sum cross(p_i,p_i+1)(p_i.p_i + p_i.p_i+1 + p_i+1.p_i+1)
    // then the parallel-axis theorem moves I from the origin to the centroid.
    static RigidBody makePolygon(std::vector<Vec2> pts, real m, Vec2 pos) {
        RigidBody b;
        b.shape.type = ShapeType::Polygon;
        const std::size_t n = pts.size();

        real area2 = 0;                                  // 2A (signed)
        for (std::size_t i = 0; i < n; ++i)
            area2 += cross(pts[i], pts[(i + 1) % n]);
        if (area2 < 0) {                                 // force CCW winding
            std::reverse(pts.begin(), pts.end());
            area2 = -area2;
        }
        const real area = area2 / 2;

        Vec2 c{};
        real ixx = 0;                                    // unit-density I about origin
        for (std::size_t i = 0; i < n; ++i) {
            const Vec2 p = pts[i], q = pts[(i + 1) % n];
            const real cr = cross(p, q);
            c += (p + q) * cr;
            ixx += cr * (dot(p, p) + dot(p, q) + dot(q, q));
        }
        c = c / (6 * area);
        ixx /= 12;
        const real iCentroidUnit = ixx - area * c.lengthSq();   // parallel axis

        for (auto& p : pts) p -= c;                      // recentre on centroid
        b.shape.verts = pts;
        b.shape.normals.resize(n);
        for (std::size_t i = 0; i < n; ++i) {
            const Vec2 e = pts[(i + 1) % n] - pts[i];
            b.shape.normals[i] = Vec2{e.y, -e.x}.normalized();  // outward for CCW
        }

        b.position = pos;
        b.setAngle(0);
        if (m > 0) {
            b.mass = m; b.invMass = 1 / m;
            b.inertia = (m / area) * iCentroidUnit;      // density rho = m / A
            b.invInertia = 1 / b.inertia;
        }
        return b;
    }
};

} // namespace phys
