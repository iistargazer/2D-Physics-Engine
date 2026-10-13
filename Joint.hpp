#pragma once
#include "RigidBody.hpp"
#include <stdexcept>

namespace phys {
enum class JointType { Distance, Revolute, Mouse, Weld, Prismatic };

struct Joint {
    JointType type = JointType::Distance;
    std::size_t a = 0, b = 0;
    Vec2 localA, localB, target, localAxis{1, 0};
    real length = 1, frequency = 0, dampingRatio = 0.7;
    real referenceAngle = 0;
    bool limits = false, motor = false, collideConnected = false;
    // Prismatic joints normally lock relative rotation (a slider block). Set this
    // false for an axle mount, where the wheel must stay free to spin.
    bool lockAngle = true;
    real lower = 0, upper = 0, motorSpeed = 0, maxMotorTorque = 0, maxForce = 1000;
    Vec2 pointImpulse;
    real impulse = 0, angularImpulse = 0, motorImpulse = 0, lowerImpulse = 0, upperImpulse = 0;

    static Joint distance(std::size_t a, std::size_t b, Vec2 localA, Vec2 localB,
                          real length, real frequency = 0, real damping = 0.7) {
        if (length < 0 || frequency < 0 || damping < 0) throw std::invalid_argument("invalid distance joint");
        Joint j; j.a = a; j.b = b; j.localA = localA; j.localB = localB;
        j.length = length; j.frequency = frequency; j.dampingRatio = damping; return j;
    }
    static Joint revolute(std::size_t a, std::size_t b, Vec2 localA, Vec2 localB, real reference = 0) {
        Joint j; j.type = JointType::Revolute; j.a = a; j.b = b;
        j.localA = localA; j.localB = localB; j.referenceAngle = reference; return j;
    }
    static Joint mouse(std::size_t body, Vec2 anchor, Vec2 target) {
        Joint j; j.type = JointType::Mouse; j.a = j.b = body;
        j.localB = anchor; j.target = target; j.frequency = 5; return j;
    }
    static Joint weld(std::size_t a, std::size_t b, Vec2 localA, Vec2 localB, real reference = 0) {
        Joint j = revolute(a, b, localA, localB, reference); j.type = JointType::Weld; return j;
    }
    static Joint prismatic(std::size_t a, std::size_t b, Vec2 localA, Vec2 localB,
                           Vec2 axis, real reference = 0) {
        if (axis.lengthSq() < kEps) throw std::invalid_argument("zero prismatic axis");
        Joint j = revolute(a, b, localA, localB, reference);
        j.type = JointType::Prismatic; j.localAxis = axis.normalized(); return j;
    }

    void prepare(std::vector<RigidBody>& bodies, real dt, real ratio) {
        if (a >= bodies.size() || b >= bodies.size()) throw std::out_of_range("joint body index");
        auto& A = bodies[a]; auto& B = bodies[b];
        if (limits && lower > upper) throw std::invalid_argument("reversed joint limits");
        dt_ = dt; rA_ = A.rot * localA; rB_ = B.rot * localB;
        const Vec2 d = B.position + rB_ - A.position - rA_;
        pointImpulse *= ratio; impulse *= ratio; angularImpulse *= ratio;
        motorImpulse *= ratio; lowerImpulse *= ratio; upperImpulse *= ratio;
        gamma_ = 0; bias_ = {}; scalarBias_ = 0;
        angularMass_ = A.invInertia + B.invInertia;
        angularMass_ = angularMass_ > 0 ? 1 / angularMass_ : 0;
        if (type == JointType::Mouse) {
            rA_ = {};
            const real omega = 2 * 3.14159265358979323846 * frequency;
            const real k = B.mass * omega * omega, c = 2 * B.mass * dampingRatio * omega;
            gamma_ = dt * (c + dt * k); gamma_ = gamma_ > 0 ? 1 / gamma_ : 0;
            bias_ = (B.position + rB_ - target) * (dt * k * gamma_);
            K_ = {B.invMass + B.invInertia * rB_.y * rB_.y + gamma_, -B.invInertia * rB_.x * rB_.y,
                  -B.invInertia * rB_.x * rB_.y, B.invMass + B.invInertia * rB_.x * rB_.x + gamma_};
            pointImpulse = limited(pointImpulse, dt * maxForce);
            B.applyImpulse(pointImpulse, rB_); return;
        }
        if (type == JointType::Distance) {
            const real distance = d.length();
            axis_ = distance > kEps ? d / distance : Vec2{1, 0};
            const real ra = cross(rA_, axis_), rb = cross(rB_, axis_);
            const real inverse = A.invMass + B.invMass + A.invInertia * ra * ra + B.invInertia * rb * rb;
            if (frequency > 0 && inverse > 0) {
                // m_eff=1/K; spring k=m_eff*w^2, damper c=2*zeta*m_eff*w.
                // Implicit spring integration gives gamma and beta (1/s).
                const real m = 1 / inverse, w = 2 * 3.14159265358979323846 * frequency;
                const real k = m * w * w, c = 2 * dampingRatio * m * w;
                gamma_ = 1 / (dt * (c + dt * k));
                scalarBias_ = (distance - length) * dt * k * gamma_;
            } else scalarBias_ = 0.2 / dt * (distance - length);
            scalarMass_ = inverse + gamma_ > 0 ? 1 / (inverse + gamma_) : 0;
            apply(A, B, axis_ * impulse); return;
        }
        // For point anchors, delta(vB-vA) = K*P:
        // Kxx=m^-1+IA^-1*rAy^2+IB^-1*rBy^2,
        // Kxy=-IA^-1*rAx*rAy-IB^-1*rBx*rBy,
        // Kyy=m^-1+IA^-1*rAx^2+IB^-1*rBx^2.
        const real mass = A.invMass + B.invMass;
        K_ = {mass + A.invInertia*rA_.y*rA_.y + B.invInertia*rB_.y*rB_.y,
              -A.invInertia*rA_.x*rA_.y - B.invInertia*rB_.x*rB_.y,
              -A.invInertia*rA_.x*rA_.y - B.invInertia*rB_.x*rB_.y,
              mass + A.invInertia*rA_.x*rA_.x + B.invInertia*rB_.x*rB_.x};
        bias_ = d * (0.2 / dt);
        if (type == JointType::Prismatic) {
            axis_ = A.rot * localAxis; perpendicular_ = axis_.perp();
            // C=d.n(A). Differentiating the rotating normal adds the d term:
            // JwA=-(rA+d) x n, JwB=rB x n.
            angularA_ = cross(rA_ + d, perpendicular_);
            angularB_ = cross(rB_, perpendicular_);
            const real k = mass + A.invInertia*angularA_*angularA_ + B.invInertia*angularB_*angularB_;
            scalarMass_ = k > 0 ? 1/k : 0;
            scalarBias_ = dot(d, perpendicular_) * 0.2/dt;
            applyPrismatic(A, B, impulse);
        } else apply(A, B, pointImpulse);
        if (!motor) motorImpulse = 0;
        else motorImpulse = clamp(motorImpulse, -dt * maxMotorTorque, dt * maxMotorTorque);
        if (!limits) lowerImpulse = upperImpulse = 0;
        if (!lockAngle) angularImpulse = motorImpulse = lowerImpulse = upperImpulse = 0;
        const real angular = angularImpulse + motorImpulse + lowerImpulse - upperImpulse;
        A.applyAngularImpulse(-angular); B.applyAngularImpulse(angular);
    }
    void solve(std::vector<RigidBody>& bodies) {
        auto& A = bodies[a]; auto& B = bodies[b];
        if (type == JointType::Mouse) {
            Vec2 delta = safeSolve(K_, -(B.velocityAt(rB_) + bias_ + pointImpulse * gamma_));
            const Vec2 old = pointImpulse;
            pointImpulse = limited(old + delta, dt_ * maxForce);
            B.applyImpulse(pointImpulse - old, rB_); return;
        }
        if (type == JointType::Distance) {
            const real delta = -scalarMass_ * (dot(B.velocityAt(rB_)-A.velocityAt(rA_), axis_)
                                               + scalarBias_ + gamma_ * impulse);
            impulse += delta; apply(A, B, axis_ * delta); return;
        }
        if (motor && type == JointType::Revolute) {
            const real old = motorImpulse;
            motorImpulse = clamp(old - angularMass_*(B.angularVelocity-A.angularVelocity-motorSpeed),
                                 -dt_*maxMotorTorque, dt_*maxMotorTorque);
            applyAngular(A, B, motorImpulse-old);
        }
        if (limits && type == JointType::Revolute) {
            const real angle = B.angle - A.angle - referenceAngle;
            // Speculative unilateral limits: for a positive gap use gap/dt to
            // prevent crossing; for violation use beta*gap/dt to restore it.
            const real lowGap = angle - lower;
            const real lowBias = (lowGap < 0 ? 0.2 : 1) * lowGap/dt_;
            real old = lowerImpulse;
            lowerImpulse = std::max(real(0), old-angularMass_*(B.angularVelocity-A.angularVelocity+lowBias));
            applyAngular(A, B, lowerImpulse-old);
            const real highGap = upper-angle;
            const real highBias = (highGap < 0 ? 0.2 : 1) * highGap/dt_;
            old = upperImpulse;
            upperImpulse = std::max(real(0), old-angularMass_*(A.angularVelocity-B.angularVelocity+highBias));
            applyAngular(A, B, -(upperImpulse-old));
        }
        if (lockAngle && (type == JointType::Weld || type == JointType::Prismatic)) {
            const real delta = -angularMass_*(B.angularVelocity-A.angularVelocity
                + 0.2/dt_*(B.angle-A.angle-referenceAngle));
            angularImpulse += delta; applyAngular(A, B, delta);
        }
        if (type == JointType::Prismatic) {
            const real cdot = dot(B.velocity-A.velocity, perpendicular_)
                + angularB_*B.angularVelocity-angularA_*A.angularVelocity;
            const real delta = -scalarMass_*(cdot+scalarBias_);
            impulse += delta; applyPrismatic(A, B, delta);
        } else {
            const Vec2 delta = safeSolve(K_, -(B.velocityAt(rB_)-A.velocityAt(rA_)+bias_));
            pointImpulse += delta; apply(A, B, delta);
        }
    }
    void correctPosition(std::vector<RigidBody>& bodies) const {
        if (type == JointType::Mouse || (type == JointType::Distance && frequency > 0)) return;
        auto& A=bodies[a];auto& B=bodies[b];
        const Vec2 ra=A.rot*localA,rb=B.rot*localB,d=B.position+rb-A.position-ra;
        const auto applyPosition=[&](Vec2 p) {
            if(!A.isStatic()){A.translate(-p*A.invMass);A.setAngle(A.angle-A.invInertia*cross(ra,p));}
            if(!B.isStatic()){B.translate(p*B.invMass);B.setAngle(B.angle+B.invInertia*cross(rb,p));}
        };
        if(type==JointType::Distance) {
            const real n=d.length();const Vec2 u=n>kEps?d/n:Vec2{1,0};
            const real k=A.invMass+B.invMass+A.invInertia*std::pow(cross(ra,u),2)
                +B.invInertia*std::pow(cross(rb,u),2);
            if(k>0)applyPosition(u*(-0.4*clamp(n-length,-0.2,0.2)/k));
        } else if(type==JointType::Prismatic) {
            // Remove only the offset perpendicular to the slide axis; motion along
            // the axis is the joint's whole purpose.
            const Vec2 axis=A.rot*localAxis;const Vec2 perpendicular=axis.perp();
            const real offset=dot(d,perpendicular);
            const real wA=cross(ra+d,perpendicular),wB=cross(rb,perpendicular);
            const real k=A.invMass+B.invMass+A.invInertia*wA*wA+B.invInertia*wB*wB;
            if(k>0)applyPosition(perpendicular*(-0.4*clamp(offset,-0.2,0.2)/k));
        } else {
            const real m=A.invMass+B.invMass;
            const Mat22 k{m+A.invInertia*ra.y*ra.y+B.invInertia*rb.y*rb.y,
                -A.invInertia*ra.x*ra.y-B.invInertia*rb.x*rb.y,
                -A.invInertia*ra.x*ra.y-B.invInertia*rb.x*rb.y,
                m+A.invInertia*ra.x*ra.x+B.invInertia*rb.x*rb.x};
            applyPosition(safeSolve(k,-d*0.4));
        }
        if(lockAngle && (type==JointType::Weld || type==JointType::Prismatic
                         || (type==JointType::Revolute && limits))) {
            const real angle=B.angle-A.angle-referenceAngle;
            const real error=type==JointType::Revolute?angle-clamp(angle,lower,upper):angle;
            const real k=A.invInertia+B.invInertia;
            if(k>0){const real p=-0.4*clamp(error,-0.2,0.2)/k;
                if(!A.isStatic())A.setAngle(A.angle-A.invInertia*p);
                if(!B.isStatic())B.setAngle(B.angle+B.invInertia*p);}
        }
    }
    void resetImpulses() {
        pointImpulse = {}; impulse = angularImpulse = motorImpulse = lowerImpulse = upperImpulse = 0;
    }
private:
    static Vec2 safeSolve(const Mat22& k, Vec2 rhs) { return std::abs(k.determinant()) > kEps ? k.solve(rhs) : Vec2{}; }
    static Vec2 limited(Vec2 p, real limit) { const real n=p.length(); return n>limit && n>0 ? p*(limit/n) : p; }
    // Every joint row write goes through the guarded RigidBody helpers, so an
    // immovable body is never touched (see docs/PARALLEL.md).
    void apply(RigidBody& A, RigidBody& B, Vec2 p) const { A.applyImpulse(-p,rA_); B.applyImpulse(p,rB_); }
    static void applyAngular(RigidBody& A, RigidBody& B, real p) { A.applyAngularImpulse(-p); B.applyAngularImpulse(p); }
    void applyPrismatic(RigidBody& A, RigidBody& B, real p) const {
        if (!A.isStatic()) {
            A.velocity -= perpendicular_*(A.invMass*p);
            A.angularVelocity -= A.invInertia*angularA_*p;
        }
        if (!B.isStatic()) {
            B.velocity += perpendicular_*(B.invMass*p);
            B.angularVelocity += B.invInertia*angularB_*p;
        }
    }
    Vec2 rA_, rB_, axis_, perpendicular_, bias_;
    Mat22 K_;
    real gamma_=0, scalarBias_=0, scalarMass_=0, angularMass_=0, dt_=0;
    real angularA_=0, angularB_=0;
};
} // namespace phys
