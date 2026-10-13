#pragma once
#include "Joint.hpp"
#include "Manifold.hpp"
#include <numeric>

namespace phys {

// An island sleeps only when EVERY dynamic member has been slow for this long.
inline constexpr real kSleepTime = 0.5;
// Speed thresholds that count as "at rest" for sleeping.
inline constexpr real kSleepLinearSpeedSq = 0.0025;   // |v| < 0.05 m/s
inline constexpr real kSleepAngularSpeed = 0.05;      // |w| < 0.05 rad/s

// Union-find over the contact + joint graph.
//
// The island pass runs at the START of the step, using the previous step's
// manifolds, on purpose: the velocities it inspects are then the post-solve
// velocities of the last frame. Inspecting velocities after the force pass would
// see gravity's per-step increment (g*dt ~ 0.16 m/s) on every resting body and
// would wake every island forever - the bug that kept this scene from sleeping.
class Islands {
public:
    void reserve(std::size_t bodies) {
        parent_.reserve(bodies); flags_.reserve(bodies); time_.reserve(bodies);
    }
    void build(std::vector<RigidBody>& bodies, const std::vector<ContactManifold>& contacts,
               const std::vector<Joint>& joints) {
        parent_.resize(bodies.size());
        std::iota(parent_.begin(), parent_.end(), std::size_t(0));
        const auto join = [&](std::size_t a, std::size_t b) {
            if (!bodies[a].isStatic() && !bodies[b].isStatic()) {
                a = root(a); b = root(b);
                if (a != b) parent_[std::max(a, b)] = std::min(a, b);
            }
        };
        // Static bodies join islands together as a constraint but are never
        // members: a shared floor must not merge two independent stacks.
        for (const auto& m : contacts) join(m.indexA, m.indexB);
        for (const auto& j : joints) {
            if (j.a >= bodies.size() || j.b >= bodies.size())
                throw std::out_of_range("island joint index");
            join(j.a, j.b);
        }
        for (std::size_t i = 0; i < bodies.size(); ++i) parent_[i] = root(i);
    }

    // Wake whole islands whose members are explicitly awake, were acted on from
    // outside, or are still moving. `awake` is the authoritative flag: an asleep
    // body is only brought back by a real disturbance, never by gravity alone.
    void wakeConnected(std::vector<RigidBody>& bodies, const std::vector<Joint>& joints) {
        flags_.assign(bodies.size(), 0);
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            const auto& b = bodies[i];
            if (b.isStatic()) continue;
            const bool moving = b.velocity.lengthSq() >= kSleepLinearSpeedSq
                             || std::abs(b.angularVelocity) >= kSleepAngularSpeed;
            const bool disturbed = b.force.lengthSq() > 0 || b.torque != 0
                                || std::abs(b.driveTorque) > 0;
            // `!b.awake` bodies are asleep; they only wake through a disturbance.
            if (b.awake || moving || disturbed) flags_[parent_[i]] = true;
        }
        for (const auto& j : joints)
            if (j.motor || j.type == JointType::Mouse) flags_[parent_[j.b]] = true;
        for (std::size_t i = 0; i < bodies.size(); ++i)
            if (!bodies[i].isStatic() && flags_[parent_[i]] && !bodies[i].awake)
                bodies[i].wake();
    }

    // Aggregate island state and put quiet islands to sleep.
    void sleep(std::vector<RigidBody>& bodies, real dt) {
        if (parent_.size() != bodies.size()) return;   // no island pass yet
        flags_.assign(bodies.size(), 1);
        time_.assign(bodies.size(), 1e30);
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            auto& b = bodies[i];
            if (b.isStatic()) continue;
            const bool slow = b.velocity.lengthSq() < kSleepLinearSpeedSq
                           && std::abs(b.angularVelocity) < kSleepAngularSpeed;
            b.sleepTime = slow ? b.sleepTime + dt : 0;
            flags_[parent_[i]] = flags_[parent_[i]] && slow;
            time_[parent_[i]] = std::min(time_[parent_[i]], b.sleepTime);
        }
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            auto& b = bodies[i];
            if (b.isStatic()) continue;
            if (flags_[parent_[i]] && time_[parent_[i]] >= kSleepTime) {
                b.awake = false;
                b.velocity = {};
                b.angularVelocity = 0;
            }
        }
    }

    std::size_t island(std::size_t body) const { return parent_[body]; }
    std::size_t count() const { return parent_.size(); }

private:
    std::size_t root(std::size_t i) {
        while (parent_[i] != i) { parent_[i] = parent_[parent_[i]]; i = parent_[i]; }
        return i;
    }
    std::vector<std::size_t> parent_;
    std::vector<unsigned char> flags_;
    std::vector<real> time_;
};

} // namespace phys
