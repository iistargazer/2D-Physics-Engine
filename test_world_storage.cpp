#include "Showcase.hpp"
#include <cstdio>

extern std::size_t testAllocations;
extern bool countTestAllocations;
using namespace phys;

static bool same(const World& a, const World& b) {
    if (a.bodies.size() != b.bodies.size() || a.joints.size() != b.joints.size()) return false;
    for (std::size_t i = 0; i < a.joints.size(); ++i) {
        const auto& x = a.joints[i]; const auto& y = b.joints[i];
        if (x.pointImpulse.x != y.pointImpulse.x || x.pointImpulse.y != y.pointImpulse.y
            || x.impulse != y.impulse || x.angularImpulse != y.angularImpulse
            || x.motorImpulse != y.motorImpulse || x.lowerImpulse != y.lowerImpulse
            || x.upperImpulse != y.upperImpulse) return false;
    }
    for (std::size_t i = 0; i < a.bodies.size(); ++i) {
        const auto& x = a.bodies[i]; const auto& y = b.bodies[i];
        if (x.position.x != y.position.x || x.position.y != y.position.y
            || x.velocity.x != y.velocity.x || x.velocity.y != y.velocity.y
            || x.angle != y.angle || x.angularVelocity != y.angularVelocity
            || x.awake != y.awake || x.sleepTime != y.sleepTime
            || x.force.x != y.force.x || x.force.y != y.force.y || x.torque != y.torque)
            return false;
    }
    return true;
}

static bool measuredStep(World& w, World::StepStatus expected) {
    testAllocations = 0; countTestAllocations = true;
    const auto status = w.tryStep(1.0 / 60);
    countTestAllocations = false;
    if (status != expected || testAllocations != 0) {
        std::fprintf(stderr, "status=%d expected=%d allocations=%zu\n",
                     int(status), int(expected), testAllocations);
        return false;
    }
    return true;
}

static bool trajectory(BroadphaseMode mode, bool sleeping) {
    auto a = makeStack(8);
    a.broadphaseMode = mode; a.sleeping = sleeping;
    a.joints.push_back(Joint::distance(0, 8, {}, {}, 8, 2));
    World b; b.gravity = a.gravity; b.bodies = a.bodies; b.joints = a.joints;
    b.solver.config() = a.solver.config(); b.broadphaseMode = mode; b.sleeping = sleeping;
    a.prepare({16, 4, 120, 120, 64});
    std::size_t contactFrames = 0;
    for (int f = 0; f < 720; ++f) {
        // Persistent small drive keeps contacts loaded and the island awake.
        a.bodies[8].applyForce({0.1, 0}, a.bodies[8].position);
        b.bodies[8].applyForce({0.1, 0}, b.bodies[8].position);
        if (!measuredStep(a, World::StepStatus::Complete)) return false;
        b.step(1.0 / 60);
        if (!same(a, b)) return false;
        contactFrames += !a.manifolds().empty();
    }
    if (contactFrames < 700 || !a.bodies[8].awake) return false;
    // Explicit cache reset and count shrink/rebuild reuse the prepared buffers.
    a.clearContactCache(); b.clearContactCache();
    a.joints.clear(); b.joints.clear();
    a.bodies.pop_back(); b.bodies.pop_back();
    if (!measuredStep(a, World::StepStatus::Complete)) return false;
    b.step(1.0 / 60);
    return same(a, b);
}

// The temporal path reserves one TemporalContact row per contact slot. The same
// prepared-vs-dynamic contract must hold: identical trajectories, zero allocations,
// and an observable allocation-free Contacts failure that leaves state untouched.
static bool temporalTrajectory(bool sleeping) {
    auto a = makeStack(8);
    a.solver.config().mode = SolverMode::TGS;
    a.solver.config().substeps = 4;
    a.solver.config().compliance = 0.0005;
    a.solver.config().damping = 20;
    a.sleeping = sleeping;
    a.joints.push_back(Joint::distance(0, 8, {}, {}, 8, 2));
    World b; b.gravity = a.gravity; b.bodies = a.bodies; b.joints = a.joints;
    b.solver.config() = a.solver.config(); b.sleeping = sleeping;
    a.prepare({16, 4, 120, 120, 64});
    std::size_t contactFrames = 0;
    for (int f = 0; f < 300; ++f) {
        a.bodies[8].applyForce({0.1, 0}, a.bodies[8].position);
        b.bodies[8].applyForce({0.1, 0}, b.bodies[8].position);
        if (!measuredStep(a, World::StepStatus::Complete)) return false;
        b.step(1.0 / 60);
        if (!same(a, b)) return false;
        contactFrames += !a.manifolds().empty();
    }
    if (contactFrames < 290) return false;
    // Rows share the contact capacity, so Contacts exhaustion is still the signal.
    a.prepare({16, 4, 120, 0, 64});
    if (!measuredStep(a, World::StepStatus::Contacts)) return false;
    a.prepare({16, 4, 120, 120, 64});
    if (!measuredStep(a, World::StepStatus::Complete)) return false;
    b.step(1.0 / 60);
    if (!same(a, b)) return false;
    // Switching back to PGS invalidates the temporal warm estimate instead of reusing it.
    a.solver.config().mode = SolverMode::PGS; b.solver.config().mode = SolverMode::PGS;
    if (!measuredStep(a, World::StepStatus::Complete)) return false;
    b.step(1.0 / 60);
    return same(a, b);
}

static bool exhaustion() {
    for (const auto expected : {World::StepStatus::Bodies, World::StepStatus::Joints,
         World::StepStatus::Vertices, World::StepStatus::Pairs, World::StepStatus::Contacts}) {
        auto a = makeStack(2);
        a.joints.push_back(Joint::distance(0, 2, {}, {}, 2, 2));
        a.bodies[2].force = {1, 2}; a.bodies[2].torque = 3;
        World original; original.bodies = a.bodies; original.joints = a.joints;
        World::Capacity capacity{3, 1, 3, 3, 12};
        if (expected == World::StepStatus::Bodies) capacity.bodies = 2;
        if (expected == World::StepStatus::Joints) capacity.joints = 0;
        if (expected == World::StepStatus::Vertices) capacity.vertices = 11;
        if (expected == World::StepStatus::Pairs) capacity.pairs = 0;
        // Start penetrating to ensure contact capacity (not candidate capacity) fails.
        a.bodies[1].position.y -= 0.05; original.bodies = a.bodies;
        if (expected == World::StepStatus::Contacts) capacity.contacts = 0;
        a.prepare(capacity);
        if (!measuredStep(a, expected) || !same(a, original) || !a.manifolds().empty()) return false;
        a.prepare({3, 1, 3, 3, 12});
        if (!measuredStep(a, World::StepStatus::Complete)) return false;
        original.step(1.0 / 60);
        if (!same(a, original)) return false;
    }
    return true;
}

int main() {
    for (auto mode : {BroadphaseMode::DynamicTree, BroadphaseMode::AllPairs})
        for (bool sleep : {false, true})
            if (!trajectory(mode, sleep)) return 1;
    if (!exhaustion()) return 1;
    if (!temporalTrajectory(false) || !temporalTrajectory(true)) return 1;
    // Exhaustion after established history must retain the previous public contacts
    // and impulses, including failed retry with pending forces.
    auto cached = makeStack(2);
    cached.prepare({4, 1, 6, 6, 16});
    for (int f = 0; f < 60; ++f)
        if (!measuredStep(cached, World::StepStatus::Complete)) return 1;
    World reference = cached;
    const auto contacts = cached.manifolds().size();
    const real impulse = cached.manifolds().front().contactPoints[0].accN;
    cached.prepare({4, 1, 6, 0, 16});
    cached.bodies[2].applyForce({0.1, 0}, cached.bodies[2].position);
    reference.bodies[2].applyForce({0.1, 0}, reference.bodies[2].position);
    if (!measuredStep(cached, World::StepStatus::Contacts) || !same(cached, reference)
        || cached.manifolds().size() != contacts
        || cached.manifolds().front().contactPoints[0].accN != impulse) return 1;
    cached.prepare({4, 1, 6, 6, 16});
    if (!measuredStep(cached, World::StepStatus::Complete)) return 1;
    reference.step(1.0 / 60);
    if (!same(cached, reference)) return 1;
    // Replacing a public shape within total vertex capacity must invalidate without
    // nested allocations; growing body count must reuse reserved tree/snapshot space.
    cached.bodies[2] = RigidBody::makeBox(0.9, 0.8, 1, {0, 2});
    reference.bodies[2] = cached.bodies[2];
    cached.add(RigidBody::makeCircle(0.2, 1, {4, 4}));
    reference.add(cached.bodies.back());
    if (!measuredStep(cached, World::StepStatus::Complete)) return 1;
    reference.step(1.0 / 60);
    if (!same(cached, reference)) return 1;
    auto bullet = makeShowcase(8);
    bullet.prepare({2, 0, 1, 1, 8});
    for (int f = 0; f < 120; ++f)
        if (!measuredStep(bullet, World::StepStatus::Complete)) return 1;
    if (bullet.bodies[1].position.x >= 0 || bullet.ccdStats.capped) return 1;
    auto sleeper = makeStack(3); sleeper.sleeping = true;
    sleeper.prepare({4, 0, 6, 6, 16});
    for (int f = 0; f < 600; ++f)
        if (!measuredStep(sleeper, World::StepStatus::Complete)) return 1;
    if (sleeper.bodies[3].awake) return 1;
    sleeper.bodies[3].applyForce({1, 0}, sleeper.bodies[3].position);
    if (!measuredStep(sleeper, World::StepStatus::Complete) || !sleeper.bodies[1].awake) return 1;
    // Exercise real multi-joint owners, including NGS, limits, motors and mouse.
    for (int scene : {3, 4, 5, 6}) {
        auto rig = makeShowcase(scene);
        rig.prepare({128, 256, 8128, 8128, 512});
        for (int f = 0; f < 120; ++f)
            if (!measuredStep(rig, World::StepStatus::Complete)) return 1;
    }
    sleeper.joints.push_back(Joint::mouse(3, {}, {1, 4}));
    sleeper.prepare({4, 1, 6, 6, 16});
    for (int f = 0; f < 120; ++f)
        if (!measuredStep(sleeper, World::StepStatus::Complete)) return 1;
    if (sizeof(std::size_t) == 8) {
        static_assert(std::is_standard_layout_v<World::CachedManifold>);
        static_assert(std::is_standard_layout_v<World::Snapshot>);
        if (sizeof(World::Capacity) != 40 || sizeof(World::CachedManifold) != 272
            || sizeof(World::Snapshot) != 96
            || offsetof(World::CachedManifold, normal) != 208
            || offsetof(World::CachedManifold, count) != 264
            || offsetof(World::Snapshot, vertexOffset) != 72
            || offsetof(World::Snapshot, type) != 88) return 1;
    }
    std::puts("PASS: prepared first/repeated active-contact + joint steps, tree/all-pairs, "
              "sleep/wake, cache reset, shrink, CCD and all five capacity failures allocate zero; "
              "prepared PGS and TGS trajectories equal their dynamic references");
    std::printf("LAYOUT Capacity %zu/%zu CachedManifold %zu/%zu Snapshot %zu/%zu "
                "ContactPoint %zu/%zu ContactManifold %zu/%zu TreeNode %zu/%zu\n",
                sizeof(World::Capacity), alignof(World::Capacity),
                sizeof(World::CachedManifold), alignof(World::CachedManifold),
                sizeof(World::Snapshot), alignof(World::Snapshot),
                sizeof(ContactPoint), alignof(ContactPoint),
                sizeof(ContactManifold), alignof(ContactManifold),
                sizeof(DynamicTree::Node), alignof(DynamicTree::Node));
    return 0;
}
