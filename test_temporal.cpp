#include "Showcase.hpp"
#include <cstdio>
#include <stdexcept>
using namespace phys;
extern std::size_t testAllocations;
extern bool countTestAllocations;
static int failures = 0;
#define REQUIRE(condition, message) do { if (!(condition)) { std::fprintf(stderr, "FAIL: %s\n", message); ++failures; } } while (false)

static void step(World& w, real dt) {
    testAllocations = 0; countTestAllocations = true;
    const auto status = w.tryStep(dt);
    countTestAllocations = false;
    REQUIRE(status == World::StepStatus::Complete && testAllocations == 0, "prepared temporal step/capacity allocates nothing");
    for (const auto& b : w.bodies)
        REQUIRE(std::isfinite(b.position.lengthSq()) && std::isfinite(b.velocity.lengthSq())
            && std::isfinite(b.angularVelocity), "finite temporal body state");
    for (const auto& m : w.manifolds()) for (int i = 0; i < m.count; ++i)
        REQUIRE(m.contactPoints[i].accN >= 0
            && std::abs(m.contactPoints[i].accT) <= m.friction * m.contactPoints[i].accN + 1e-9,
            "temporal unilateral impulse and current friction cone");
}
static void configure(World& w, int slices) {
    w.solver.config().mode = SolverMode::TGS;
    w.solver.config().substeps = slices;
    w.solver.config().iterations = 20;
    w.prepare({64, 64, 2016, 2016, 256});
}

static void stacks() {
    for (real dt : {1.0 / 30, 1.0 / 60, 1.0 / 120}) for (int slices : {1, 4, 16}) {
        auto w = makeStack(5); configure(w, slices);
        for (int f = 0; f < int(6 / dt); ++f) step(w, dt);
        std::printf("STACK dt=%.6f N=%d y=%.6f speed=%.6f\n", dt, slices, w.bodies.back().position.y, w.bodies.back().velocity.length());
        REQUIRE(std::abs(w.bodies.back().position.y - 4.5) < 0.12, "five-box resting height");
        REQUIRE(w.bodies.back().velocity.length() < 0.05, "five-box resting velocity");
        // Heavy body above light body: loaded 1000:1, continuously disturbed in x.
        World load;
        load.add(RigidBody::makeBox(20, 1, 0, {0, -0.5}));
        auto light = RigidBody::makeBox(2, 1, 1, {0, 0.49}); light.restitution = 0; light.friction = 1;
        auto heavy = RigidBody::makeBox(1, 1, 1000, {0, 1.48}); heavy.restitution = 0; heavy.friction = 1;
        load.add(light); load.add(heavy); configure(load, slices);
        load.solver.config().iterations = 1000;
        real depth = 0;
        for (int f = 0; f < int(4 / dt); ++f) {
            load.bodies[2].applyForce({1, 0}, load.bodies[2].position);
            step(load, dt);
            for (const auto& m : load.manifolds()) depth = std::max(depth, m.penetration);
        }
        std::printf("LOAD dt=%.6f N=%d depth=%.6f y=%.6f speed=%.6f\n", dt, slices,
                    depth, load.bodies[2].position.y, load.bodies[2].velocity.length());
        REQUIRE(depth < 0.08 && load.bodies[2].position.y > 1.4, "active 1000:1 load penetration/height");
        REQUIRE(load.bodies[2].velocity.length() < 0.08, "active 1000:1 load bounded velocity");
    }
}
static real softLoad(real alpha, real dt, int slices) {
    World w;
    w.add(RigidBody::makeBox(20, 1, 0, {0, -0.5}));
    auto b = RigidBody::makeCircle(0.5, 10, {0, 0.49}); b.restitution = 0; w.add(b);
    configure(w, slices); w.solver.config().compliance = alpha; w.solver.config().damping = 40;
    for (int f = 0; f < int(8 / dt); ++f) step(w, dt);
    const real compression = 0.5 - w.bodies[1].position.y - w.solver.config().slop;
    std::printf("SOFT alpha=%.6f dt=%.6f N=%d compression=%.6f target=%.6f\n",
                alpha, dt, slices, compression, alpha * 98.1);
    REQUIRE(std::abs(compression - alpha * 98.1) < 0.004, "soft contact static compression equals compliance times load");
    REQUIRE(w.bodies[1].velocity.length() < 0.005, "damped soft contact settles");
    return compression;
}
static void joints() {
    for (int slices : {1, 4, 16}) for (bool soft : {false, true}) {
        World w; w.gravity = {};
        w.add(RigidBody::makeCircle(0.2, 1000, {}));
        w.add(RigidBody::makeCircle(0.2, 1, {1, 0}));
        w.joints.push_back(Joint::distance(0, 1, {}, {}, 1, soft ? 3 : 0, 0.7));
        configure(w, slices);
        real error = 0;
        for (int f = 0; f < 600; ++f) {
            w.bodies[1].applyForce({2, 0}, w.bodies[1].position);
            step(w, 1.0 / 60);
            error = std::max(error, std::abs((w.bodies[1].position - w.bodies[0].position).length() - 1));
        }
        REQUIRE(error < (soft ? 0.012 : 0.004), "1000:1 driven rigid/soft distance error");
        const real momentum = w.bodies[0].mass * w.bodies[0].velocity.x + w.bodies[1].mass * w.bodies[1].velocity.x;
        REQUIRE(std::abs(momentum - 20) < 1e-7, "joint substeps do not multiply force or impulse momentum");
    }
    World zero; zero.gravity = {};
    zero.add(RigidBody::makeCircle(0.2, 0, {})); zero.add(RigidBody::makeCircle(0.2, 0, {}));
    zero.joints.push_back(Joint::distance(0, 1, {}, {}, 0, 3)); configure(zero, 4);
    for (int i = 0; i < 20; ++i) step(zero, 1.0 / 60);
    REQUIRE(zero.bodies[0].position.lengthSq() == 0, "static/static coincident anchors stay finite and fixed");
    World degenerate; degenerate.gravity = {};
    degenerate.add(RigidBody::makeCircle(0.2, 0, {})); degenerate.add(RigidBody::makeCircle(0.2, 1, {}));
    degenerate.joints.push_back(Joint::distance(0, 1, {}, {}, 0)); configure(degenerate, 4);
    step(degenerate, 1.0 / 60);
    REQUIRE(degenerate.bodies[1].position.lengthSq() == 0, "coincident dynamic distance anchor remains fixed");
}
static void impactAndAnchors() {
    for (int slices : {1, 4, 16}) {
        World w; w.gravity = {};
        w.add(RigidBody::makeCircle(0.5, 1, {-0.49, 0}));
        w.add(RigidBody::makeCircle(0.5, 1, {0.49, 0}));
        w.bodies[0].velocity = {2, 0}; w.bodies[1].velocity = {-2, 0};
        for (auto& b : w.bodies) { b.restitution = 1; b.friction = 0; }
        configure(w, slices);
        step(w, 1.0 / 60);
        REQUIRE(std::abs(w.bodies[0].velocity.x + 2) < 1e-9
            && std::abs(w.bodies[1].velocity.x - 2) < 1e-9, "elastic impulse occurs once, not per slice");
        for (int f = 0; f < 30; ++f) step(w, 1.0 / 60);
        REQUIRE(std::abs(w.bodies[0].velocity.x + 2) < 1e-9, "separating contacts do not rebounce");
    }
    World anchor; anchor.gravity = {};
    anchor.add(RigidBody::makeBox(2, 2, 0, {}));
    anchor.add(RigidBody::makeBox(1, 1, 1, {1.4, 0.2}));
    ContactManifold m;
    REQUIRE(collide(anchor.bodies[0], anchor.bodies[1], m), "anchor fixture overlaps");
    std::vector<ContactManifold> contacts{m}; std::vector<TemporalContact> rows;
    SolverConfig cfg; cfg.mode = SolverMode::TGS;
    prepareTemporal(contacts, rows, cfg);
    anchor.bodies[1].position += Vec2{0.03, 0.02}; anchor.bodies[1].setAngle(0.12);
    refreshTemporal(contacts, rows, cfg, 1.0 / 480, false);
    for (int p = 0; p < m.count; ++p) {
        const auto& row = rows[0].points[p]; const auto& c = contacts[0].contactPoints[p];
        const real expected = row.separation + dot(anchor.bodies[1].position + anchor.bodies[1].rot * row.localB
            - (anchor.bodies[0].position + anchor.bodies[0].rot * row.localA), m.normal);
        REQUIRE(std::abs(c.depth + expected) < 1e-12, "local-anchor separation refresh includes translation and rotation");
    }
    // Direct static/static rows and duplicate points exercise zero and singular mass.
    anchor.bodies[1].setStatic(); contacts[0].count = 2;
    contacts[0].contactPoints[1] = contacts[0].contactPoints[0];
    prepareTemporal(contacts, rows, cfg); refreshTemporal(contacts, rows, cfg, 1.0 / 60, true);
    solveTemporal(contacts, cfg, 1.0 / 60);
    REQUIRE(anchor.bodies[1].velocity.lengthSq() == 0, "static duplicate rows are inert");
}

static real oscillation(real damping) {
    World w; w.gravity = {};
    w.add(RigidBody::makeBox(20, 1, 0, {0, -0.5}));
    auto b = RigidBody::makeCircle(0.5, 1, {0, 0.45}); b.restitution = 0; w.add(b);
    configure(w, 4); w.solver.config().compliance = 0.001; w.solver.config().damping = damping;
    // Constant external compression keeps the unilateral spring engaged.
    real late = 0, peakEnergy = 0;
    for (int f = 0; f < 480; ++f) {
        w.bodies[1].force = {0, -20}; step(w, 1.0 / 120);
        const real error = w.bodies[1].position.y - 0.47;
        if (f > 360) late = std::max(late, std::abs(error));
        peakEnergy = std::max(peakEnergy, 0.5 * w.bodies[1].velocity.lengthSq() + 500 * error * error);
    }
    REQUIRE(peakEnergy < 0.21, "compressed compliant oscillator does not gain energy above initial 0.2 J");
    std::printf("DAMP c=%.1f late_error=%.9f peak_energy=%.9f\n", damping, late, peakEnergy);
    return late;
}

static void policies() {
    REQUIRE(oscillation(40) < oscillation(0) * 0.1, "explicit damping measurably reduces late oscillation");
    for (bool warm : {false, true}) {
        auto w = makeStack(5); configure(w, 8); w.solver.config().warmStarting = warm;
        for (int f = 0; f < 360; ++f) step(w, f % 2 ? 1.0 / 120 : 1.0 / 60);
        REQUIRE(std::abs(w.bodies[5].position.y - 4.5) < 0.12 && w.bodies[5].velocity.length() < 0.05,
                "warm/cold variable-timestep resting stack bounds");
    }
    for (int scene : {3, 4, 5, 6}) {
        auto w = makeShowcase(scene); configure(w, 8);
        // The bridge has 102 bodies: preserve the same bounded owners with adequate capacity.
        w.prepare({128, 256, 8128, 8128, 512});
        for (int f = 0; f < 120; ++f) step(w, 1.0 / 60);
    }
}

static void boundaries() {
    World empty; configure(empty, 4); step(empty, 1.0 / 60);
    World force; force.gravity = {}; auto b = RigidBody::makeCircle(0.5, 2, {});
    b.bullet = true; force.add(b); configure(force, 8);
    force.bodies[0].force = {12, 0}; step(force, 0.1);
    REQUIRE(std::abs(force.bodies[0].velocity.x - 0.6) < 1e-12 && force.bodies[0].force.lengthSq() == 0,
            "CCD slices preserve applied forces until tick completion");
    auto invalid = makeStack(1); configure(invalid, 4);
    invalid.solver.config().substeps = 0;
    const Vec2 original = invalid.bodies[1].position;
    bool rejected = false;
    try { invalid.step(1.0 / 60); } catch (const std::invalid_argument&) { rejected = true; }
    REQUIRE(rejected && invalid.bodies[1].position.y == original.y, "invalid slice count rejected before state mutation");
    for (real alpha : {-1.0, std::numeric_limits<real>::quiet_NaN()}) {
        invalid.solver.config().substeps = 4; invalid.solver.config().compliance = alpha; rejected = false;
        try { invalid.step(1.0 / 60); } catch (const std::invalid_argument&) { rejected = true; }
        REQUIRE(rejected, "invalid compliance rejected");
    }
    // A legal but subnormal slice duration underflows h*(h+c*alpha) to zero, so
    // gamma overflows. Reject it instead of propagating inf into the impulse solve.
    const Vec2 beforeSoft = invalid.bodies[1].position;
    invalid.solver.config().substeps = 1000000000;
    invalid.solver.config().compliance = 1e-300; invalid.solver.config().damping = 0; rejected = false;
    try { invalid.step(1e-300); } catch (const std::invalid_argument&) { rejected = true; }
    REQUIRE(rejected, "unrepresentable subnormal softness rejected before mutation");
    REQUIRE(invalid.bodies[1].position.y == beforeSoft.y, "rejected softness leaves state untouched");
    auto mode = makeStack(3); configure(mode, 4); mode.sleeping = true;
    for (int f = 0; f < 300; ++f) step(mode, 1.0 / 60);
    mode.solver.config().mode = SolverMode::PGS; step(mode, 1.0 / 60);
    mode.solver.config().mode = SolverMode::TGS; mode.solver.config().substeps = 16; step(mode, 1.0 / 30);
    mode.bodies[3].applyForce({1, 0}, mode.bodies[3].position); step(mode, 1.0 / 60);
    REQUIRE(mode.bodies[1].awake, "temporal mode changes and sleeping wake propagation");
    auto capacity = makeStack(1); capacity.bodies[1].position.y -= 0.1; configure(capacity, 4);
    capacity.prepare({2, 0, 1, 0, 8});
    testAllocations = 0; countTestAllocations = true;
    const auto status = capacity.tryStep(1.0 / 60); countTestAllocations = false;
    REQUIRE(status == World::StepStatus::Contacts && testAllocations == 0, "temporal rows share observable manifold capacity");
    capacity.prepare({2, 0, 1, 1, 8}); step(capacity, 1.0 / 60);
}
int main() {
    stacks(); joints(); boundaries(); impactAndAnchors(); policies();
    for (real dt : {1.0 / 30, 1.0 / 120}) for (int slices : {1, 8}) {
        const real hard = softLoad(0.0002, dt, slices), soft = softLoad(0.001, dt, slices);
        REQUIRE(soft > hard * 4, "softer material has larger measured deformation");
    }
    std::printf("%s temporal tests (%d failed); TemporalPoint %zu/%zu TemporalContact %zu/%zu\n",
                failures ? "FAIL" : "PASS", failures, sizeof(TemporalPoint), alignof(TemporalPoint),
                sizeof(TemporalContact), alignof(TemporalContact));
    return failures ? 1 : 0;
}
