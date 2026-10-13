// Headless sanity tests - no graphics. Run after every change to the engine.
#include "World.hpp"
#include "Scenes.hpp"
#include "IntegratorStudy.hpp"
#include "Showcase.hpp"
#include <chrono>
#include <random>
#include <cstdio>
#include <cstdlib>

using namespace phys;

static int g_failed = 0;
#define CHECK(cond, ...)                                                   \
    do {                                                                   \
        if (!(cond)) { ++g_failed; std::printf("  FAIL: "); std::printf(__VA_ARGS__); std::printf("\n"); } \
        else { std::printf("  ok:   "); std::printf(__VA_ARGS__); std::printf("\n"); } \
    } while (0)

static bool near(real a, real b, real tol) { return std::abs(a - b) <= tol; }

static void testMassProperties() {
    std::puts("[mass properties]");
    RigidBody box = RigidBody::makeBox(2, 1, 3, {});
    CHECK(near(box.inertia, 3 * (4 + 1) / 12.0, 1e-9), "box I = m(w^2+h^2)/12 = %.4f", box.inertia);

    // Same box, but vertices given far from the origin and clockwise.
    RigidBody p = RigidBody::makePolygon({{5, 5}, {5, 6}, {7, 6}, {7, 5}}, 3, {});
    CHECK(near(p.inertia, box.inertia, 1e-9), "polygon I matches box (%.4f), CW input + offset handled", p.inertia);
    CHECK(near(p.shape.verts[0].x + p.shape.verts[2].x, 0, 1e-9), "vertices re-centred on centroid");

    RigidBody c = RigidBody::makeCircle(0.5, 2, {});
    CHECK(near(c.inertia, 0.25, 1e-12), "circle I = m r^2 / 2 = %.4f", c.inertia);
}

static void testSymplecticEnergy() {
    std::puts("[symplectic Euler: harmonic oscillator energy]");
    const real k = 100, dt = 0.01;                       // omega*dt = 0.1
    // Symplectic: velocity first, then position with the NEW velocity.
    RigidBody b = RigidBody::makeCircle(0.1, 1, {1, 0});
    auto energy = [&](const RigidBody& r) {
        return 0.5 * r.mass * r.velocity.lengthSq() + 0.5 * k * r.position.lengthSq();
    };
    const real e0 = energy(b);
    real earlyMax = 0, lateMax = 0, emin = e0;
    const int N = 100000;
    for (int i = 0; i < N; ++i) {
        b.force = -k * b.position;
        symplecticEuler(b, {}, dt);
        const real e = energy(b);
        emin = std::min(emin, e);
        if (i < N / 10) earlyMax = std::max(earlyMax, e);
        if (i >= N - N / 10) lateMax = std::max(lateMax, e);
    }
    // True energy oscillates ~ +-omega*dt/2 around a conserved shadow energy - but never drifts.
    CHECK(lateMax <= earlyMax * 1.001, "no secular drift over 1000 s: late max %.4f vs early max %.4f (E0=%.1f)",
          lateMax, earlyMax, e0);
    CHECK((earlyMax - emin) / e0 < 0.12, "bounded oscillation ~ omega*dt: %.1f%% of E0", 100 * (earlyMax - emin) / e0);

    // Contrast: EXPLICIT Euler (position uses OLD velocity) multiplies energy by (1+(omega dt)^2) each step.
    Vec2 x{1, 0}, v{};
    for (int i = 0; i < 2000; ++i) { Vec2 a = -k * x; x += v * dt; v += a * dt; }
    const real eExplicit = 0.5 * v.lengthSq() + 0.5 * k * x.lengthSq();
    CHECK(eExplicit > 100 * e0, "explicit Euler blows up in 20 s: E/E0 = %.0f (shown for contrast)", eExplicit / e0);
}

static void testRestingBox() {
    std::puts("[box dropped on ground]");
    World w;
    w.add(RigidBody::makeBox(40, 1, 0, {0, -0.5}));                 // static ground, top at y=0
    const auto i = w.add(RigidBody::makeBox(1, 1, 1, {0, 3}, 0.3)); // tilted
    for (int s = 0; s < 60 * 5; ++s) w.step(1.0 / 60);
    const RigidBody& b = w.bodies[i];
    const real ang = std::remainder(b.angle, 1.5707963267948966);
    CHECK(near(b.position.y, 0.5, 0.03), "rests at y = %.4f (expected ~0.49-0.5)", b.position.y);
    CHECK(near(ang, 0, 0.02), "settles flat, angle mod 90deg = %.4f", ang);
    CHECK(b.velocity.length() < 0.05 && std::abs(b.angularVelocity) < 0.05, "at rest (|v|=%.4f, w=%.4f)",
          b.velocity.length(), b.angularVelocity);
}

static void testBounce() {
    std::puts("[circle restitution]");
    World w;
    w.add(RigidBody::makeBox(40, 1, 0, {0, -0.5}));
    RigidBody ball = RigidBody::makeCircle(0.5, 1, {0, 5.5});
    ball.restitution = 0.8;
    const auto i = w.add(ball);
    bool bounced = false;
    real peak = 0;
    for (int s = 0; s < 60 * 4; ++s) {
        const real vy = w.bodies[i].velocity.y;
        w.step(1.0 / 60);
        if (vy < 0 && w.bodies[i].velocity.y > 0) bounced = true;
        if (bounced) peak = std::max(peak, w.bodies[i].position.y);
    }
    // Drop height 5 -> rebound height e^2 * 5 = 3.2 above contact height 0.5.
    CHECK(bounced && near(peak - 0.5, 3.2, 0.5), "first rebound height = %.2f (ideal 3.2)", peak - 0.5);
}

static void testFrictionSlope() {
    std::puts("[Coulomb friction on a 0.35 rad slope: tan(0.35)=0.365]");
    const real th = 0.35;
    const Vec2 up{-std::sin(th), std::cos(th)};
    const Vec2 downhill{-std::cos(th), -std::sin(th)};
    for (real mu : {0.6, 0.05}) {
        World w;
        RigidBody ground = RigidBody::makeBox(60, 1, 0, {0, 0}, th);
        ground.friction = mu;
        w.add(ground);
        RigidBody box = RigidBody::makeBox(1, 1, 1, up * 1.0, th);
        box.friction = mu;
        const Vec2 start = box.position;
        const auto i = w.add(box);
        for (int s = 0; s < 60 * 3; ++s) w.step(1.0 / 60);
        const real slid = dot(w.bodies[i].position - start, downhill);
        if (mu > 0.365) CHECK(std::abs(slid) < 0.1, "mu=%.2f > tan(theta): holds, slid %.3f m", mu, slid);
        else            CHECK(slid > 5, "mu=%.2f < tan(theta): slides, %.2f m in 3 s", mu, slid);
    }
}

static void testStack() {
    std::puts("[3-box stack]");
    World w;
    w.solver.config().iterations = 20;
    w.add(RigidBody::makeBox(40, 1, 0, {0, -0.5}));
    std::size_t top = 0;
    for (int k = 0; k < 3; ++k) top = w.add(RigidBody::makeBox(1, 1, 1, {0, 0.5 + k * 1.0}));
    for (int s = 0; s < 60 * 5; ++s) w.step(1.0 / 60);
    const RigidBody& b = w.bodies[top];
    CHECK(near(b.position.y, 2.5, 0.1), "top box y = %.3f (ideal 2.5)", b.position.y);
    CHECK(std::abs(b.position.x) < 0.05, "no lateral drift: x = %.4f", b.position.x);
    CHECK(b.velocity.length() < 0.05, "stack at rest: |v| = %.4f", b.velocity.length());
}

static void testAABBs() {
    std::puts("[AABB geometry]");
    const AABB a{{-1, -2}, {1, 2}};
    CHECK(a.overlaps({{1, 2}, {3, 4}}), "touching bounds overlap conservatively");
    CHECK(!a.overlaps({{1.01, 0}, {3, 1}}), "separated bounds reject");
    CHECK(a.fatten(0.1).contains(a) && near(a.perimeter(), 12, 1e-12), "fat containment and perimeter");
    const auto circle = computeAABB(RigidBody::makeCircle(2, 1, {3, -4}));
    CHECK(near(circle.min.x, 1, 1e-12) && near(circle.max.y, -2, 1e-12), "circle bounds");
    const auto box = computeAABB(RigidBody::makeBox(2, 4, 1, {}, 0.5));
    const real extent = std::cos(0.5) + 2 * std::sin(0.5);
    CHECK(near(box.max.x, extent, 1e-12) && near(box.min.x, -extent, 1e-12), "rotated polygon bounds");
}

static void testDynamicTree() {
    std::puts("[dynamic tree: sorted insertion, movement, removal, reuse]");
    DynamicTree tree;
    std::vector<int> proxies, scratch;
    bool valid = true;
    for (std::size_t i = 0; i < 1024; ++i) {
        const real x = 2 * static_cast<real>(i);
        proxies.push_back(tree.createProxy({{x, 0}, {x + 1, 1}}, i));
        valid = valid && tree.validate();
    }
    CHECK(valid && tree.height() <= 15, "sorted insertion stays balanced, height=%d", tree.height());
    CHECK(!tree.moveProxy(proxies[0], {{0.05, 0}, {1.05, 1}}), "small motion retains fat proxy");
    CHECK(tree.moveProxy(proxies[0], {{-20, -20}, {-19, -19}}), "teleport reinserts proxy");
    std::vector<std::size_t> hits;
    tree.query({{-20, -20}, {-19, -19}}, [&](std::size_t id) { hits.push_back(id); }, scratch);
    CHECK(hits == std::vector<std::size_t>{0}, "query finds teleported leaf only");
    valid = tree.validate();
    for (int id : proxies) { tree.destroyProxy(id); valid = valid && tree.validate(); }
    CHECK(valid && tree.height() == 0, "removal through empty tree preserves invariants");
    const int recycled = tree.createProxy({{0, 0}, {1, 1}}, 42);
    hits.clear();
    tree.query({{0, 0}, {1, 1}}, [&](std::size_t id) { hits.push_back(id); }, scratch);
    CHECK(tree.validate() && hits == std::vector<std::size_t>{42}, "free-list reuse has no stale leaf");
    tree.destroyProxy(recycled);
}

static std::vector<BodyPair> contactPairs(std::vector<RigidBody>& bodies,
                                        const std::vector<BodyPair>& candidates) {
    std::vector<BodyPair> result;
    for (const auto& pair : candidates) {
        ContactManifold m;
        if (collide(bodies[pair.first], bodies[pair.second], m)) result.push_back(pair);
    }
    return result;
}

static void testBroadphaseOracle() {
    std::puts("[broadphase: fixed-seed oracle and public body mutations]");
    std::mt19937 rng(0xBADC0DE);
    std::uniform_real_distribution<real> pos(-12, 12), size(0.1, 2), angle(-3.14, 3.14);
    std::vector<RigidBody> bodies;
    for (int i = 0; i < 200; ++i) {
        const Vec2 p{pos(rng), pos(rng)};
        const real mass = i % 7 == 0 ? 0 : 1;
        if (i % 3 == 0) bodies.push_back(RigidBody::makeCircle(size(rng), mass, p));
        else bodies.push_back(RigidBody::makeBox(size(rng), size(rng), mass, p, angle(rng)));
    }
    Broadphase broad, oracle;
    bool match = true, boundsMatch = true, valid = true;
    for (int frame = 0; frame < 50; ++frame) {
        for (auto& b : bodies) {
            b.position += Vec2{pos(rng), pos(rng)} * 0.03;
            b.setAngle(b.angle + angle(rng) * 0.02);
        }
        if (frame == 5) bodies[0].position = {100, -100};
        if (frame == 10) bodies.push_back(RigidBody::makeCircle(1, 1, {}));
        if (frame == 15) bodies.erase(bodies.begin() + 17);
        if (frame == 20) std::reverse(bodies.begin(), bodies.end());
        if (frame == 25) bodies[8] = RigidBody::makeBox(30, 0.2, 1, {}, 0.8);
        const auto pairs = broad.findPairs(bodies);
        const auto all = oracle.findPairs(bodies, BroadphaseMode::AllPairs);
        match = match && contactPairs(bodies, pairs) == contactPairs(bodies, all);
        std::vector<BodyPair> expected;
        for (const auto& pair : all)
            if (computeAABB(bodies[pair.first]).overlaps(computeAABB(bodies[pair.second])))
                expected.push_back(pair);
        boundsMatch = boundsMatch && pairs == expected;
        valid = valid && broad.tree().validate();
    }
    CHECK(match, "identical narrowphase contact-pair sets over 50 randomized frames");
    CHECK(boundsMatch, "candidate pairs exactly equal tight-AABB oracle, sorted and unique");
    CHECK(valid, "tree invariants survive rotation, teleport, grow, erase, reorder, replacement");
    bodies.clear();
    CHECK(broad.findPairs(bodies).empty() && broad.tree().validate(), "empty world resets proxies");
    bodies.push_back(RigidBody::makeBox(1, 1, 0, {}));
    bodies.push_back(RigidBody::makeCircle(1, 0, {}));
    CHECK(broad.findPairs(bodies).empty(), "static/static pairs omitted");
}

static void testWorldOracle() {
    std::puts("[world: deterministic tree vs all-pairs trajectories]");
    World tree, oracle;
    tree.add(RigidBody::makeBox(40, 1, 0, {0, -0.5}));
    for (int i = 0; i < 20; ++i)
        tree.add(RigidBody::makeBox(0.8, 0.8, 1, {real(i % 5) * 1.1 - 2, 1 + real(i / 5)}, 0.1));
    oracle.bodies = tree.bodies;
    oracle.broadphaseMode = BroadphaseMode::AllPairs;
    bool identical = true, finite = true;
    for (int frame = 0; frame < 600; ++frame) {
        tree.step(1.0 / 60); oracle.step(1.0 / 60);
        for (std::size_t i = 0; i < tree.bodies.size(); ++i) {
            const auto& a = tree.bodies[i]; const auto& b = oracle.bodies[i];
            identical = identical && a.position.x == b.position.x && a.position.y == b.position.y
                && a.velocity.x == b.velocity.x && a.velocity.y == b.velocity.y
                && a.angle == b.angle && a.angularVelocity == b.angularVelocity;
            finite = finite && std::isfinite(a.position.x) && std::isfinite(a.position.y)
                && std::isfinite(a.velocity.lengthSq()) && std::isfinite(a.angularVelocity);
        }
    }
    CHECK(identical, "bit-identical trajectories over 600 fixed steps");
    CHECK(finite, "all simulated states remain finite");
}

static void testWarmStarting() {
    std::puts("[persistent feature IDs and warm impulses]");
    auto a = RigidBody::makeBox(8, 1, 0, {0, -0.5});
    auto b = RigidBody::makeBox(1, 1, 1, {0, 0.49});
    ContactManifold m, shifted;
    const bool first = collide(a, b, m);
    b.position.x += 0.02;
    const bool second = collide(a, b, shifted);
    CHECK(first && second && m.count == 2 && shifted.count == 2
        && m.contactPoints[0].featureId != m.contactPoints[1].featureId
        && m.contactPoints[0].featureId == shifted.contactPoints[0].featureId
        && m.contactPoints[1].featureId == shifted.contactPoints[1].featureId,
        "clipped contact IDs are distinct and stable under tangential motion");
    std::vector<ContactManifold> contacts{shifted};
    contacts[0].contactPoints[0].accN = 2;
    contacts[0].contactPoints[0].accT = 0.1;
    ContactSolver solver;
    solver.prepare(contacts, 1.0 / 60);
    CHECK(near(b.velocity.y, 2, 1e-12) && near(b.velocity.x, 0.1, 1e-12),
        "cached normal and tangent impulses actually seed body velocity");
    b.velocity = {}; b.angularVelocity = 0;
    solver.config().warmStarting = false;
    solver.prepare(contacts, 1.0 / 60);
    CHECK(b.velocity.lengthSq() == 0 && contacts[0].contactPoints[0].accN == 0,
        "cold-start option clears accumulated impulses");
    World w = makeStack(1);
    for (int i = 0; i < 120; ++i) w.step(1.0 / 60);
    w.bodies[1].position = {0, 10};
    w.step(1.0 / 60);
    CHECK(w.manifolds().empty() && near(w.bodies[1].velocity.y, -9.81 / 60, 1e-8),
        "teleport invalidates old support impulses");
}

static void testTallScenes() {
    std::puts("[warm-start 20-box stack and 15-row pyramid: 20 seconds]");
    for (bool pyramid : {false, true}) {
        World w = pyramid ? makePyramid() : makeStack();
        const auto initial = w.bodies;
        real maxSpeed = 0, maxDrift = 0, maxTilt = 0, maxDepth = 0;
        bool finite = true;
        for (int frame = 0; frame < 1200; ++frame) {
            w.step(1.0 / 60);
            for (std::size_t i = 1; i < w.bodies.size(); ++i) {
                const auto& b = w.bodies[i];
                finite = finite && std::isfinite(b.position.lengthSq()) && std::isfinite(b.velocity.lengthSq());
                if (frame >= 900) {
                    maxSpeed = std::max(maxSpeed, b.velocity.length());
                    maxDrift = std::max(maxDrift, std::abs(b.position.x - initial[i].position.x));
                    maxTilt = std::max(maxTilt, std::abs(b.angle));
                }
            }
            if (frame >= 900) for (const auto& m : w.manifolds()) maxDepth = std::max(maxDepth, m.penetration);
        }
        CHECK(finite && maxSpeed < 0.05 && maxDrift < 0.15 && maxTilt < 0.03 && maxDepth < 0.03,
            "%s stable: speed=%.5f drift=%.5f tilt=%.5f depth=%.5f",
            pyramid ? "15-row pyramid" : "20-box stack", maxSpeed, maxDrift, maxTilt, maxDepth);
        CHECK(w.bodies.back().position.y > initial.back().position.y - 0.35,
            "%s retains height: top=%.4f ideal=%.1f", pyramid ? "pyramid" : "stack",
            w.bodies.back().position.y, initial.back().position.y);
    }
}

static void testJoints() {
    std::puts("[joint constraints]");
    for (real frequency : {real(0), real(3)}) {
        World w; w.gravity = {}; w.solver.config().iterations = 20;
        w.add(RigidBody::makeCircle(0.1, 0, {}));
        w.add(RigidBody::makeCircle(0.1, 1, {2, 0}));
        w.joints.push_back(Joint::distance(0, 1, {}, {}, 1, frequency));
        for (int f = 0; f < 600; ++f) w.step(1.0 / 60);
        CHECK(near(w.bodies[1].position.length(), 1, 1e-5),
            "distance frequency=%.1f reaches rest length: %.8f", frequency, w.bodies[1].position.length());
    }
    World pendulum;
    pendulum.add(RigidBody::makeCircle(0.02, 0, {}));
    pendulum.add(RigidBody::makeCircle(0.02, 1, {std::sin(0.08)*2, -std::cos(0.08)*2}));
    pendulum.joints.push_back(Joint::distance(0, 1, {}, {}, 2));
    std::vector<real> crossings;
    for (int f = 0; f < 1200; ++f) {
        const real x = pendulum.bodies[1].position.x;
        pendulum.step(1.0 / 60);
        if (x > 0 && pendulum.bodies[1].position.x <= 0) crossings.push_back(real(f)/60);
    }
    const real period = crossings.size() > 1 ? crossings[1] - crossings[0] : 0;
    CHECK(near(period, 2*3.14159265358979323846*std::sqrt(2/9.81), 0.05),
        "small-angle pendulum period=%.4f s", period);
    World chain; chain.solver.config().iterations = 40;
    chain.add(RigidBody::makeCircle(0.1, 0, {}));
    for (int i=0; i<10; ++i) {
        chain.add(RigidBody::makeBox(0.2, 1, 1, {0, -0.5-i}));
        chain.joints.push_back(Joint::revolute(std::size_t(i), std::size_t(i+1),
            i==0 ? Vec2{} : Vec2{0,-0.5}, {0,0.5}));
    }
    chain.bodies.back().applyImpulse({1,0}, {});
    for (int f=0; f<600; ++f) chain.step(1.0/60);
    real error=0;
    for (const auto& j : chain.joints)
        error=std::max(error,(chain.bodies[j.b].position+chain.bodies[j.b].rot*j.localB
                             -chain.bodies[j.a].position-chain.bodies[j.a].rot*j.localA).length());
    CHECK(error<0.03, "10-link revolute chain anchor error=%.6f",error);
    World motor; motor.gravity={};
    motor.add(RigidBody::makeCircle(0.1,0,{})); motor.add(RigidBody::makeBox(1,1,1,{}));
    auto hinge=Joint::revolute(0,1,{},{}); hinge.motor=true; hinge.motorSpeed=2; hinge.maxMotorTorque=10;
    motor.joints.push_back(hinge);
    for (int f=0;f<120;++f) motor.step(1.0/60);
    CHECK(near(motor.bodies[1].angularVelocity,2,1e-9), "motor reaches prescribed speed");
    motor.joints[0].limits=true; motor.joints[0].lower=-0.3; motor.joints[0].upper=0.3;
    motor.bodies[1].setAngle(0); motor.clearContactCache();
    for (int f=0;f<300;++f) motor.step(1.0/60);
    CHECK(motor.bodies[1].angle<=0.301 && motor.bodies[1].angle>=-0.301,
        "motor respects angular limit: angle=%.6f",motor.bodies[1].angle);
    World mouse; mouse.gravity={}; mouse.add(RigidBody::makeBox(1,1,1,{}));
    mouse.joints.push_back(Joint::mouse(0,{}, {3,2}));
    for(int f=0;f<300;++f)mouse.step(1.0/60);
    CHECK((mouse.bodies[0].position-Vec2{3,2}).length()<1e-5, "mouse spring converges to target");
    for (bool weld : {false,true}) {
        World w; w.gravity={}; w.add(RigidBody::makeCircle(0.1,0,{}));
        w.add(RigidBody::makeBox(1,1,1,{0,1},0.2));
        w.joints.push_back(weld ? Joint::weld(0,1,{}, {},0)
            : Joint::prismatic(0,1,{}, {}, {1,0},0));
        w.bodies[1].velocity={1,0};
        for(int f=0;f<300;++f)w.step(1.0/60);
        CHECK(std::abs(w.bodies[1].position.y)<1e-6 && std::abs(w.bodies[1].angle)<1e-6
            && (weld ? w.bodies[1].position.length()<1e-6 : w.bodies[1].position.x>4),
            "%s constrains intended degrees of freedom", weld ? "weld" : "prismatic");
    }
}

static void testStabilityUpgrades() {
    std::puts("[NGS and sleeping islands]");
    World w; w.gravity={}; w.solver.config().positionCorrection=true;
    w.add(RigidBody::makeBox(8,1,0,{0,-0.5}));
    w.add(RigidBody::makeBox(1,1,1,{0,0.3}));
    w.step(1.0/60);
    CHECK(w.bodies[1].position.y>0.45 && w.bodies[1].velocity.lengthSq()==0,
        "NGS reduces overlap without adding kinetic velocity: y=%.6f",w.bodies[1].position.y);
    World sleep=makeStack(3);sleep.sleeping=true;
    for(int f=0;f<300;++f)sleep.step(1.0/60);
    bool asleep=true;for(std::size_t i=1;i<sleep.bodies.size();++i)asleep=asleep&&!sleep.bodies[i].awake;
    CHECK(asleep,"contact-connected stack sleeps as one island");
    sleep.bodies.back().applyForce({10,0},sleep.bodies.back().position);
    sleep.step(1.0/60);
    bool awake=true;for(std::size_t i=1;i<sleep.bodies.size();++i)awake=awake&&sleep.bodies[i].awake;
    CHECK(awake,"external force wakes the entire connected island");
    World separate;separate.sleeping=true;
    separate.add(RigidBody::makeBox(40,1,0,{0,-0.5}));
    separate.add(RigidBody::makeBox(1,1,1,{-5,0.49}));
    separate.add(RigidBody::makeBox(1,1,1,{5,0.49}));
    separate.step(1.0/60);
    CHECK(separate.islands().island(1)!=separate.islands().island(2),"static floor does not merge independent islands");
    bool rejected=false;try{separate.step(0);}catch(const std::invalid_argument&){rejected=true;}
    CHECK(rejected,"nonpositive dt rejected");
}

static void testCCD() {
    std::puts("[continuous shape casts and thin bullets]");
    // Discrete travel per step at these speeds is 3.3 m to 167 m: every case
    // below would tunnel with narrowphase sampling alone.
    for(bool polygon:{false,true})for(real velocity:{real(200),real(1000),real(10000)}) {
        World w;w.gravity={};
        auto wall=RigidBody::makeBox(0.02,8,0,{});wall.restitution=0;w.add(wall);
        auto bullet=polygon ? RigidBody::makeBox(0.08,0.02,1,{-2,0},0.1)
                            : RigidBody::makeCircle(0.04,1,{-2,0});
        bullet.velocity={velocity,0};bullet.restitution=0;bullet.bullet=true;w.add(bullet);
        w.step(1.0/60);
        CHECK(w.bodies[1].position.x<0.0 && w.ccdStats.events>0 && !w.ccdStats.capped,
            "%s bullet %.0f m/s stops before 0.02m wall: x=%.6f events=%d pushed=%d",
            polygon?"thin polygon":"circle",velocity,w.bodies[1].position.x,
            w.ccdStats.events,w.ccdStats.pushedOut);
        CHECK(w.bodies[0].position.x==0,"static wall is never moved by the sweep");
        // A thin tilted plate strikes corner-first, so the impulse legitimately
        // torques it; the guarantee is non-penetration, not a dead centre velocity.
        ContactManifold overlap;
        CHECK(!collide(w.bodies[0],w.bodies[1],overlap)||overlap.penetration<0.02,
            "%s bullet ends the step out of the wall (depth=%.6f)",polygon?"polygon":"circle",
            collide(w.bodies[0],w.bodies[1],overlap)?overlap.penetration:real(0));
        // The sweep resolves impacts at the centre of mass, so the outcome to assert
        // is a real rest state: no residual speed pressed into the wall and no
        // fabricated spin. A tilted plate used to leave the wall with ~8e3 rad/s and
        // a 72 m/s centre velocity from an approximate lever arm; the fallbacks
        // (clamp/push-out) are nets now, not the mechanism, and must stay idle here.
        CHECK(std::abs(w.bodies[1].velocity.x)<1e-6,
            "%s impact absorbs the normal velocity: vx=%.9f",polygon?"tilted polygon":"head-on circle",
            w.bodies[1].velocity.x);
        CHECK(w.bodies[1].velocity.length()<1e-6&&std::abs(w.bodies[1].angularVelocity)<1e-6,
            "%s comes to rest with no invented spin: |v|=%.9f |w|=%.9f",polygon?"tilted polygon":"circle",
            w.bodies[1].velocity.length(),w.bodies[1].angularVelocity);
        CHECK(w.ccdStats.clamped==0&&w.ccdStats.pushedOut==0&&w.ccdStats.unconverged==0,
            "%s resolves inside the sweep with no fallback: clamped=%d pushedOut=%d unconverged=%d",
            polygon?"tilted polygon":"circle",w.ccdStats.clamped,w.ccdStats.pushedOut,w.ccdStats.unconverged);
    }
    // A spinning bullet must not slip through either.
    {
        World w;w.gravity={};
        auto wall=RigidBody::makeBox(0.02,8,0,{});wall.restitution=0;w.add(wall);
        auto spinner=RigidBody::makeBox(0.6,0.04,1,{-3,0});
        spinner.velocity={2000,0};spinner.angularVelocity=60;spinner.restitution=0;spinner.bullet=true;w.add(spinner);
        w.step(1.0/60);
        CHECK(w.bodies[1].position.x<0.0,"spinning thin bullet does not tunnel: x=%.6f",w.bodies[1].position.x);
        // Zero normal force at the resting contact means no friction torque either:
        // the plate is held in place and keeps the spin it arrived with.
        CHECK(w.bodies[1].velocity.length()<1e-6&&near(w.bodies[1].angularVelocity,60,1e-9),
            "spinning plate is held without losing its spin: |v|=%.9f w=%.6f",
            w.bodies[1].velocity.length(),w.bodies[1].angularVelocity);
    }
    // Regression: a bullet event must not freeze the rest of the world.
    {
        World w;w.gravity={};
        auto wall=RigidBody::makeBox(0.02,8,0,{});wall.restitution=0;w.add(wall);
        auto bullet=RigidBody::makeCircle(0.04,1,{-2,0});
        bullet.velocity={5000,0};bullet.restitution=0;bullet.bullet=true;w.add(bullet);
        auto bystander=RigidBody::makeCircle(0.5,1,{0,200});
        bystander.velocity={3,0};const auto i=w.add(bystander);
        w.step(1.0/60);
        CHECK(near(w.bodies[i].position.x,3/60.0,1e-12),
            "unrelated body keeps integrating during a bullet step: x=%.9f (expected %.9f)",
            w.bodies[i].position.x,3/60.0);
    }
    // Initially overlapping pairs belong to the discrete solver, not the sweep.
    {
        World w;w.gravity={};
        w.add(RigidBody::makeBox(0.02,8,0,{}));
        auto overlapping=RigidBody::makeCircle(0.5,1,{});
        overlapping.bullet=true;w.add(overlapping);
        w.step(1.0/60);
        CHECK(w.ccdStats.skipped>0&&w.bodies[1].position.x>0.1,
            "already-touching pair is skipped by CCD and pushed out discretely: x=%.6f",w.bodies[1].position.x);
    }
    // No obstacle: the bullet must actually travel its full swept distance.
    {
        World w;w.gravity={};
        auto free=RigidBody::makeCircle(0.04,1,{-2,0});
        free.velocity={600,0};free.bullet=true;w.add(free);
        w.step(1.0/60);
        CHECK(near(w.bodies[0].position.x,-2+10,1e-9)&&w.ccdStats.events==0,
            "unobstructed bullet travels the full step: x=%.9f",w.bodies[0].position.x);
    }
    auto a=RigidBody::makeCircle(0.5,1,{-2,0});auto b=RigidBody::makeCircle(0.5,1,{2,0});
    a.velocity={10,0};b.velocity={-10,0};
    const auto toi=timeOfImpact(a,b,0.2);
    CHECK(toi.hit&&toi.converged&&near(toi.time,0.15,1e-5),"moving/moving circle TOI matches analytic root: %.8f",toi.time);
    b.position={2,4};
    CHECK(!timeOfImpact(a,b,0.2).hit,"separated sweep is not a false impact");
    b.position={2,4};
    const auto resting=timeOfImpact(a,b,0.2);
    CHECK(!resting.hit&&!resting.initiallyTouching&&resting.converged,
        "separated motionless pair reports no contact and no unresolved state");
    RigidBody touchingA=RigidBody::makeCircle(0.5,0,{}),touchingB=RigidBody::makeCircle(0.5,1,{0.9,0});
    touchingB.velocity={1,0};
    CHECK(timeOfImpact(touchingA,touchingB,0.2).initiallyTouching,
        "overlapping start is reported instead of silently cast forward");
}

template<class Policy> static real oscillatorError(real dt) {
    ParticleState s{{1,0},{}};
    const int count=int(std::round(1/dt));
    for(int i=0;i<count;++i)Policy::step(s,i*dt,dt,[](Vec2 x,real){return -x;});
    return (s.position-Vec2{std::cos(1.0),0}).length()
        +(s.velocity-Vec2{-std::sin(1.0),0}).length();
}
static void testIntegratorPolicies() {
    std::puts("[smooth-force policy convergence]");
    const real se=oscillatorError<SymplecticPolicy>(0.1)/oscillatorError<SymplecticPolicy>(0.05);
    const real vv=oscillatorError<VerletPolicy>(0.1)/oscillatorError<VerletPolicy>(0.05);
    const real rk=oscillatorError<RK4Policy>(0.1)/oscillatorError<RK4Policy>(0.05);
    CHECK(se>1.8&&se<2.2,"symplectic first-order convergence ratio %.4f",se);
    CHECK(vv>3.8&&vv<4.2,"Verlet second-order convergence ratio %.4f",vv);
    CHECK(rk>15&&rk<17,"RK4 fourth-order convergence ratio %.4f",rk);
}
static void testFuzz() {
    std::puts("[fixed-seed randomized no-gravity elastic fuzz]");
    std::mt19937 rng(1234567);std::uniform_real_distribution<real> speed(-0.5,0.5);
    World a;a.gravity={};a.solver.config().positionCorrection=true;
    for(int i=0;i<32;++i){auto b=RigidBody::makeCircle(0.3,1,{real(i%8)*0.8,real(i/8)*0.8});
        b.velocity={speed(rng),speed(rng)};b.restitution=1;b.friction=0;a.add(b);}
    World b;b.gravity=a.gravity;b.bodies=a.bodies;b.solver.config()=a.solver.config();
    const auto energy=[](const World& w){real e=0;for(const auto& body:w.bodies)e+=0.5*body.mass*body.velocity.lengthSq()
        +0.5*body.inertia*body.angularVelocity*body.angularVelocity;return e;};
    const real initial=energy(a);bool finite=true,deterministic=true;real maxEnergy=initial;
    for(int f=0;f<2000;++f){a.step(1.0/60);b.step(1.0/60);maxEnergy=std::max(maxEnergy,energy(a));
        for(std::size_t i=0;i<a.bodies.size();++i){const auto& x=a.bodies[i];const auto& y=b.bodies[i];
            finite=finite&&std::isfinite(x.position.lengthSq())&&std::isfinite(x.velocity.lengthSq());
            deterministic=deterministic&&x.position.x==y.position.x&&x.position.y==y.position.y
                &&x.velocity.x==y.velocity.x&&x.velocity.y==y.velocity.y;}}
    CHECK(finite,"fuzz has no nonfinite state over 2000 steps");
    CHECK(deterministic,"fixed seed produces bit-identical repeated trajectories");
    CHECK(maxEnergy<=initial*1.001,"fuzz has no energy blow-up: max/initial=%.8f",maxEnergy/initial);
}

static void testShowcaseSmoke() {
    std::puts("[showcase smoke: every scene factory stays finite]");
    bool finite=true;
    for(int scene=0;scene<kSceneCount;++scene) {
        auto w=makeShowcase(scene);
        for(int f=0;f<30;++f) {
            w.step(1.0/60);
            for(const auto& b:w.bodies)finite=finite&&std::isfinite(b.position.lengthSq())&&std::isfinite(b.velocity.lengthSq());
        }
    }
    CHECK(finite,"all %d showcase scene factories produce finite states",kSceneCount);
    auto w=makeStack(1);w.profiling=true;w.step(1.0/60);
    CHECK(w.timings.broadphase>=0&&w.timings.narrowphase>=0&&w.timings.solve>=0&&w.timings.integrate>=0,
        "phase timing instrumentation produces nonnegative measurements");
}

// Newton's cradle as a physics claim rather than a picture: identical elastic
// frictionless balls in a row, zero gravity, so total linear momentum is a strict
// conserved quantity. The chain must pass the momentum to the far ball.
static void testMomentumConservation() {
    std::puts("[Newton's cradle: momentum transfer]");
    World w=makeMomentumRow(5,5);
    const auto momentum=[&]{real p=0;for(const auto&b:w.bodies)p+=b.mass*b.velocity.x;return p;};
    const real initial=momentum();
    real farPeak=0;
    for(int f=0;f<180;++f){w.step(1.0/60);farPeak=std::max(farPeak,w.bodies.back().velocity.x);}
    CHECK(near(momentum(),initial,1e-9),"total x momentum conserved: %.9f -> %.9f",initial,momentum());
    CHECK(std::abs(w.bodies[0].velocity.x)<1e-6,"incoming ball stops: vx=%.9f",w.bodies[0].velocity.x);
    CHECK(near(farPeak,5,0.05),"far ball leaves with the incoming speed: %.6f m/s",farPeak);
    CHECK(near(w.bodies.back().velocity.x,initial,0.05),"far ball keeps the momentum: %.6f",w.bodies.back().velocity.x);
}

// The visual pendulum cradle must actually transfer momentum down the chain.
static void testPendulumCradle() {
    std::puts("[pendulum cradle swings the far ball]");
    World w=makeCradle();
    real farMaxX=-1e9,farMaxSpeed=0;
    for(int f=0;f<420;++f){w.step(1.0/60);
        farMaxX=std::max(farMaxX,w.bodies.back().position.x);
        farMaxSpeed=std::max(farMaxSpeed,w.bodies.back().velocity.length());}
    const real anchorX=w.bodies.back().position.x;
    CHECK(farMaxSpeed>1.5,"far ball is driven by the chain: peak speed=%.4f m/s",farMaxSpeed);
    CHECK(farMaxX>anchorX+0.3,"far ball swings out: reached x=%.4f (anchor %.4f)",farMaxX,anchorX);
}

// Motorized car with soft-spring suspension: prismatic axles, distance springs and
// wheel drive torque. This asserts the suspension actually works, not just that a
// box moved.
static void testMotorizedCar() {
    std::puts("[motorized car with prismatic suspension]");
    CarRig rig=makeCarRig();
    World& w=rig.world;
    const real startX=w.bodies[rig.chassis].position.x;
    const real startWheelAngle=w.bodies[rig.wheelLeft].angle-w.bodies[rig.chassis].angle;
    real maxTilt=0,peakSpin=0,minTravel=9,maxTravel=0;
    for(int f=0;f<900;++f){
        w.step(1.0/60);
        maxTilt=std::max(maxTilt,std::abs(w.bodies[rig.chassis].angle));
        peakSpin=std::max(peakSpin,std::abs(w.bodies[rig.wheelLeft].angularVelocity));
        // Skip the first second: the wheels are still dropping onto the road.
        if(f>60) for(const auto& j:w.joints) if(j.type==JointType::Prismatic) {
            const auto& A=w.bodies[j.a];const auto& B=w.bodies[j.b];
            const real travel=(B.position+B.rot*j.localB-A.position-A.rot*j.localA).length();
            minTravel=std::min(minTravel,travel);maxTravel=std::max(maxTravel,travel);
        }
    }
    const real distance=w.bodies[rig.chassis].position.x-startX;
    const real wheelTurn=std::abs(std::remainder(
        (w.bodies[rig.wheelLeft].angle-w.bodies[rig.chassis].angle)-startWheelAngle,6.283185307179586));
    CHECK(peakSpin>5&&wheelTurn>1.0,
        "wheels spin freely through the prismatic axle (lockAngle=false): peak |w|=%.2f rad/s, turned %.2f rad",peakSpin,wheelTurn);
    CHECK(distance>20,"car drives itself across the terrain: %.2f m",distance);
    CHECK(maxTilt<0.6,"car never tips over: max |angle|=%.3f rad",maxTilt);
    CHECK(std::abs(w.bodies[rig.chassis].angle)<0.05,"car finishes level: %.4f rad",w.bodies[rig.chassis].angle);
    CHECK(minTravel>0.0&&minTravel<0.5&&maxTravel>0.70,
        "suspension compresses and extends under load: %.4f..%.4f m (rest 0.70)",minTravel,maxTravel);
}

// Rube-Goldberg causality: the switch changes whether the machine completes.
static void testRubeGoldbergCausality() {
    std::puts("[Rube-Goldberg causality switch]");
    RubeMachine armed=makeRubeMachine(true);
    for(int f=0;f<720;++f)armed.world.step(1.0/60);
    RubeMachine idle=makeRubeMachine(false);
    for(int f=0;f<720;++f)idle.world.step(1.0/60);
    CHECK(armed.world.bodies[armed.flag].angle<-1.0,
        "with the trigger the machine completes: flag angle=%.4f rad",armed.world.bodies[armed.flag].angle);
    CHECK(std::abs(idle.world.bodies[idle.flag].angle)<0.05,
        "without the trigger nothing moves: flag angle=%.4f rad",idle.world.bodies[idle.flag].angle);
}

// The wrecking ball must leave the pyramid alone until it arrives, then wreck it.
static void testWreckingPyramid() {
    std::puts("[wrecking-ball pyramid]");
    World w=makeWreckingPyramid(20);
    const std::size_t pyramidBodies=w.bodies.size()-2;   // excludes anchor + ball
    const std::size_t ball=w.bodies.size()-1;
    // The pile settles by one slop distance over the first second (boxes are
    // placed exactly touching). Stability is therefore measured as residual
    // velocity while the ball is still travelling, not as absolute position.
    for(int f=0;f<90;++f)w.step(1.0/60);
    real restingSpeed=0;
    for(std::size_t i=1;i<pyramidBodies;++i)
        restingSpeed=std::max(restingSpeed,w.bodies[i].velocity.length());
    CHECK(w.bodies[ball].position.x<-9,"ball has not reached the pile yet: x=%.3f",w.bodies[ball].position.x);
    CHECK(restingSpeed<0.05,"pyramid is at rest while the ball swings in: max |v|=%.5f m/s",restingSpeed);
    const std::vector<RigidBody> before=w.bodies;
    for(int f=0;f<600;++f)w.step(1.0/60);
    real wreckedDrift=0;
    for(std::size_t i=1;i<pyramidBodies;++i)
        wreckedDrift=std::max(wreckedDrift,(w.bodies[i].position-before[i].position).length());
    CHECK(wreckedDrift>1.0,"pyramid is wrecked after the impact: max drift=%.4f m",wreckedDrift);
}

// Bridge acceptance: rigid-limit projection keeps a 100-plank deck connected.
static void testBridgeProjection() {
    std::puts("[100-plank suspension bridge]");
    World bridge=makeBridge();
    real error=0;
    for(int f=0;f<240;++f) {
        bridge.step(1.0/60);
        for(const auto& j:bridge.joints) {
            const auto& a=bridge.bodies[j.a];const auto& b=bridge.bodies[j.b];
            error=std::max(error,(b.position+b.rot*j.localB-a.position-a.rot*j.localA).length());
        }
    }
    CHECK(error<0.02,"100-plank bridge projection max anchor error=%.6f",error);
}

// Sleeping correctness only; the speed criterion belongs to benchmark_sleeping.
static void testStressSleeping() {
    std::puts("[stress 5000: settling, islands and sleep]");
    World w=makeStress(5000,true);
    for(int f=0;f<720;++f)w.step(1.0/60);
    std::size_t asleep=0;real maxSpeed=0;
    for(const auto& b:w.bodies){if(b.isStatic())continue;if(!b.awake)++asleep;maxSpeed=std::max(maxSpeed,b.velocity.length());}
    CHECK(asleep>4900,"settled field sleeps: %zu/%zu dynamic bodies asleep",asleep,w.bodies.size());
    CHECK(maxSpeed<0.05,"sleeping stops residual drift: max |v|=%.6f",maxSpeed);

}

// Domino run: the nudge has to travel down the line. The claim is causality, not a
// picture - the far end of a 29 m row cannot know about the first plank in under a
// second, and with the nudge removed nothing may move at all.
static void testDominoCascade() {
    std::puts("[domino cascade travels the row]");
    World w=makeDominoRun();
    const std::size_t planks=w.bodies.size()-1;
    const auto standing=[&]{
        std::size_t up=0;
        for(std::size_t i=1;i<w.bodies.size();++i)
            if(std::abs(w.bodies[i].angle)<0.3)++up;
        return up;};
    const auto advance=[&](int frames){for(int f=0;f<frames;++f)w.step(1.0/60);};
    const std::size_t start=standing();
    advance(60);
    const std::size_t oneSecond=standing();
    const real farAngleAtOneSecond=std::abs(w.bodies.back().angle);
    advance(120);
    const std::size_t threeSeconds=standing();
    advance(180);
    const std::size_t sixSeconds=standing();
    advance(360);
    const std::size_t twelveSeconds=standing();
    CHECK(start==planks,"every plank starts upright: %zu/%zu",start,planks);
    CHECK(farAngleAtOneSecond<0.05,"the far plank cannot know about the nudge yet: |angle|=%.6f rad",farAngleAtOneSecond);
    CHECK(threeSeconds<=oneSecond&&sixSeconds<threeSeconds&&twelveSeconds==0,
        "the wave propagates one plank at a time: standing %zu -> %zu -> %zu -> %zu",
        oneSecond,threeSeconds,sixSeconds,twelveSeconds);
    CHECK(std::abs(w.bodies.back().angle)>1.2,"the last plank falls flat: |angle|=%.4f rad",std::abs(w.bodies.back().angle));
    // Same scene, nudge removed: nothing may move, so the cascade is caused by the
    // impulse and not by an unstable start.
    World idle=makeDominoRun();
    idle.bodies[1].angularVelocity=0;
    real worst=0;
    for(int f=0;f<600;++f){idle.step(1.0/60);
        for(std::size_t i=1;i<idle.bodies.size();++i)worst=std::max(worst,std::abs(idle.bodies[i].angle));}
    CHECK(worst<1e-6,"without the nudge the row is inert for 10 s: max |angle|=%.9f rad",worst);
}

// The CCD showcase scene must actually stop its bullet: the claim is that the
// bullet reaches the wall and comes to rest without passing through it.
static void testBulletScene() {
    std::puts("[bullet vs wall showcase scene]");
    World w=makeBulletScene();
    const Vec2 wallStart=w.bodies[0].position;
    const real startX=w.bodies[1].position.x;
    w.step(1.0/60);
    CHECK(w.bodies[1].position.x>startX+5,"the bullet closes 15 m of distance in one step: x=%.4f -> %.6f",
        startX,w.bodies[1].position.x);
    CHECK(w.bodies[1].position.x<0,"the bullet stops before the wall instead of appearing behind it: x=%.9f",w.bodies[1].position.x);
    CHECK(w.bodies[1].velocity.length()<1e-6&&std::abs(w.bodies[1].angularVelocity)<1e-6,
        "the bullet comes to rest at the wall: |v|=%.9f |w|=%.9f",
        w.bodies[1].velocity.length(),w.bodies[1].angularVelocity);
    CHECK(!w.ccdStats.capped&&w.ccdStats.events>0,"the impact is a resolved event, not a capped budget: events=%d capped=%d",
        w.ccdStats.events,int(w.ccdStats.capped));
    for(int f=0;f<600;++f)w.step(1.0/60);
    CHECK(w.bodies[1].position.x<0&&(w.bodies[0].position-wallStart).length()==0,
        "the resting bullet never creeps through the wall and never moves the wall: x=%.9f",w.bodies[1].position.x);
}

// Ragdoll: revolute limits hold, the limbs settle on the floor, and nothing
// hovers. The joints are offset from the limb centroids, so this is also the
// project's worst case for warm-started joint error.
static void testRagdoll() {
    std::puts("[ragdoll limits and settling]");
    World w=makeRagdoll();
    real worstAngle=0,worstError=0,maxSpeed=0;
    for(int f=0;f<900;++f) {
        w.step(1.0/60);
        for(const auto& j:w.joints) {
            if(j.limits)worstAngle=std::max(worstAngle,std::abs(w.bodies[j.b].angle-w.bodies[j.a].angle));
            worstError=std::max(worstError,(w.bodies[j.b].position+w.bodies[j.b].rot*j.localB
                -w.bodies[j.a].position-w.bodies[j.a].rot*j.localA).length());
        }
    }
    for(std::size_t i=1;i<w.bodies.size();++i)maxSpeed=std::max(maxSpeed,w.bodies[i].velocity.length());
    real lowest=1e30;
    for(std::size_t i=1;i<w.bodies.size();++i) {
        const RigidBody& b=w.bodies[i];
        real bottom=b.position.y;
        if(b.shape.type==ShapeType::Circle)bottom-=b.shape.radius;
        else for(const auto& v:b.shape.verts)bottom=std::min(bottom,b.position.y+(b.rot*v).y);
        lowest=std::min(lowest,bottom);
    }
    CHECK(worstAngle<=1.51,"revolute limits hold: worst relative angle=%.4f rad (limit 1.5)",worstAngle);
    CHECK(maxSpeed<0.01,"the ragdoll settles to rest: max |v|=%.6f m/s",maxSpeed);
    CHECK(std::abs(lowest)<0.02,"the ragdoll rests on the floor instead of hovering: lowest point y=%.5f",lowest);
    CHECK(w.bodies[2].position.y>w.bodies[1].position.y,"the head stays above the torso after landing: %.3f > %.3f",
        w.bodies[2].position.y,w.bodies[1].position.y);
    std::printf("  info: ragdoll anchor error %.6f m (worst warm-started joint in the project)\n",worstError);
}

int main() {
    testMassProperties();
    testSymplecticEnergy();
    testRestingBox();
    testBounce();
    testFrictionSlope();
    testStack();
    testAABBs();
    testDynamicTree();
    testBroadphaseOracle();
    testWorldOracle();
    testWarmStarting();
    testTallScenes();
    testJoints();
    testStabilityUpgrades();
    testCCD();
    testIntegratorPolicies();
    testFuzz();
    testShowcaseSmoke();
    testMomentumConservation();
    testPendulumCradle();
    testDominoCascade();
    testRagdoll();
    testBulletScene();
    testMotorizedCar();
    testRubeGoldbergCausality();
    testWreckingPyramid();
    testStressSleeping();
    testBridgeProjection();
    std::printf("\n%s (%d failed)\n", g_failed ? "SOME TESTS FAILED" : "ALL TESTS PASSED", g_failed);
    return g_failed ? 1 : 0;
}
