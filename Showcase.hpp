#pragma once
// ============================================================================
// Showcase scenes.
//
// Every scene is a plain `World` factory, so the same definition drives the
// headless tests, the benchmarks and the SFML demo. That is deliberate: a scene
// that only exists in the renderer cannot be regression tested.
//
// Scenes that carry an acceptance claim (conservation, causality, suspension
// travel) are asserted in test_engine.cpp.
// ============================================================================
#include "Scenes.hpp"
#include <string_view>

namespace phys {

inline constexpr int kSceneCount = 10;
inline constexpr const char* sceneNames[kSceneCount] = {
    "Newton's cradle", "Domino run", "Wrecking pyramid", "Ragdoll",
    "Suspension bridge", "Motorized car", "Rube-Goldberg machine",
    "Stress: 5000 boxes", "Bullet vs wall (CCD)", "Momentum row (cradle test)"
};

// ---------------------------------------------------------------------------
// Newton's cradle: five balls on equal-length distance constraints, equal mass,
// perfectly elastic, frictionless. The classic elastic-collision chain: the ball
// chain transmits momentum one link at a time, so the far ball leaves with the
// incoming speed while the near balls stay put.
// ---------------------------------------------------------------------------
inline World makeCradle(int balls = 5, bool liftFirstBall = true) {
    World w;
    w.solver.config().iterations = 40;
    const auto pivot = w.add(RigidBody::makeCircle(0.05, 0, {0, 3}));
    for (int i = 0; i < balls; ++i) {
        const real x = (real(i) - real(balls - 1) / 2) * 1.0;
        auto ball = RigidBody::makeCircle(0.5, 1, {x, 2.0});
        ball.restitution = 1;
        ball.friction = 0;
        const auto index = w.add(ball);
        // Anchor above the ball, one radius away: rest length 1.0 for every ball.
        w.joints.push_back(Joint::distance(pivot, index, {x, 0}, {}, 1.0));
    }
    if (liftFirstBall) {
        // Raise the left ball 40 degrees and release it from rest.
        const real angle = 0.7;
        w.bodies[1].position = {-(real(balls - 1) / 2) - std::sin(angle), 3 - std::cos(angle)};
    }
    return w;
}

// ---------------------------------------------------------------------------
// Momentum transfer without constraints: identical frictionless balls in a row
// on a zero-gravity plane. Total linear momentum is a conserved quantity here,
// which makes it a strict regression test rather than a visual claim.
// ---------------------------------------------------------------------------
inline World makeMomentumRow(int balls = 5, real speed = 5, real gap = 0.01) {
    World w;
    w.gravity = {};
    w.solver.config().iterations = 30;
    for (int i = 0; i < balls; ++i) {
        auto ball = RigidBody::makeCircle(0.5, 1, {real(i) * (1.0 + gap), 0});
        ball.restitution = 1;
        ball.friction = 0;
        w.add(ball);
    }
    w.bodies[0].position = {-2, 0};
    w.bodies[0].velocity = {speed, 0};
    return w;
}

// ---------------------------------------------------------------------------
// Domino run.
// ---------------------------------------------------------------------------
inline World makeDominoRun(int count = 45) {
    World w;
    w.solver.config().iterations = 20;
    w.add(RigidBody::makeBox(120, 1, 0, {0, -0.5}));
    for (int i = 0; i < count; ++i)
        w.add(RigidBody::makeBox(0.18, 1.4, 1, {-18 + 0.65 * i, 0.7}));
    w.bodies[1].angularVelocity = -2;   // the nudge that starts the reaction
    return w;
}

// ---------------------------------------------------------------------------
// Twenty-row pyramid and a wrecking ball on a rope.
// ---------------------------------------------------------------------------
// The ball hangs 19 m from an anchor above the pyramid's left flank and is
// released at 45 degrees, so its lowest point (-6, 5) passes straight through
// row 4 of the pile rather than swinging in free space beside it.
inline World makeWreckingPyramid(int rows = 20) {
    World w = makePyramid(rows);
    w.solver.config().iterations = 40;
    w.solver.config().positionCorrection = true;
    const real rope = 19.0, angle = 0.7853981633974483;   // 45 degrees
    const Vec2 anchorPoint{-6, 24};
    const Vec2 release{anchorPoint.x - rope * std::sin(angle), anchorPoint.y - rope * std::cos(angle)};
    const auto anchor = w.add(RigidBody::makeCircle(0.1, 0, anchorPoint));
    auto ball = RigidBody::makeCircle(1.2, 25, release);
    ball.restitution = 0;
    const auto body = w.add(ball);
    w.joints.push_back(Joint::distance(anchor, body, {}, {}, rope));
    return w;
}

// ---------------------------------------------------------------------------
// Ragdoll: revolute joints with angle limits, draggable with the mouse joint.
// ---------------------------------------------------------------------------
inline World makeRagdoll() {
    World w;
    w.solver.config().iterations = 20;
    w.solver.config().positionCorrection = true;
    w.add(RigidBody::makeBox(120, 1, 0, {0, -0.5}));
    const auto torso = w.add(RigidBody::makeBox(0.9, 1.4, 3, {0, 6}));
    const auto head = w.add(RigidBody::makeCircle(0.4, 1, {0, 7.2}));
    auto neck = Joint::revolute(torso, head, {0, 0.7}, {0, -0.5});
    neck.limits = true; neck.lower = -0.5; neck.upper = 0.5;
    w.joints.push_back(neck);
    for (real side : {real(-1), real(1)}) {
        const auto arm = w.add(RigidBody::makeBox(1, 0.25, 1, {side * 0.95, 6.5}));
        auto shoulder = Joint::revolute(torso, arm, {side * 0.45, 0.5}, {-side * 0.5, 0});
        shoulder.limits = true; shoulder.lower = -1.5; shoulder.upper = 1.5;
        w.joints.push_back(shoulder);
        const auto leg = w.add(RigidBody::makeBox(0.3, 1.4, 1, {side * 0.25, 4.6}));
        auto hip = Joint::revolute(torso, leg, {side * 0.25, -0.7}, {0, 0.7});
        hip.limits = true; hip.lower = -0.8; hip.upper = 0.8;
        w.joints.push_back(hip);
    }
    return w;
}

// ---------------------------------------------------------------------------
// Suspension bridge: 100 planks pinned end to end between two static anchors.
// Solved with rigid-limit position projection so the deck does not sag open.
// ---------------------------------------------------------------------------
inline World makeBridge(int planks = 100) {
    World w;
    w.solver.config().iterations = 100;
    w.solver.config().positionIterations = 60;
    w.solver.config().positionCorrection = true;
    const auto left = w.add(RigidBody::makeCircle(0.1, 0, {-20, 5}));
    std::size_t previous = left;
    for (int i = 0; i < planks; ++i) {
        const auto body = w.add(RigidBody::makeBox(0.4, 0.15, 1, {-19.8 + 0.4 * i, 5}));
        w.joints.push_back(Joint::revolute(previous, body,
            i == 0 ? Vec2{} : Vec2{0.2, 0}, {-0.2, 0}));
        previous = body;
    }
    const auto right = w.add(RigidBody::makeCircle(0.1, 0, {20, 5}));
    w.joints.push_back(Joint::revolute(previous, right, {0.2, 0}, {}));
    return w;
}

// ---------------------------------------------------------------------------
// Motorized car with soft-spring suspension.
//
// Each wheel is mounted by a PRISMATIC joint along the chassis-local y axis with
// `lockAngle = false`: the wheel is constrained to slide vertically and stays free
// to spin. A soft distance joint provides the spring, and `driveTorque` turns the
// wheel. That is the standard 2D wheel-mount construction; a revolute joint could
// not be used here because it would pin the wheel to a fixed point (no travel).
// ---------------------------------------------------------------------------
struct CarRig { World world; std::size_t chassis = 0; std::size_t wheelLeft = 0, wheelRight = 0; };

inline CarRig makeCarRig(real drive = -5, real springFrequency = 2.5) {
    CarRig rig;
    World& w = rig.world;
    w.solver.config().iterations = 20;
    w.solver.config().positionCorrection = true;
    w.solver.config().positionIterations = 20;
    w.add(RigidBody::makeBox(300, 1, 0, {0, -0.5}));
    // Gentle terrain: low bumps and one small step, not launch ramps. The scene
    // is about suspension travel and wheel torque, not jumps.
    w.add(RigidBody::makeBox(4, 0.3, 0, {18, 0.15}, 0.05));
    w.add(RigidBody::makeBox(4, 0.3, 0, {30, 0.15}, -0.05));
    w.add(RigidBody::makeBox(3, 0.6, 0, {44, 0.3}));
    rig.chassis = w.add(RigidBody::makeBox(3.2, 0.5, 6, {-8, 1.4}));
    w.bodies[rig.chassis].friction = 0.6;
    int side_index = 0;
    for (real side : {real(-1), real(1)}) {
        auto tire = RigidBody::makeCircle(0.45, 1.0, {-8 + side * 1.15, 0.55});
        tire.friction = 1.5;
        const auto wheel = w.add(tire);
        // Prismatic axle: vertical travel only, and `lockAngle = false` keeps the
        // wheel free to spin. A revolute joint could not be used here, because it
        // would pin the wheel to a fixed point and remove all suspension travel.
        auto axle = Joint::prismatic(rig.chassis, wheel, {side * 1.15, -0.15}, {}, {0, 1});
        axle.lockAngle = false;
        w.joints.push_back(axle);
        // Soft spring: rest length 0.7, ~2.5 Hz, critically-ish damped.
        w.joints.push_back(Joint::distance(rig.chassis, wheel,
            {side * 1.15, -0.15}, {}, 0.7, springFrequency, 1.0));
        w.bodies[wheel].driveTorque = drive;
        if (side_index++ == 0) rig.wheelLeft = wheel; else rig.wheelRight = wheel;
    }
    return rig;
}

inline World makeCar(real drive = -5) { return makeCarRig(drive).world; }

// ---------------------------------------------------------------------------
// Rube-Goldberg machine with an explicit trigger switch.
//
// The switch exists so causality can be tested rather than asserted: with the
// trigger the final flag must fall, without it the machine must stay inert.
// ---------------------------------------------------------------------------
struct RubeMachine { World world; std::size_t flag = 0; std::size_t ball = 0; bool hasBall = false; };

inline RubeMachine makeRubeMachine(bool trigger = true) {
    RubeMachine machine;
    World& w = machine.world;
    w.solver.config().iterations = 25;
    w.solver.config().positionCorrection = true;
    w.solver.config().positionIterations = 20;
    w.add(RigidBody::makeBox(200, 1, 0, {0, -0.5}));
    // Stage 1: ramp sloping down to the right, ending just above the first domino.
    // A positive angle would raise the +x end and send the ball the wrong way.
    w.add(RigidBody::makeBox(5.2, 0.3, 0, {-12.5, 2.2}, -0.30));
    // Stage 2: dominoes.
    const real firstDomino = -9.4;
    for (int i = 0; i < 8; ++i)
        w.add(RigidBody::makeBox(0.18, 1.3, 1, {firstDomino + 0.62 * i, 0.65}));
    // Stage 3: weighted pendulum, knocked by the last domino.
    const real pivotX = firstDomino + 0.62 * 8 + 0.10;
    const auto pivot = w.add(RigidBody::makeCircle(0.08, 0, {pivotX, 3.1}));
    const auto hammer = w.add(RigidBody::makeBox(0.3, 2.0, 3, {pivotX, 2.1}));
    w.joints.push_back(Joint::distance(pivot, hammer, {}, {0, 1.0}, 1.0));
    // Stage 4: the flag, which only falls if the chain completes.
    machine.flag = w.add(RigidBody::makeBox(0.25, 1.7, 1, {pivotX + 1.15, 0.85}));
    if (trigger) {
        auto ball = RigidBody::makeCircle(0.2, 2, {-14.3, 3.35});
        ball.restitution = 0.05;
        machine.ball = w.add(ball);
        machine.hasBall = true;
    }
    return machine;
}

inline World makeRubeGoldberg(bool trigger = true) { return makeRubeMachine(trigger).world; }

// ---------------------------------------------------------------------------
// Stress scene: a dense grid of boxes that settle and then sleep.
// ---------------------------------------------------------------------------
// A wide, shallow field of boxes: many bodies for the broadphase to handle, but a
// shallow pile so it can actually come to rest. A tall 50-layer tower with a
// handful of solver iterations jitters forever and never sleeps.
inline World makeStress(int count = 5000, bool enableSleeping = true, int columns = 500) {
    World w;
    w.gravity = {0, -9.81};
    w.sleeping = enableSleeping;
    w.solver.config().iterations = 10;
    w.solver.config().positionCorrection = true;
    w.solver.config().positionIterations = 6;
    const real width = 0.55 * columns;
    for (int i = 0; i < count; ++i) {
        const real x = -0.5 * width + 0.55 * (i % columns);
        const real y = 0.3 + 0.55 * (i / columns);
        w.add(RigidBody::makeBox(0.5, 0.5, 1, {x, y}));
    }
    w.add(RigidBody::makeBox(width + 40, 1, 0, {0, -0.5}));
    return w;
}

// ---------------------------------------------------------------------------
// CCD demonstration: a fast thin bullet against a thin wall.
// ---------------------------------------------------------------------------
inline World makeBulletScene(real speed = 900) {
    World w;
    w.gravity = {};
    auto wall = RigidBody::makeBox(0.05, 12, 0, {0, 6});
    wall.restitution = 0;
    w.add(wall);
    auto bullet = RigidBody::makeBox(0.10, 0.03, 1, {-12, 6}, 0.12);
    bullet.bullet = true;
    bullet.restitution = 0;
    bullet.velocity = {speed, 0};
    w.add(bullet);
    return w;
}

inline World makeShowcase(int scene, bool trigger = true) {
    switch (scene) {
        case 0: return makeCradle();
        case 1: return makeDominoRun();
        case 2: return makeWreckingPyramid();
        case 3: return makeRagdoll();
        case 4: return makeBridge();
        case 5: return makeCar();
        case 6: return makeRubeGoldberg(trigger);
        case 7: return makeStress();
        case 8: return makeBulletScene();
        default: return makeMomentumRow();
    }
}

} // namespace phys
