# Applications of phys2d

This document explains what phys2d can genuinely be used for today, where it falls short of production quality, and how to present it honestly to judges, professors, or recruiters. Every capability and limitation statement is checkable against the headers named in each section; every number quoted below is copied from the recorded suite run in [build/final_tests.log](../build/final_tests.log) or from a CSV under `build/`, and the command that produces it is given so it can be re-measured on the presentation machine.

## 1. What this engine is

phys2d is a hand-written **C++20 2D rigid-body engine**. The core is **header-only and dependency-free**: linear algebra, the integrator, manifold generation, the constraint solver, the broadphase, and the continuous-collision machinery are all implemented from first principles in this repository. No physics library (Box2D, Chipmunk, Bullet, Matter.js) is used. SFML is used **only** for the optional desktop demo renderer, never inside the core.

Conventions are fixed and documented in the guide: right-handed coordinates with **y-up** and **CCW-positive** angles, polygon vertices stored in **local space, CCW with the centroid at the origin**, body `position` meaning the **centre of mass**, and every contact normal pointing **A → B**. The scalar type is `phys::real`, currently `double`. Time is a **fixed 1/60 s step**; the demo uses an accumulator and never feeds a variable `dt` to the solver.

Implemented features:

- **Shapes and mass properties.** `Circle` and convex `Polygon`, created through `RigidBody::makeCircle`, `makeBox`, and `makePolygon`. Polygon area, centroid, and inertia come from a triangle-fan derivation with the parallel-axis theorem applied to re-centre on the centroid.
- **Integration.** Semi-implicit (symplectic) Euler, split into `integrateForces` / `integrateVelocities` so the solver runs between them. Static bodies are represented by zero inverse mass and inverse inertia so no impulse formula needs a special case.
- **Narrowphase.** SAT for polygon-polygon with Sutherland-Hodgman clipping (1–2 contact points), Voronoi-region handling for polygon-circle, and a direct circle-circle test. Manifold normals are always A → B.
- **Solver.** Sequential impulses (Gauss-Seidel on the contact LCP) with **accumulated clamping**, **Coulomb friction** box-clamped to the friction cone, Baumgarte stabilization by default, **feature-ID warm starting** across frames, and a **two-point block solver** for two-point manifolds.
- **Broadphase.** A balanced **dynamic AABB tree** with fat proxies (margin 0.1), perimeter-cost descent, AVL-style height rotations, proxy removal and free-list reuse. The original O(N²) all-pairs path is retained as a test oracle.
- **Position correction.** Opt-in **nonlinear Gauss-Seidel (NGS)** contact and rigid-joint pose projection, replacing Baumgarte velocity bias when enabled.
- **Islands and sleeping.** Union-find islands that sleep and wake together, without merging through static grounds.
- **Joints.** Distance (rigid and **soft** via frequency/damping ratio), revolute with **angle limits and a torque-limited motor**, **mouse spring** for dragging, plus basic **weld** and **prismatic** constraints. Joints and contacts are solved in the same iteration loop, joints first.
- **Continuous collision.** Opt-in `RigidBody::bullet` with conservative advancement using translational and angular speed bounds, earliest-impact event integration, and impacts resolved as one-sided centre-of-mass impulses, so a bullet stops instead of being spun by an approximate lever arm. Per-bullet fallbacks (motion clamp, positional safety net) are reported through `World::ccdStats`.
- **Integrator study.** Standalone smooth-force policies for symplectic Euler, Velocity Verlet, and RK4 with measured convergence and energy plots.
- **Instrumentation.** `World::profiling` exposes broadphase, narrowphase, solve, and integrate millisecond buckets; `World::ccdStats` reports cast, event, and cap counts.

## 2. What it cannot do yet

These are real, current limits. Anything built on phys2d should be scoped around them:

- **Only convex shapes.** Concave shapes must be decomposed by the caller. There is no mesh, no compound body type, no edge/chain shape.
- **No collision layer filtering.** Joints expose `collideConnected`, and static-static pairs are skipped, but there is no general category/mask system.
- **No serialization.** No scene save/load format, no replay file format for external tools.
- **No stable body handles.** Manifolds hold raw `RigidBody*`; body indices are used by joints and the tree. Do not add or erase bodies mid-step, and rebuilding the body vector invalidates joints.
- **Single-threaded.** No parallel island solving. Determinism is verified for a fixed seed on a single thread; there is no guarantee across threads.
- **Sleeping does not remove all work.** Sleeping reuses frozen manifolds and skips inactive solves/corrections, but broadphase, island and global snapshot/cache work remains.
- **Continuous collision is still narrow.** Circles and nonspinning tilted polygons at 200/1k/10k m/s stop/rest before a 0.02 m static wall, with idle clamp/push-out/unconverged counters in the suite. A separate spinning-plate case stops translation while retaining 60 rad/s. [ccd.csv](../build/ccd.csv) reports zero event caps, not the other fallback counters. Swept impulses include translational Coulomb-clamped friction at the centre of mass, but omit off-center torque, joint re-solving and joint-connected exclusions; no swept-tree acceleration or validated jointed CCD scene exists.
- **Long chains stretch.** The 100-plank bridge reaches a 0.010674 m maximum anchor error after joint projection, but the unprojected 100-link benchmark still reports a 3.021928 maximum error at 40 sweeps. Tall kinematic chains are not ship quality.
- **No sanitizer execution.** ASan/UBSan are configured in [CI](../.github/workflows/ci.yml) but have not run: the local toolchain lacks `-lasan` / `-lubsan`, so debug-mode results must not be read as a sanitizer pass.
- **The browser build is a demo, not a product.** `bash web/build.sh` produces nine required files that must stay together: `index.html`, `index.js`, `index.wasm`, `showcase.css` and the five frontend scripts. Responsive desktop/narrow layout and pointer controls have Chromium evidence; the page is hosted and was verified running at <https://site-22be3251e8494dca85d1edd16f3d6ae8.freebuff.page>, while physical touch and other engines remain unverified. Serve over HTTP, not `file://`. See [SHOWCASE](SHOWCASE.md) for setup and controls and [DEPLOY](DEPLOY.md) for the hosting route.

## 3. Applications

### 2D games and platformers

A 2D rigid-body engine underpins platformers, top-down shooters, puzzle games, pinball, and sandbox titles: character capsules against terrain, crates and barrels that topple, ropes and swinging hazards, vehicles that roll over uneven ground, and debris that must settle rather than jitter. phys2d covers the core loop for this: SAT-based contact generation, sequential impulses with friction, warm-started stacking, and a dynamic AABB tree that keeps pair generation cheap as object counts grow.

What works today: the tested small/medium scene factories, resting-stack bounds and fixed 1/60 s stepping. That step size is not a measured rendered 60 Hz guarantee. What production would need: collision layers for projectiles and triggers, one-way platforms, a stable body handle system so content can be spawned and destroyed at any time, and serialization for level and save data.

Embedding it is two lines, because the core is headers and nothing else:

```bash
g++ -std=c++20 -O2 -I. examples/embed/game.cpp -o build/embed_game
./build/embed_game   # steps=240 crates=12 contacts=13 y=[-0.500, 2.477] player_x=1.842
```

[`examples/embed/game.cpp`](../examples/embed/game.cpp) is the smallest honest integration: the game owns the clock, the input and the spawning, and the engine owns only the simulation. It builds with one include path on this host (g++ 13.2.0, no warnings under `-Wall -Wextra -Wpedantic`), links nothing, and prints that line byte for byte on a single thread. A renderer would read `world.bodies` after each step, which is exactly what the browser build does. What a shipped game still needs is the list above — stable handles and serialization first, because content streaming is what breaks the index-based joint and manifold model.

### Sandbox and creative tools

Creative sandboxes (build-a-machine toys, demolition toys, procedural playgrounds) need spawning, dragging, and destruction at runtime. phys2d already supports the interaction primitives: `RigidBody::applyForce` for a force applied at a point, `Joint::mouse` for a spring-drag, and runtime `World::add` for spawning new bodies between steps. SFML uses left-drag/right-click spawning; the browser uses primary-pointer dragging and explicit + Box/+ Circle tools.

What works today: interactive spawning and dragging, soft and rigid constraints between parts, and cheap rebuilds by constructing a fresh `World`. What production would need: body removal that updates joints and proxies safely, undo/redo, a scene file format, and undo-safe impulse cache invalidation across structural edits.

### Educational physics software

This is the strongest fit for phys2d. Because the derivations live in the source next to the code — effective mass in [Solver.hpp](../Solver.hpp), SAT and clipping in [Collision.hpp](../Collision.hpp), the SAH cost model in [DynamicTree.hpp](../DynamicTree.hpp) — the engine doubles as teaching material. Measured behaviour can be compared against closed-form results: a small-angle pendulum period of 2.8333 s against the theoretical 2π√(L/g) ≈ 2.837 s, and a moving-circle time of impact of 0.15 s against the analytic root.

The integrator study is directly reusable for coursework: step-halving convergence ratios of 2.0403 / 4.0031 / 16.1013 confirm first-, second-, and fourth-order behaviour for symplectic Euler, Velocity Verlet, and RK4, and the measured oscillator energy shows a bounded symplectic range [47.619047619, 52.631578938] against RK4's final 49.930690443 with secular decay. Baumgarte versus NGS is also measurable: a penetrated zero-gravity box adds 2.599200 J with Baumgarte and 0 J with NGS.

The browser now supplies non-programmer controls and experiment explanations. A teaching product would still need guided quantitative experiments and in-browser plot export; the standalone energy CSV/SVG tools are a starting point.

### Robotics and 2D manipulator simulation

Planar robot arms, grippers, and walking mechanisms are naturally modelled as rigid links connected by revolute joints with limits and motors — exactly the joint set phys2d implements. A chain of links with torque-limited motors can approximate a planar manipulator, and `Joint::revolute` supports both angle limits (`limits`, `lower`, `upper`) and velocity-target motors (`motor`, `motorSpeed`, `maxMotorTorque`).

What works today: forward simulation of joint-space trajectories, motor speed tracking (a motor reaches a prescribed 2 rad/s and respects a 0.3 rad limit in tests), and hanging or pushed chains. What production would need: a general kinematic/dynamic chain abstraction, inverse dynamics or trajectory controllers, sensor readout APIs, and accurate joint friction and compliance. Long-chain drift also needs addressing: the 10-link chain reaches a 0.000035 final anchor error, but a 100-link chain does not.

### Vehicle and suspension studies

Wheeled vehicle behaviour — spring compression, load transfer, traction limits, and rollover thresholds — is a classic 2D rigid-body problem. phys2d offers soft distance joints (frequency and damping ratio) as springs, circle wheels with tunable friction, and revolute joints with motors as drivetrains, all solved in one iteration loop with contacts.

What works today: a chassis on spring struts with circular wheels over gently varying terrain is a scene factory, and the wheels drive themselves from a persistent `driveTorque` through a prismatic suspension axle with `lockAngle = false`, so travel and spin are independent. The headless suite asserts the outcome: 48.90 m travelled, max tilt 0.322 rad finishing level at 0.0038 rad, suspension travel 0.0082..0.7153 m against a 0.70 m rest length, and wheels turning 2.63 rad with a peak 63.32 rad/s. What is missing: the drive is a constant torque rather than a motor with a curve, and there is no braking, traction limit or per-surface friction model. Production would need those, plus load-transfer and rollover measurement.

### Procedural animation

Ragdolls, tails, tentacles, hanging cables, and swinging props are all constraint chains. phys2d's revolute joints with angle limits build a ragdoll (neck, shoulders, hips with documented limits), soft distance joints build springy attachments, and the mouse joint throws a body at a target point. Islands let a settled ragdoll stop consuming solve time.

What works today: finite, non-exploding ragdoll and chain scenes with documented joint drift, plus interactive dragging. What production would need: joint stiffness that survives large mass ratios, a muscle/motor-drive layer for animation control, collision filtering between connected limbs, and broader visual acceptance. The native renderer interpolates rotation/translation; the browser draws current fixed-step poses.

### Mechanism and linkage design

Four-bar linkages, cams, gears, ratchets, and escapements are planar mechanisms described almost entirely by revolute joints, angle limits, and motors. phys2d provides those primitives, so a designer can assemble a linkage and watch it move, and can use `World::profiling` to check that the mechanism stays within a real-time budget.

What works today: joint limits and motors let a driven wheel turn a linkage or a conveyor, and contacts let parts engage. What production would need: gear ratios as first-class constraints, a general torque/velocity source with a power budget, backlash and stiction, and analysis tools (joint reaction forces, mechanical advantage, efficiency).

### Structural studies

Stacked boxes, pyramids, arches, bridges, and trusses are standard stress cases for a constraint solver. phys2d reports these explicitly: at 30 sweeps with warm starting and two-point block solving, a 20-box stack holds late speed and drift below 0.000001 with a top box at 19.3878 against an ideal 19.5, and a 15-row pyramid shows 0.00244 late drift with a top at 14.3791 against 14.5. The 100-plank suspension bridge reaches a 0.010674 m maximum anchor error with joint position projection, against a documented 0.361208 m before.

The comparison tooling is part of the deliverable: cold-start scalar solving collapses the stack, scalar-only warm starting sways it (late speed 0.506678, drift 0.492979), and warm+block stabilizes it. Those failures are retained deliberately as evidence rather than hidden.

What production would need: yield or fracture modelling for material failure, accurate mass and stiffness tuning per part, and stress readout rather than positional error proxy.

### Reinforcement-learning environments

A physics engine can be the simulator inside a control-learning loop: an agent applies forces or joint torques, receives state, and learns a policy. phys2d is a plausible candidate because the step is deterministic for a fixed seed (a 32-circle 2,000-step elastic fuzz reproduces bit-identical trajectories) and because stepping is cheap for small scenes.

What works today: deterministic single-threaded stepping, direct force and torque application, and observability of all body state. What production would need: a vectorized or batched stepping mode, multiprocess or multithreaded environment rollouts, float precision for SIMD, exposed observation/action interfaces, and strict reset semantics. No training run has been performed with this engine, and no RL integration exists yet.

### Browser and WebAssembly interactive demos

Shipping the same C++ core to a browser makes the engine linkable from a résumé and shareable with anyone without a build step. phys2d's core is header-only and dependency-free, which is exactly what a small Emscripten target wants, and sources for it exist: an Emscripten entrypoint, an HTML canvas shell, and a build script.

What works today: nine sibling browser assets built from the same core, responsive scene navigation, per-scene framing/camera inverse, explanations and measured-claim boundaries, plus a live telemetry dashboard and a tuning panel that writes to the running solver. Local Chromium receipts contain 221 interaction, 50 framing and 87 dashboard assertions; [SHOWCASE](SHOWCASE.md) owns their scope and reproduction setup. It is hosted at <https://site-22be3251e8494dca85d1edd16f3d6ae8.freebuff.page> ([DEPLOY](DEPLOY.md) records the deployment and its verification). What remains: portable browser CI, physical touch/other engines and release packaging. The `.wasm` must be served over HTTP.

## 4. Presenting this to judges, professors, or recruiters

### What is genuinely impressive

- The solver is **derived, not copied**. Effective mass, accumulated clamping, the friction cone, the two-point LCP enumeration, joint Jacobians, and conservative advancement are implemented from first principles with the derivation in comments.
- **Correctness is engineered, not asserted.** The broadphase is validated against an all-pairs oracle, including bit-identical 600-step world trajectories, and the tree's own invariants (parent links, exact unions, heights, balance, reachability) are asserted after adversarial mutation.
- **Measured claims replace hand-waving.** Broadphase medians from 100 to 10,000 bodies, a 20-second 20-stack and 15-row pyramid acceptance, convergence ratios for three integrators, and a Baumgarte-versus-NGS energy comparison all come with reproducible commands.
- **Failures are documented.** The comparison tables keep scalar-only warm starting failing the stack test and the 100-link chain's 3.021928 error, and the dense 10k full step is recorded at 177.5652 ms, far over the 16.67 ms budget.
- **143 checks** pass warning-clean with `-Wall -Wextra -Wpedantic -Werror` on GCC 13.2.0, in the saved optimized receipt. Eight default CMake targets build (nine with SFML); CTest registers one engine executable. The older checked-container debug receipt covers 76 assertions, not this whole revision.

### What to say honestly

Say that this is a regression-tested educational core, not a shipped physics framework. Swept impacts use translational friction but no off-center torque or joint re-solving; general CCD remains unproven. Sanitizers are configured, not executed. The 5,000-box field has stacking contacts, then sleeps; its own early-versus-settled timing is not a controlled sleep-on/off comparison or a rendered frame rate. Sustained active load is now measured separately (5,000 awake, 5,000 stirred, per-stage split, median/tail, host drift reported) in [BENCHMARKS](BENCHMARKS.md#active-interaction-load-5000-bodies-awake), which owns the timing tables and host caveats. Rendered frame rate remains unmeasured. Naming these limits first makes the evidence credible.

### Three demo flows to show

1. **Stability and iteration budget.** Run the stack and pyramid acceptance, then the warm/cold and scalar/block comparison table. This shows that a solver's quality is about accumulated impulse handling, not iteration count, and it ends on a measured before/after.
2. **Broadphase scaling.** Run the benchmark across 100, 1,000, and 10,000 bodies with the all-pairs oracle enabled and show that candidate and contact counts still match. This demonstrates both performance engineering and correctness under mutation.
3. **Continuous collision and joints.** Show one circle and one thin polygon hitting a 0.02 m wall at 1,000 m/s - both stop dead and stay stopped - then the 100-plank bridge and the ragdoll, and state plainly that swept torque/joint handling remains incomplete, despite translational impact friction being implemented.

## 5. Roadmap priorities

Car drive and triggered-machine causality have native assertions. The browser builds/runs with responsive presentation, per-scene framing and contact overlays. Native SFML additionally draws AABBs, joint anchors and a separate visualization tree, and interpolates poses. Do not conflate frontend features or browser checks with headless physics tests.

Ordered by what most limits usefulness now:

1. **Finish CCD acceptance.** Correct off-center torque, connected-body filtering and joint-aware impacts, with focused tests; swept-tree acceleration remains a performance task.
2. **Profiling-led optimization.** Measure representative active interaction first. Investigate cache allocation, layouts, precision or parallel islands only when measurements justify them.
3. **Sanitizer CI execution.** Get ASan/UBSan running in CI and fix whatever they surface; debug-mode runs are not a substitute.
4. **Delivery and frontend coverage.** Portable browser CI, other engines/touch, complete native mouse checks and a clean release/hosting path. Optional extra overlays are not completed acceptance.
5. **Framework work.** Collision filtering, serialization, concave decomposition and a stable body handle system - the work of turning a validated engine into a framework.
