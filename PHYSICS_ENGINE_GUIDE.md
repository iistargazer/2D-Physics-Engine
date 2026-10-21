# phys2d — Project guide

> Read this first for current conventions and status. This is the status/ownership ledger, not a duplicate of the learning guide or benchmark tables. All phases are **not** finished.

## 1. Mission and boundaries

Build a hand-written, header-only C++20 2D impulse-based rigid-body engine and an interview/judging showcase. No Box2D, Chipmunk, Bullet, Matter.js or other physics library. SFML is optional rendering only; Emscripten compiles the same headers for the browser.

The owner is a Mathematics & Computing undergraduate: explain intuition and derivations beside the implementation, with the detailed learning path in [FROM_SCRATCH](docs/FROM_SCRATCH.md). No required quizzes or owner implementation checkpoints. Preserve existing behavior; add focused tests for engine changes. Measure before claiming performance or adding optimizations.

Native build commands live in [README](README.md); browser/desktop operation lives in [SHOWCASE](docs/SHOWCASE.md). Commands assume the repository root and Bash/Git Bash. The checkout currently has no Git metadata; no diff/history comparison is available.

## 2. Verification ledger

| Surface | Actual evidence | What it does not establish |
|---|---|---|
| Optimized native suite | [final_tests.log](build/final_tests.log): 143 `ok:` assertions, zero failures; existing native executable rerun during audit also passed | Cross-platform or sanitizer safety; assertion count is not a feature-completeness score |
| CMake | [build log](build/cmake_build.log): eight default targets at that revision; current tree is fourteen targets, warnings-clean. [CTest receipt](build/temporal/ctest-active.log): five registered tests (`engine`, `ccd_allocations`, `world_storage`, `temporal`, `rows`) all passed | Remote CI or portable packaging; a target count is not a coverage score |
| Checked-container debug | [final_debug_tests.log](build/final_debug_tests.log): older 76-assertion revision passed | Current 142-assertion debug coverage; ASan/UBSan |
| Browser build | Emscripten 6.0.12 produced HTML, JS, WASM, CSS and showcase JS; syntax/build and copied-asset checks passed; the nine files are deployed and verified on a real origin ([DEPLOY](docs/DEPLOY.md)) | Clean-machine setup or every browser engine |
| Browser interaction/layout | [221-check receipt](build/browser-check/polish-results.log) and [50-check framing receipt](build/browser-check/framing-results.log): Chromium, widths 390–1500, DPR 1/2, all scenes, spawn/drag inverse, resize, cleanup, restart, follow camera; screenshots inspected | Physical touch devices; complete OS focus-loss behavior (blur handler partly tested with a dispatched event); rendered frame performance |
| Browser dashboard | [87-check receipt](build/browser-check/dashboard-results.log) and [driven measurement log](build/browser-check/dashboard-drive.log): telemetry tiles against the frame buffer, gauge arithmetic and over-budget state, opt-in stage split reconciled with the step, tuning controls changing the measured stage costs, wake-on-sleep-off, first-run persistence, recovery panel, reduced-motion suppression; dashboard screenshots at 1440/1024/390 inspected | One host and one browser; WASM excludes rendering; the stage split reads `BELOW CLOCK QUANTUM` on small scenes; the earlier driven-stress table did not reproduce (see SHOWCASE) |
| Active 5k interaction | [active.csv](build/active.csv) / [active.err](build/active.err): simulation-only per-stage split, median/p95/p99/max, three trials, 5,000 awake vs asleep vs stirred, plus a driven browser stress run (0 awake/13.3 ms to 5,020 awake/44.7 ms under continuous input) | Host-load dependent (calibration drifted +69.9% mid-run); WASM and native figures are not interchangeable; render cost and rendered FPS unmeasured; single host, one compiler |
| Native SFML | Local SFML 2.6.2 build, startup/scene/debug/pause/step and PNG generation exercised | Complete native mouse/visual acceptance; portable release bundle |
| Sanitizers | [CI configuration](.github/workflows/ci.yml) includes ASan/UBSan; local MinGW linking failed for missing runtimes | **No executed sanitizer or remote CI pass is recorded** |

Receipts under `build/` are local artifacts, not guaranteed to exist in a clean clone. Keep assertions and commands reproducible; curate evidence separately before release. The documentation-only pass did not rerun physics/browser checks. The subsequent mission prerequisite pass did: [initial results](docs/FEATURE_ACCEPTANCE.md#fresh-verification-receipts) record optimized success, debug/CTest sleeping-timing failures, allocation regression success, rebuilt WASM/221 Chromium checks and sanitizer linker blockers. The latest [storage receipts](docs/STORAGE.md#fresh-receipts-for-this-pass) record 142 behavioral assertions and all three CTests passing, with the retained sleeping-speed criterion executed separately and passing its five-trial benchmark. The active-interaction pass added [active.csv](build/active.csv) (per-stage split, median/tail, three trials, reported host calibration drift), [five passing CTests](build/temporal/ctest-active.log) and rebuilt WASM with [221 + 50 Chromium checks](build/browser-check/polish-results.log). The presentation pass split the frontend into five scripts, added a live telemetry dashboard and a live tuning panel with additive, default-off engine instrumentation, rebuilt the WASM, and added an [87-assertion dashboard suite](build/browser-check/dashboard-results.log) beside the unchanged 221 + 50. Host load moved the driven figures by more than 2x between two runs of that suite, so its assertions are relational (sleeping cheaper than awake, gauge equals its step, stage sum reconciles with the step) rather than absolute millisecond thresholds. Sanitizer execution remains blocked.

## 3. Fixed conventions

| Topic | Convention |
|---|---|
| Coordinates | Right-handed, y-up, angles CCW-positive; flip y only in renderers |
| Scalar | `phys::real = double`; no automatic float migration |
| Body origin | Centre of mass; convex polygon vertices local, CCW, centroid at origin |
| Static bodies | `invMass == 0 && invInertia == 0` |
| Contact normal | A → B; impulse −λn on A, +λn on B |
| Tangent | `(n.y, -n.x)` |
| Materials | Friction `sqrt(muA*muB)`; restitution `max(eA,eB)` |
| Rotation | Cached `rot = R(angle)`; call `setAngle()` |
| Time | Showcase fixed 1/60 s accumulator; never feed variable render dt to the solver |

`World::bodies` and joint indices are public; no stable generational handles exist. Do not add/erase during a step. Reallocation invalidates old manifold pointers; erasing/reordering requires rebuilding joints. Wake a sleeping body before editing its state/shape: cache/proxy shortcuts assume sleeping poses are unchanged.

## 4. Ownership and main path

| Owner | Responsibility |
|---|---|
| [Vec2](Vec2.hpp), [Mat22](Mat22.hpp), [RigidBody](RigidBody.hpp) | Math, convex shape/mass properties, state, forces/impulses |
| [Integrator](Integrator.hpp), [IntegratorStudy](IntegratorStudy.hpp) | Split symplectic Euler; separate smooth-force study policies |
| [AABB](AABB.hpp), [DynamicTree](DynamicTree.hpp), [Broadphase](Broadphase.hpp) | Bounds, fat proxies, balanced tree, deterministic pairs |
| [Collision](Collision.hpp), [Manifold](Manifold.hpp) | SAT/clipping/circle tests, feature IDs, contact data |
| [Solver](Solver.hpp), [Joint](Joint.hpp) | Accumulated impulses, block normals, friction, joint constraints/projection |
| [Islands](Islands.hpp), [Continuous](Continuous.hpp) | Sleep/wake graph; limited bullet event advancement |
| [World](World.hpp) | Step orchestration, cache validation/restoration/save, NGS, profiling |
| [Scenes](Scenes.hpp), [Showcase](Showcase.hpp) | Test-oriented and presentation factories |
| [test_engine](test_engine.cpp), root benchmark/study programs | Assertions, timing comparisons and integrator experiment |
| [native renderer](demo/main.cpp) | SFML input, fixed-step rendering/interpolation, overlays and capture |
| [web/main](web/main.cpp) | WASM world/control exports and flat frame buffer |
| [web/shell](web/shell.html), [showcase JS](web/showcase.js), [CSS](web/showcase.css) | Page structure; scene presentation/camera/input/rendering; styling |
| [web/build](web/build.sh) | Compile WASM and copy CSS/JS into `build/web/` |

Main step: validate cache/joint indices → build/wake islands on previous contacts → forces → pair/manifold generation → warm start/prepare → joint then contact sweeps → pose integration or CCD slices → optional NGS → sleep → save cache. Full explanation: [FROM_SCRATCH §4](docs/FROM_SCRATCH.md#4-one-step-stage-by-stage).

Documentation ownership:
- This guide: current status, conventions, ownership and remaining acceptance.
- [README](README.md): short entry point/build and reading path.
- [FROM_SCRATCH](docs/FROM_SCRATCH.md): learning/derivations/pipeline/debugging.
- [BENCHMARKS](docs/BENCHMARKS.md): measured tables and methodology; do not copy timing tables elsewhere.
- [SHOWCASE](docs/SHOWCASE.md): setup, controls, presenting and verification boundaries.
- [SCENES](docs/SCENES.md): factory/claim catalogue; [APPLICATIONS](docs/APPLICATIONS.md): practical fit and missing product capabilities.
- [STORAGE](docs/STORAGE.md): prepared capacity, ownership, layouts and failure contract.
- [TGS](docs/TGS.md): temporal/compliant contact rows, equations and measured limits.
- [PARALLEL](docs/PARALLEL.md): solver-row identity, deterministic colour schedule and the residual parallel blockers.

## 5. Current phase status

| Phase | State and acceptance boundary |
|---|---|
| Math, mass, integration | Tested analytic properties and bounded oscillator energy |
| Collision and impulses | Convex SAT/clipping/circle tests, Coulomb friction, restitution and accumulation tested |
| Broadphase | Dynamic tree, mutations/invariants, all-pairs oracle and deterministic trajectories tested; worst-case pair output remains quadratic |
| Warm start/block solve | Tested 20-second 20-stack/15-row pyramid bounds; cold/scalar failure configurations retained |
| NGS/sleeping | Contact and rigid-joint pose correction; island sleep/wake tested; broadphase/cache work remains while asleep |
| Joints | Distance/soft, revolute motor/limits, mouse, basic weld/prismatic tested; no prismatic translation motor/limits; unprojected 100-link chain error remains large |
| SFML showcase | Ten scenes, rotation/translation interpolation, title-bar feedback and overlays implemented; full native interaction/packaging acceptance incomplete |
| Performance | Benchmarks and phase timing exist; representative active-5k/rendered budget and profiling-led optimization remain unproven |
| CCD | Limited prototype with per-body nets; specific thin-wall cases pass, not general off-center/jointed collision fidelity |
| Browser showcase | Built and exercised, responsive presentation and per-scene framing, deployed and verified on a real origin; other engines/touch/clean-clone CI remain |

### CCD: precisely what is implemented

[Continuous.hpp](Continuous.hpp) uses conservative advancement (128 cast iterations, default 64 events). Normal and Coulomb-clamped tangential impulses are applied at the **centre of mass (`r=0`)**. This avoids fabricated spin but omits physically correct off-center swept impact torque; existing spin is not removed. Later discrete contacts have real lever arms, but that does not recover the missing instantaneous impact response.

Circle and nonspinning tilted-polygon tests at 200/1000/10000 m/s against a 0.02 m static wall assert rest/no crossing and idle clamp/push-out/unconverged counters. A separate spinning-plate test preserves 60 rad/s while stopping translation; it is not a zero-spin rest test at all three speeds. Analytic moving-circle TOI and unrelated-body continuation are also tested.

There is **no global freeze**. Event cap, per-bullet motion withholding and positional push-out are reported separately. `capped_trials=0` alone says nothing about `clamped`/`pushedOut`. Remaining: swept-tree acceleration, contact-point torque, joint re-solving, connected-body exclusions during casts, and broader moving/oblique/frictional impact acceptance.

### Stress: what the number means

The factory is 5,000 boxes in 500 columns/10 rows with stacking contacts, plus one static floor. [showcase.csv](build/showcase.csv) averages 600 steps including settling and sleeping, ending with zero awake dynamics. `testStressSleeping` times the first 30 steps, then 120 steps after a 600-step settling period. It is **not** a controlled sleeping-on/off comparison or sustained active-interaction benchmark. Its log denominator includes the static floor (5,000/5,001), although all 5,000 dynamic bodies sleep.

The [benchmark owner](docs/BENCHMARKS.md) records local costs and caveats. No active 5k-at-60-FPS or rendered frame-rate claim is supported. The inherited timing inequality now lives in the dedicated `benchmark_sleeping` Release executable, not debug behavioral CTest; it remains a failing exit criterion when unmet. Sleeping/waking correctness stays in the suite. See [prepared storage](docs/STORAGE.md) for ownership, capacity failures, layouts and reproduction.

## 6. Remaining acceptance and GitHub preparation

The expanded five-track mission is owned by [FEATURE_ACCEPTANCE](docs/FEATURE_ACCEPTANCE.md): equations, dependency order, per-track gates, allocation/layout contracts and frontend acceptance. These are requested outcomes, not completed features. The work is committed and pushed to https://github.com/iistargazer/2D-Physics-Engine (`main`, commit `56d5997`, clean tree, remote ref verified by `ls-remote`) and the browser lab is published at <https://site-22be3251e8494dca85d1edd16f3d6ae8.freebuff.page> — see [DEPLOY](docs/DEPLOY.md) for what was verified on that origin.

Available now: `timeOfImpact` borrows geometry and samples stack-only `SweepPose` values. [Prepared World stepping](docs/STORAGE.md) adds explicit body/joint/pair/contact/vertex capacity, allocation-free `tryStep` success/failure, flat contact/snapshot storage and owned broadphase/island/CCD scratch. Active-contact/joint and sleep/wake regressions cover whole steps, not just casts. Unprepared use still grows dynamically; the standalone CCD convenience overload still allocates scratch. Center-of-mass response and existing CCD limitations are unchanged.


Ordered unfinished outcomes, not a promise to implement every possible feature:

- [ ] Current-revision fresh optimized/debug verification and executed ASan/UBSan; resolve supported-host findings and publish actual receipts only after execution.
- [x] Representative sustained active interaction benchmark at 5k, correctness plus timing: [active.csv](build/active.csv) with a per-stage split (broadphase, narrowphase, islands, solver, integration, CCD) plus a bookkeeping residual, median/p95/p99/max frame costs, three trials per case and a reported host-load calibration drift. Cases: 5,000 awake, 5,000 asleep, 5,000 stirred, and eight showcase scenes held in motion; NGS projection is isolated by an adjacent projection-off twin. See [BENCHMARKS](docs/BENCHMARKS.md#active-interaction-load-5000-bodies-awake).
- [ ] Render cost separated from simulation cost at 5k, and rendered-FPS evidence: the interactive showcase reports a step EMA that excludes rendering, and no rendered frame timing exists.
- [ ] CCD acceptance for off-center/oblique and moving impacts, jointed scenes and connected-body filtering; retain honest prototype status until supported.
- [ ] Browser checks in reproducible CI, other browser engines and physical touch; native mouse/overlay/portable bundle acceptance.
- [x] Publish the browser build to a real origin. The nine assets are deployed through [deploy/](deploy) and verified running (HTML, CSS, JS and `application/wasm` all 200; the engine simulated hundreds of fixed steps with no console errors). Pages remains unconfigured.
- [ ] Repository organization and packaging without breaking public includes/builds; safe runtime removal/handles if framework use is claimed.
- [ ] Personal learning path is present; complete a clean-machine walkthrough and keep examples/links aligned as APIs change.

GitHub preparation (commit `56d5997` is published on `main`; the remaining items are unchecked):
- [x] Destination is user-selected: https://github.com/iistargazer/2D-Physics-Engine. Remote `main` was confirmed to point at the pushed commit; the license/provenance decision and contribution expectations are still open.
- [x] Add ignore rules for `build/`, SDK/dependencies, local client metadata, logs, probes and secrets. Curate receipts outside disposable output if they will be cited publicly — `.gitignore` keeps documented logs/CSVs/SVGs and the palette check while excluding toolchains, binaries and logs that nothing cites.
- [ ] Validate a clean source checkout on supported hosts; pin Emscripten/build inputs and install compatible SFML only for its optional target.
- [ ] Add browser build/test plus Pages artifact preparation. Nine assets, not five: `index.html`, `index.js`, `index.wasm`, `showcase.css` and the five frontend scripts. Deployment today copies exactly those nine ([deploy/make-dist.sh](deploy/make-dist.sh) fails on any other count) and was verified on a real origin; Pages subpath URLs are untested because CI still has **no Pages or browser job**.
- [x] Prepare release instructions and screenshots: [DEPLOY](docs/DEPLOY.md) owns hosting, [VISUAL](docs/VISUAL.md) owns the before/after captures. Portable native assets remain unchecked; do not copy the entire local `build/` tree.
- [x] Initialize/stage/commit/push/publish with explicit authorization and ownership review. A configuration file or local preview is not a deployment: the published origin was loaded in Chromium and the engine was observed stepping there.

## 7. Maintainer agreement

Preserve conventions and dependency-free headers. Make focused changes at the owners above, document why, run affected checks after the final edit, and keep process exit status when filtering output. Never treat configured CI, mocked drawing or a successful upload as exercised behavior. No mandatory Socratic checkpoints, speculative features or optimization campaigns without measurements.
