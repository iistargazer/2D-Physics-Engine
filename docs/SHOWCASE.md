# Showcasing phys2d

The presentation path is the browser **Mechanics lab**, with ten existing C++ scene factories, per-scene explanations, live measurements and camera framing. Native SFML and headless evidence are alternatives. [Project guide](../PHYSICS_ENGINE_GUIDE.md) owns status; [BENCHMARKS](BENCHMARKS.md) owns numbers; [FROM_SCRATCH](FROM_SCRATCH.md) explains implementation; [VISUAL](VISUAL.md) owns the palette, type scale and layout of the page itself.

## 1. Browser: build and serve

Prerequisites: Bash/Git Bash, Python, Git and Emscripten. Commands start at the repository root. Emscripten **6.0.12** is the locally verified version; the repo has not yet automated clean-clone setup.

```bash
# Once, if build/emsdk does not already exist:
git clone https://github.com/emscripten-core/emsdk.git build/emsdk
(cd build/emsdk && python emsdk.py install 6.0.12 && python emsdk.py activate 6.0.12)
# Every new shell:
source build/emsdk/emsdk_env.sh
bash web/build.sh
python -m http.server 8777 --bind 127.0.0.1 --directory build/web
# Open http://127.0.0.1:8777/index.html; use a different port if occupied.
```

Keep **all nine files** together:

| Output under `build/web/` | Source/role |
|---|---|
| `index.html` | [shell](../web/shell.html), generated HTML |
| `index.js` | Emscripten runtime glue from [main.cpp](../web/main.cpp) |
| `index.wasm` | Same C++ physics headers as native tests |
| `showcase.css` | Copied [presentation styles](../web/showcase.css) |
| `core.js` | Copied [DOM handles, camera transform, formatting](../web/core.js) |
| `scenes.js` | Copied [scene metadata and editorial copy](../web/scenes.js) |
| `render.js` | Copied [frame-buffer decode and canvas painting](../web/render.js) |
| `instrument.js` | Copied [telemetry dashboard controller](../web/instrument.js) |
| `showcase.js` | Copied [runtime glue, input, tuning controls, frame loop](../web/showcase.js) |

The five scripts load in that order as classic scripts (not ES modules), because the
Emscripten glue expects `Module` to be a real global. Each references the others only
inside functions that run after load, so the order is a reading order rather than a
dependency chain — but keep it.

#### One owner per read-out

State is not shared so much as delegated, and each of these is the only writer of what it
owns:

| Owner | Holds | Never does |
|---|---|---|
| `core.js` | DOM handles, the camera transform that both painting and picking use, the three body colours (`COLORS`), the frame budget (`BUDGET_MS`, `GAUGE_FULL_MS`), formatting | State or behaviour |
| `scenes.js` | The ten scenes' copy and framing, in engine order | Touch the DOM |
| `render.js` | The canvas: frame-buffer decode and painting | Write a read-out, or read one back |
| `instrument.js` | Every measured read-out — tiles, gauge, composition, stage split, plot axes, metre ruler — plus the status stamp through `setState` and the tuning controls' mirror of the solver | Drive input or navigation |
| `showcase.js` | The runtime object, scene navigation, pointer/keyboard input, tool state, the frame loop, and transport chrome (`syncChrome`) | Compute a measurement |
| `shell.html` | Structure and pre-boot placeholders; its seven stage rows are the one list of stages; the budget caption and the gauge's numeric label are filled from `core.js` | State a number that the code also states |

Two duplicates are deliberate. `COLORS` in `core.js` holds the canvas hues — staged a step
darker than the legend — while `showcase.css` owns the legend tokens; `a11y.cjs` asserts the
two stay the same hue, so they are a paired decision rather than drift. And the shell's
scene-0 copy repeats `scenes.js` because it is what a browser shows before the engine boots;
`showScene(0)` overwrites it with the same strings.

No CDN or runtime backend is required. The favicon is an inline SVG data URI, so the page
issues no third-party requests. Use HTTP, not `file://`. The directory may contain local
logs: copy only these nine files for delivery, never the SDK, dependencies or complete
`build/` tree. It is published at
<https://site-22be3251e8494dca85d1edd16f3d6ae8.freebuff.page>; [DEPLOY](DEPLOY.md)
records how that deployment was made, how to repeat it locally and which other hosts are
still untested.

### First run and controls

Start with Newton's cradle. The scene navigator becomes a horizontally scrollable strip at narrow widths. Each scene explains what to watch and separates native regression evidence from prototype limitations.

| Input | Action |
|---|---|
| Navigator / `1`–`9`, `0` | Load an experiment; `0` is Momentum row. Clears drag/spawn state and resumes |
| Pause/Resume / `Space` | Toggle running; a focused button handles Space as its normal activation |
| Step / `n` | One fixed 1/60 s step; use while paused |
| Restart | Rebuild the current scene; clear tools/drag and resume. Slow/debug preferences persist |
| ¼ speed | Toggle 0.25× simulation time; visible pressed state |
| Contacts / `d` | Contact points and normals scaled by accumulated normal impulse |
| Primary pointer | Grab a body with a mouse spring; release/cancel/lost capture/blur ends it |
| + Box / + Circle then canvas click | Spawn at the mapped cursor; choose the same tool again to turn it off |
| Iterations slider | Writes `SolverConfig::iterations` on the running world (1–400, clamped) and reports the value the solver actually holds |
| Gravity slider | Writes `World::gravity.y` (−50…50 m/s², y-up). Zero gravity is a valid experiment, not an error |
| Position correction / `g` | Toggles opt-in NGS projection on the live world |
| Sleeping / `s` | Toggles island sleeping. Switching it off also wakes every dynamic body, so the control cannot leave a silently frozen world |
| Stage profile / `p` | Enables `World::profiling`, which exposes the seven per-stage timings; see below |

Pause, contacts, scene and spawn tools acknowledge state through selected styling/`aria-pressed`; Pause changes to Resume. Camera and pointer inverse share the same transform, including CSS size/DPR. The car follows its chassis; tracking freezes during a drag so the target does not move under the pointer. Stress fits the whole field, making individual boxes tiny. Momentum's outgoing ball eventually leaves the fixed view—restart to repeat.

### Telemetry dashboard

Below the stage, the page reports what the engine is doing rather than what the UI assumes:

- **Counts.** Bodies, awake, asleep, static and contact points are decoded from the same
  frame buffer the renderer draws, so a tile and the picture cannot disagree. The sleep
  state is inferred from those counts (`ACTIVE` / `SETTLING` / `SLEEPING`, or `PAUSED`).
- **Step budget.** The measured single-step EMA against a 60 Hz frame, as a bar with the
  60 Hz mark at halfway and a `WITHIN`/`OVER 60 HZ BUDGET` verdict. Two budgets fill the bar.
- **Composition.** Static / asleep / awake as real proportions of the body count.
- **Step-cost history.** The last 120 frames of the step EMA with the budget drawn as a
  labelled reference line and the axis ruled from the same pad constants the series uses, so
  the 60 Hz rule and its label cannot drift apart; samples above budget are drawn in the
  over-budget colour.
- **Stage breakdown (opt-in).** The seven `World::timings` buckets — broadphase,
  narrowphase, islands, solve, integrate, CCD, bookkeeping — as shares of the measured
  total, with the Σ shown against the step EMA. They sum toward the step rather than
  hiding work in one bucket, so the reconciliation is itself a spot check.

The metre ruler under the stage is not decoration: `instrument.js` derives the on-screen size
of one metre from the same camera width the pointer inverse maps through, snaps the tick step so
ticks never collapse into a haze, and labels the two edges with the world coordinates the view
spans. Section numbers run 01–05, telemetry is a ruled ledger whose six tiles each carry a 2px
left rule in their own hue, and the breakdown and tuning panels share a row above 1180px.
[VISUAL](VISUAL.md) states the palette and the hue rule (warm = awake, lavender = asleep, teal
= static, rust = over budget and nothing else).

The tuning controls are seeded from whatever the current scene factory configured and
re-read from the engine after every change, scene switch and restart: they mirror the
solver, they are not a second source of truth. Restart therefore resets them to the
experiment's own values. The dashboard writes two things and reads everything else: it can
change iterations, NGS and sleeping, but it never writes `SolverConfig::mode`, so the
temporal solver stays unreachable from the page.

Because the browser coarsens its clock, a six-body step is shorter than the timer's
resolution and the stage rows read as `0.00 ms` or `BELOW CLOCK QUANTUM`. That is the
clock, not the engine; the breakdown resolves on the loaded scenes. Requesting the
breakdown is itself measurable work, which is why it is off by default.

### Feedback and recovery

Measurements are body count, awake/asleep/static dynamics, **contact points**, physics-step EMA, simulated step count and frame-pump cadence. They exclude rendering and do not measure achieved frame rate. Contact-point count is not the native title's manifold count. Colours distinguish static (teal), awake (amber) and asleep (lavender) bodies, the same three hues the telemetry tiles use — the stage varies their lightness, never their hue, and [VISUAL](VISUAL.md) records the ratio each one has to clear; the CCD bullet is rust, the one deliberate exception to the "rust means over budget" rule. Cradle/pendulum lines are presentation guides, not added constraints.

A first-run strip explains the three things worth doing — pick an experiment, drag or arm a
spawn tool, pause and step — and its dismissal is remembered in `localStorage`. If the
runtime aborts, an alert panel replaces it with the two things that actually fix it: serve
the directory over HTTP, and keep `index.js` and `index.wasm` beside `index.html`. The
reload button in that panel stays enabled while every other control is disabled. All
decorative motion (grain, vignette, the stage sweep, the status pulse, the scene-change
wipe) is suppressed under `prefers-reduced-motion: reduce`.

| Symptom | Action |
|---|---|
| Loading engine… remains | Check HTTP access and all five assets; inspect console/network, rebuild and reload |
| Engine unavailable | The runtime aborted; reload from HTTP and check JS/WASM sibling files. Missing initial scripts may fail before this message can appear |
| Unstyled page or dead controls | `showcase.css` or `showcase.js` missing; copy both from the build output |
| Timer cadence | Browser did not schedule rAF during startup; timer fallback is used, not a 60-FPS guarantee |
| Tiny stress bodies | Expected full-field overview, not a blank simulation; inspect awake count and timing |
| A small cost reads 0 or microseconds | Browser clock quantization affects the EMA; it cannot recover precision absent from the samples |

### Browser evidence

Final local Chromium receipts: [221 interaction checks](../build/browser-check/polish-results.log), [50 framing checks](../build/browser-check/framing-results.log) and [87 dashboard checks](../build/browser-check/dashboard-results.log). The first two are unchanged by the presentation pass; the third covers the telemetry tiles against the frame buffer, the gauge arithmetic and its over-budget state, the opt-in stage breakdown and its reconciliation with the step, the tuning controls' effect on the measured stage split, the wake-on-sleep-off behaviour, first-run guidance persistence, recovery-state visibility and reduced-motion suppression.

Real mouse/keyboard flows cover all scenes, spawning/grabbing, center/edge mapping, resize during drag, nondefault bitmap, restart and selected states; widths 390–1500 and DPR 1/2. The same three suites were re-run unchanged after the presentation rebuild — 221 + 87 + 50 assertions, exit 0, empty page-error lists ([check](../build/browser-check/presentation-check.log), [dashboard](../build/browser-check/presentation-dashboard.log), [framing](../build/browser-check/presentation-framing.log)) — and [VISUAL](VISUAL.md) records the before/after screenshots at 1440/1024/390 with the panel crops. A fourth check measures the palette itself and is versioned rather than disposable: [a11y.cjs](../build/browser-check/a11y.cjs) reads 83 text/graphic pairs off the live page against WCAG AA, finds 0 below (from 18 of 79 pairs before the repair), and adds nine assertions that no state is encoded only as colour — [receipt](../build/browser-check/a11y-after.log), [the before state](../build/browser-check/a11y-before.log). These are local scripts/receipts under `build/`, not a portable CI test suite, and appearance is asserted only where it can be measured: contrast and colour redundancy. Physical touch and other engines remain untested; OS blur coverage partly uses a dispatched blur event. No rendered performance acceptance was measured.

Driven steps are taken through the engine's own pause + single-step path, and the suite
asserts the simulated step counter equals the number of driver iterations, so a throttled
`requestAnimationFrame` cannot inflate the figures.

**Host load dominates these figures.** Two runs of the same suite on the same build moved
the stress-field step from 13.1–17.9 ms to 26–112 ms for the loaded cases and the settled
step from 6.9 ms to 13.7 ms. That is why the suite's assertions are relational — sleeping
cheaper than awake, the gauge equal to the step it was given, the stage sum reconciling with
the step — and why a millisecond threshold was removed as an assertion after it failed on a
loaded run. The table below is the quieter first run; treat it as a shape, not a constant.

| configuration (scene 7) | bodies | awake | contacts | step EMA | dominant stage |
|---|---:|---:|---:|---:|---|
| island sleeping off, 5000 awake | 5001 | 5000 | 10000 | 13.1–17.9 ms | bookkeeping 44% |
| same, iterations raised 10 → 120 | 5001 | 5000 | 10000 | 43–47 ms | solve 33.6 ms |
| same, NGS off (iterations 120) | 5001 | 5000 | 10000 | 35.7 ms | solve 29.6 ms |
| settled, sleeping on | 5001 | 0 | 1000 | 6.4–8.0 ms | broadphase |
| wrecking pyramid, default | 213 | 211 | 970 | 7.1 ms | bookkeeping 3.50 ms |

Two findings follow, both visible on the dashboard as it runs: raising iterations 10 → 120
took the solve stage from 2.8–3.3 ms to 31.0–33.6 ms, and switching position projection off
took bookkeeping from 5.8–7.6 ms to 0.4–0.5 ms — the browser-side corroboration of the
native result that NGS re-runs narrowphase per manifold per iteration. These are one host,
one build, simulation-only and not a rendered frame rate.

**These figures do not reproduce the earlier driven-stress table** in
[BENCHMARKS](BENCHMARKS.md#browser-stress-scene-driven-with-continuous-input), which recorded
13.3 ms resident and 44.7 ms under continuous input with sleep arriving around 270 driver
iterations. Re-running the same path here reached sleep inside 75 steps and measured a
resident step of 6.9 ms on a quiet host, rising to 13.7 ms under load; 44 ms was reached only
by configurations that raise the cost (iterations 10 → 120, or a heavily loaded host). The two
runs use different drivers (the older receipt labelled driver iterations; this one asserts the
step count 1:1) and this host's step costs are load-dependent, but the earlier step labels
should not be read as verified step counts.

## 2. Native SFML alternative

Requires ABI-compatible SFML **2.x, 2.5+**; this host built 2.6.2 locally. Do not substitute SFML 3 without adapting the renderer.

```bash
cmake -S . -B build/demo -DPHYS_BUILD_DEMO=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build/demo --target demo
# Run the produced executable from the repository root.
# For this host's earlier build: ./build/sfml-demo/demo.exe
```

Executables may be in configuration subdirectories with multi-config generators. Windows needs matching SFML and compiler runtime DLLs beside the executable. Native startup, scene/debug/pause/step and PNG writing were exercised; complete mouse/appearance acceptance is not recorded.

| Input | Action |
|---|---|
| `1`–`9`, `0` | Ten scenes, same order as browser |
| Space / N / S / R | Pause / step / 0.2× slow / restart |
| D | Body AABBs, contacts/impulse-scaled normals and joint anchors |
| G | A separately rebuilt visualization tree from current bodies—not access to World's internal tree |
| H | Help in title bar |
| F1 | Save `build/physics_scene_<n>.png`; run from repo root so `build/` exists |
| Left / right / middle mouse | Spring drag / spawn box / spawn circle |

The title contains narration/counts/phase timing; there is no in-window text HUD. Rendering interpolates position and rotation. Native tree overlays and browser contact overlays are not feature-equivalent.

## 3. Headless evidence and replay fallback

Use [README native commands](../README.md#run-the-native-checks). Every assertion prints `ok:` and the executable exits nonzero on failure. Count only a successful run, preserving its exit status:

```bash
set -o pipefail
./build/test_engine | tee build/presentation-tests.log
grep -c '^  ok:' build/presentation-tests.log
```

In scripts, stop immediately if the tested command fails (for example `set -euo pipefail`). CTest runs one engine executable. Benchmark build/capture commands and exact tables belong to [BENCHMARKS](BENCHMARKS.md), not this document.

Replay exports are C++ frames, **not live WASM**. They are useful offline fallbacks, with older mocked-canvas checks rather than the current live-browser acceptance:

```bash
mkdir -p build
g++ -std=c++20 -O2 -Wall -Wextra -Wpedantic -Werror -I. demo_broadphase.cpp -o build/demo_broadphase
./build/demo_broadphase build/broadphase_demo.html boxes
./build/demo_broadphase build/warm_start_demo.html pyramid
./build/demo_broadphase build/joint_demo.html chain
./build/demo_broadphase build/stability_demo.html ngs
./build/demo_broadphase build/bridge_demo.html bridge
./build/demo_broadphase build/domino_demo.html domino
```

## 4. A five-minute presentation

1. **Cradle → Momentum row:** show constrained transfer, then the ideal unconstrained conservation case. Do not attribute the row's exact momentum/speed assertions to the pendulum.
2. **Bridge or ragdoll:** grab a part and release; explain joint/contact sweeps and finite anchor error. Contact drawing shows impulses, not material stress.
3. **Dominoes or Rube-Goldberg:** causality is falsifiable. Native tests include the no-trigger comparison; the browser only displays the triggered machine.
4. **Car:** soft springs, sliding axles, persistent wheel torque. Following camera is rendering only; no scripted motion. It is not a complete drivetrain/tire model.
5. **Bullet:** restart at ¼ speed, or pause and step. At 900 m/s a discrete step spans 15 m. This demonstrated stop is not general off-center/jointed CCD correctness.
6. **Stress field:** show awake count collapsing. Say “settling and sleeping, simulation-only cost,” not “5k active at rendered 60 FPS.”

For ninety seconds: cradle, bridge, bullet; keep the evidence-boundary panel visible. For engineering follow-up, show the native test output and [benchmark tables](BENCHMARKS.md), including failed scalar-stack/long-chain configurations.

## 5. Before the room

- Build/serve fresh on the presentation machine; test one scene switch, pause/step, drag, spawn and restart.
- Keep the five browser assets together; do not depend on a running server from an earlier session.
- Read the [verification ledger](../PHYSICS_ENGINE_GUIDE.md#2-verification-ledger): 143 native assertions is not sanitizer evidence; configured CI is not an executed pass.
- Re-measure timing on the host if quoting it, retaining command exit status and methodology.
- Have a replay and headless output ready. Know convex-only shapes, raw pointer/index ownership, CCD torque/joint limitations and unproven active stress performance before someone asks.
