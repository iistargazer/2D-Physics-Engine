# phys2d / The mechanics lab

A hand-written, header-only **C++20 2D rigid-body engine** with no physics-library dependency. The browser showcase compiles the same core to WebAssembly; SFML is used only for the optional desktop renderer.

Convex circles/polygons, SAT and clipping, symplectic Euler, sequential impulses with friction, a dynamic AABB tree, feature-ID warm starting, two-point block normals, joints, opt-in NGS position correction and island sleeping are implemented. Bullet CCD is a **limited prototype**, not general collision certification.

## Run the native checks

From the repository root, using Bash (Git Bash on Windows) and a C++20 compiler:

```bash
mkdir -p build
g++ -std=c++20 -O2 -Wall -Wextra -Wpedantic -Werror -I. test_engine.cpp -o build/test_engine
./build/test_engine
```

Or with CMake 3.16+:

```bash
cmake -S . -B build/cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build/cmake
ctest --test-dir build/cmake --output-on-failure
```

The saved [native receipt](build/final_tests.log) has **143 passing assertions**. CTest registers one engine executable, not 143 separate tests. The saved checked-container debug receipt covers an older 76-assertion revision. [CI](.github/workflows/ci.yml) configures ASan/UBSan and SFML compilation, but no remote execution is recorded; local MinGW sanitizer linking was blocked. See the [current evidence ledger](PHYSICS_ENGINE_GUIDE.md#2-verification-ledger) for boundaries.

## Run the browser showcase

Install/activate Emscripten once (6.0.12 is the locally verified version):

```bash
git clone https://github.com/emscripten-core/emsdk.git build/emsdk
(cd build/emsdk && python emsdk.py install 6.0.12 && python emsdk.py activate 6.0.12)
source build/emsdk/emsdk_env.sh
bash web/build.sh
python -m http.server 8777 --bind 127.0.0.1 --directory build/web
# Open http://127.0.0.1:8777/index.html
```

For subsequent runs, source the SDK environment and build again. Serve the output over HTTP; do not open it with `file://`. Choose a free port. The output is **nine required files**: `index.html`, `index.js`, `index.wasm`, `showcase.css`, and the five frontend scripts `core.js`, `scenes.js`, `render.js`, `instrument.js`, `showcase.js`. Keep them together. Do not publish local logs or toolchains alongside them.

Ten experiments have selected-state navigation, per-scene framing, explanations, evidence boundaries, pause/step/restart, contact drawing, spawning and spring dragging. A live telemetry dashboard reads the engine every frame — body/sleep counts, the step cost against a 60 Hz budget, a wake/sleep composition bar, a rolling cost history and an opt-in seven-stage breakdown — and a tuning panel drives solver iterations, position correction, gravity and island sleeping on the running world. Chromium checks cover desktop/narrow layouts, DPR 1/2, camera inverses, interaction cleanup and the dashboard (221 + 50 + 87 assertions). Physical touch devices and other browser engines remain untested. [Showcase instructions](docs/SHOWCASE.md) own the controls and presentation script.

## Learn the project from scratch

1. [Project guide](PHYSICS_ENGINE_GUIDE.md): current status, conventions, ownership and unfinished acceptance.
2. [From scratch](docs/FROM_SCRATCH.md): personal reading/experiment path, data model, one complete step and debugging.
3. [Math notes](docs/math-notes/README.md): derive SAT, clipping, effective mass, accumulated clamping, friction and joint Jacobians.
4. [Scenes](docs/SCENES.md): connect each factory to its physical claim and regression.
5. [Benchmarks](docs/BENCHMARKS.md): measured tables, methodology and limitations.
6. [Applications](docs/APPLICATIONS.md): educational/sandbox uses versus work needed for games, robotics or other production uses.

## Architecture and limits

[World](World.hpp) validates state → builds/wakes islands → integrates forces → generates pairs/manifolds → restores/prepares impulses → interleaves joint/contact sweeps → integrates poses (or CCD event slices) → applies optional NGS → sleeps islands → saves caches.

- Coordinates are y-up, angles CCW, normals A → B; `phys::real` is `double`.
- Shapes are convex; concave/compound shapes, collision layers and serialization are absent.
- Bodies/joints use public vectors and indices; manifolds contain raw pointers. Do not mutate during a step; erasing/reordering bodies requires joint rebuilding. Sleeping body edits require waking them first.
- CCD applies normal and tangential impulses at the centre of mass, so off-center swept impact torque is omitted. No joint re-solving or connected-body exclusion during casts; no swept-tree acceleration. Per-bullet clamp/push-out counters are reported; there is no global freeze.
- The 5,000-box field has stacking contacts, then sleeps. Its headless settle/sleep timing is **not sustained active-5k or rendered 60 FPS evidence**. Dense 10k exceeds the step budget in the recorded benchmark.

## Desktop alternative

With an ABI-compatible SFML **2.x (2.5+)** installation:

```bash
cmake -S . -B build/demo -DPHYS_BUILD_DEMO=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build/demo --target demo
```

Run the resulting demo executable from the repository root. Windows requires matching SFML/compiler runtime DLLs beside the executable. Native startup/keyboard controls and PNG capture have been exercised; mouse/appearance coverage is narrower than the browser checks.

**Not yet shipped:** repository packaging, clean-clone/browser CI, executed sanitizers, representative active stress performance, and GitHub/Pages preparation. The [remaining checklist](PHYSICS_ENGINE_GUIDE.md#6-remaining-acceptance-and-github-preparation) is explicit. No permanent deployment is recorded.
