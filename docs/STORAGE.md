# Prepared stepping storage

This is the storage contract for [World](../World.hpp), [Broadphase](../Broadphase.hpp),
[DynamicTree](../DynamicTree.hpp), [Islands](../Islands.hpp) and the contact cache.
The five-track acceptance owner remains [FEATURE_ACCEPTANCE](FEATURE_ACCEPTANCE.md).
No TGS, worker pool, general CCD or rollback feature was added by this storage change
itself. The temporal (TGS) path has since landed as a consumer of this capacity budget:
World reserves one `TemporalContact` row per contact slot, so `prepare()` covers it with no
per-slice allocation — see [TGS](TGS.md) and `temporalTrajectory` in the storage regression.
`prepare()` also reserves the per-tick [row list](PARALLEL.md) (`joints + contacts` rows), so
the addressing pass that a parallel schedule needs is allocation-free too. Worker pools and
rollback remain absent.

## API and failure contract

```cpp
phys::World w;
// Create bodies/shapes/joints here (setup may allocate).
w.prepare({5001, 100, 20000, 20000, 20004});
// Limits: bodies, joints, candidate pairs, contact manifolds, total polygon vertices.
const auto status = w.tryStep(1.0 / 60);
if (status != phys::World::StepStatus::Complete) {
    // Do not treat this as a simulated tick. Prepare larger limits outside the loop,
    // then retry. No partial integration or silent contact dropping occurred.
}
```

`prepare(Capacity)` reserves storage; it does not warm up the simulation or require a
first allocating step. Limits are logical counts, not implementation-dependent vector
capacity. Calling it again can increase or decrease logical limits without clearing
valid solver history. Tree indices use `int`; setup rejects body limits above INT_MAX/2.
A prepared world's valid first and repeated steps allocate nothing within these limits.
Input exceptions (invalid dt/joint indices) and `step()`'s exception path are **not** part
of the allocation-free contract: use `tryStep()` for capacity status without exceptions.
Unprepared World usage still grows dynamically as before.

Status values are `Complete`, `Bodies`, `Joints`, `Vertices`, `Pairs`, `Contacts`.
A failed capacity check preserves bodies (including forces/awake state), joint impulses,
previous manifolds, contact history and previous dt. Candidate generation may update
the tree/query scratch and profiling timings before discovering pair/contact exhaustion;
those are derived data, not a committed tick. Partial candidate/manifold scratch is not
submitted to the solver. Retrying rebuilds it. `Broadphase::exhausted()` exposes bounded
candidate exhaustion to standalone callers too; do not consume its partial pair output.
AllPairs World mode streams candidates and therefore does not use the pair-buffer limit.

Body/shape/joint construction and public vector mutation remain setup/input operations
and may allocate. No add/erase during a step. Wake before editing sleeping geometry, as
before. A copied World keeps logical preparation limits but standard vector copying
need not retain reserved capacity: **call prepare again on the copy before allocation-free
use**. Manifold body pointers in a copied World are transient; the step rebuilds them.

## Owners and data flow

- **World:** public bodies/joints; current and pending manifold vectors; sorted previous
  contacts; previous-state snapshots and flat vertex storage; reusable CCD clamp bytes.
  Solver joint accumulators remain owned by Joint, never duplicated into a second graph.
- **Broadphase:** tight AABBs, proxy IDs, sorted candidate pairs and tree query stack.
  DynamicTree owns its pooled nodes/free list. Reserve 2B−1 nodes and the same worst-case
  traversal scratch (zero for empty capacity). Removing/reinserting proxies reuses nodes.
- **Islands:** union-find parents, one byte-per-body flag buffer reused by wake and sleep,
  and minimum quiet-time reduction scratch. No per-call vector<bool> or temporary vector.
- **Contact cache:** one sorted contiguous vector of pointer-free CachedManifold records.
  Two fixed ContactPoints contain both geometry and accumulated impulses; no per-pair map
  nodes and no second list of point impulses. Lookup merges the two lexicographically sorted lists with one forward cursor;
  rebuilding retains only current contacts, preventing stale impulse resurrection.
- **Snapshots:** POD records reference a shared World-owned flat copy of local vertices
  by offset/count. No nested Shape vectors or geometry hashes/collision risk. Cache
  mutation validation retains the existing exact-value comparison and sleeping shortcut.

Pipeline: validate inputs/body/joint/vertex limits → detect external cache edits → stage
pairs/manifolds and check their limits → commit cache invalidation → build/wake islands
using previous contacts → swap pending/current contacts → integrate forces → warm start/
solve → integrate poses or CCD using owned scratch → optional NGS → sleep → save sorted
contacts and flat snapshots. Geometry does not depend on forces, so staging it before
force integration preserves the existing solve inputs while allowing capacity failures
before simulation mutation. Invalidated old indices are not fed into the island pass.

The second manifold buffer is intentional: it is the staging boundary that permits
recoverable contact exhaustion without rolling back body state or copying a World.
No new storage manager, ECS, generic pool or alternative solver state is introduced.

## Exact layouts

Observed native GCC 13.2 MinGW x64 ABI (`double`/size_t 8 bytes, int/ShapeType 4 bytes).
These are in-process layouts, **not** binary snapshot/network formats; 32-bit WASM
size_t/pointer fields differ and must be measured independently before such a claim.
[Test source](../test_world_storage.cpp) prints actual sizes/alignment in receipts.

| Record | Member offsets (bytes) | Size / alignment |
|---|---|---|
| World::Capacity | bodies 0, joints 8, pairs 16, contacts 24, vertices 32 | 40 / 8 |
| World::CachedManifold | contactPoints 0 (2×104), normal 208, penetration 224, friction 232, restitution 240, indexA 248, indexB 256, count 264, tail pad 268–271 | 272 / 8 |
| World::Snapshot | position 0, velocity 16, angle 32, angularVelocity 40, invMass 48, invInertia 56, radius 64, vertexOffset 72, vertexCount 80, type 88, tail pad 92–95 | 96 / 8 |
| ContactPoint (unchanged) | position 0, featureId 16, depth 24, rA 32, rB 48, massN 64, massT 72, targetVn 80, accN 88, accT 96 | 104 / 8 |
| ContactManifold (unchanged) | bodyA 0, bodyB 8, normal 16, penetration 32, contactPoints 40, indexA 248, indexB 256, count 264, friction 272, restitution 280 | 288 / 8 |
| DynamicTree::Node (unchanged) | bounds 0, parent 32, left 36, right 40, height 44, next 48, body 56 | 64 / 8 |

Extra prepared element storage (excluding vector control blocks, allocator rounding and
public body shapes/joints): two 288C manifold buffers + 272C cache + 96B snapshots +
16V vertex copies + B CCD bytes; broadphase 64(2B−1) nodes, 4B proxies, 32B bounds,
4(2B−1) query scratch, 16P pairs; islands 8B parents + B flags + 8B times.
No cache-line padding is added: all of this is serial and no false-sharing claim follows.

## Checks and performance

[World storage regression](../test_world_storage.cpp), linked with
[allocation instrumentation](../test_allocation_counter.cpp), counts allocations around
**every full tryStep**, including the first step. Four 720-step driven contact/joint
trajectories cover tree/all-pairs × sleeping on/off against an unprepared reference,
plus cache clear/count shrink, body growth/shape replacement, all five exhaustion
statuses and retry with established history, bullet steps, actual sleeping and force-driven
wake propagation. Additional scenes exercise revolute limits, NGS bridge joints, car
suspension, weld/prismatic/motor owners and mouse constraints. Active contact frames and awake state
are asserted; sleeping throughput is not used as active-load evidence.

The separate [sleeping benchmark](../benchmark_sleeping.cpp) is deliberately not a debug
CTest gate. It runs five sequential trials: 30 early steps, 570 settling, 120 settled,
then 240-step settled sleep-on/off windows with alternating order. It prints raw CSV,
uses medians, requires all 5000 dynamics asleep and finite states, and exits nonzero if
**either** inherited `median(settled)<median(early)` or controlled
`median(sleep_on)<median(sleep_off)` fails. Moving the noisy criterion does not delete it.
The early/settled comparison still compares different motion states; controlled windows
start from identical settled poses, not an active interaction workload. No rendered FPS.
An active-interaction workload with a per-stage split is measured separately in
[BENCHMARKS](BENCHMARKS.md#active-interaction-load-5000-bodies-awake); this storage
benchmark is not that measurement.

```bash
set -euo pipefail
cmake -S . -B build/release -DCMAKE_BUILD_TYPE=Release
cmake --build build/release
ctest --test-dir build/release --output-on-failure
./build/release/benchmark_sleeping > build/sleeping.csv 2> build/sleeping-summary.log
```

Run benchmarks without concurrent compiles/tests; retain failure exits and raw samples.
Existing sleeping/force wake/island separation correctness checks remain in test_engine;
only wall-clock measurement and speed assertions moved out.

## Fresh receipts for this pass

GCC 13.2 MinGW x64, CMake 4.4.4, strict warnings-as-errors:

- [CMake/CTest](../build/prepared/ctest.log): all 3 executables passed (engine,
  cast allocations, World storage); all 11 default executables built.
- [Checked-container engine](../build/prepared/debug-final.log): 142 behavioral
  assertions passed, zero failures. The 143rd historical assertion is now the dedicated
  performance criterion, not a deleted acceptance outcome.
- [Checked-container storage](../build/prepared/storage-debug.log): zero allocations
  and exact prepared/dynamic trajectories passed; native sizes match the table above.
- [Five-trial benchmark](../build/prepared/sleeping-final.csv) and
  [summary](../build/prepared/sleeping-final-summary.log): exit 0. Median early
  27.578440 ms, settled 13.245532 ms; controlled sleep-on 12.655526 ms vs sleep-off
  42.387166 ms. All trials ended with 5000 sleeping dynamics. Local shared-host results,
  not an engine-version speedup claim or active/rendered throughput.
- Final whitespace, Markdown fences and local file links passed. No installed
  formatter/configuration was available; no clang-format certification is claimed.
- One initial full checked-container run was stopped after identifying debug quadratic
  range validation in repeated lower_bound cache lookup. The final forward-merge lookup
  was rebuilt and passed full Release and debug runs; that stopped run is not a pass.
- ASan/UBSan remain blocked by the previously verified missing MinGW runtimes. Browser/
  WASM/frontend and delivery work were intentionally out of scope for this pass.

Receipts remain disposable local build output, not curated release evidence.

## Next concrete dependencies

**TGS (done):** local contact anchors plus original separation, separation refreshed at
substep poses, and an explicit warm-start/restitution policy for h=dt/N are implemented in
[TemporalSolver](../TemporalSolver.hpp) and documented in [TGS](TGS.md). The prepared
buffers hold the fixed-size anchor rows; persistent impulses stay owned by `ContactPoint`
and no per-slice allocation was introduced.

**Parallel solver (next):** the two preconditions are now closed - solver row kernels
perform **no writes to immovable bodies** (guarded `RigidBody` helpers, bitwise verified),
and contact/joint rows have unique, index-stable identity in a deterministic order with a
validated deterministic colouring. See [PARALLEL](PARALLEL.md) for the row/schedule structure
and the seven named residual blockers (single row driver, the TGS refresh pass, an
allocation-free movable view, per-tick row-buffer ownership, the NGS pass, serial CCD/islands
and warm-cache ordering). Compare a serial coloured reference before adding workers/queues:
storage preparation alone does not establish thread safety, 10k performance, rollback or
bit determinism.
