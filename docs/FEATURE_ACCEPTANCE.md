# Mission acceptance and architecture

Prepared storage follow-up: [STORAGE](STORAGE.md) owns the implemented capacity API,
flat caches/snapshots, scratch ownership, measured native layouts, zero-allocation
full-step regressions and dedicated sleeping benchmark. The historical audit below
predates that implementation; it is not the latest storage status.

This is the acceptance owner for the five-track expansion requested by the user.
Read [the project guide](../PHYSICS_ENGINE_GUIDE.md) first for existing conventions,
then the [learning guide](FROM_SCRATCH.md) and [math notes](math-notes/README.md).
The existing engine and ten-scene showcase are the baseline, not disposable scaffolding.
Unchecked gates are requested work, **not implemented capability**.

## Delivery contract

- Preserve public C++20 headers, y-up/CCW coordinates, double precision, A→B normals,
  existing scene behavior, native builds and the five browser output assets.
- Every track needs equations with units/signs, algorithm steps, exact tested layouts
  (`sizeof`, `alignof`, offsets, capacity), allocation tests, pipeline placement,
  numerical/overflow handling and focused behavioral evidence.
- Setup may allocate. Core casts/solves/steps must execute without allocation after
  explicit capacity preparation. Capacity exhaustion must be reported, not silently
  grow, truncate contacts, or claim guaranteed no tunneling.
- Test current optimized/debug and sanitizer builds; exercise the actual native and
  browser interfaces. Record toolchain/hardware, input, timing distribution and revision.
- Do not equate sleeping, separated bodies, headless throughput or simulation EMA with
  sustained active interaction/rendered FPS. Do not equate repeated runs of one binary
  with cross-platform bit-level determinism.
- Commit/push to https://github.com/iistargazer/2D-Physics-Engine only after implementation,
  meaningful verification and ownership review. No Pages publication was requested here.

## Smallest coherent extension and dependency order

Keep World as orchestrator; extend Continuous, Solver/Joint, Broadphase/Islands and
Collision at their existing seams. Introduce separate shape preprocessing, worker
execution and binary snapshot owners only when their implementations require them.
Do not replace all public bodies with a speculative ECS or duplicate the World state.

1. Establish fresh checks, allocation measurement, ABI/layout receipts and active-load
   benchmarks. Remove owning body copies from casts; prepare reusable scratch and then
   replace per-step map-node/point-vector allocations. A cast-only improvement is not
   an allocation-free World claim.
2. Add convex distance witnesses (GJK) against SAT/analytic oracles; use them for accurate
   TOI contact points. Preserve SAT/clipping's stable two-contact manifolds.
3. Add local-anchor temporal substeps and soft contact rows; validate serial TGS first.
4. Upgrade CCD to solve event contacts **and affected joints** for the remaining fraction;
   enforce connected-pair filtering everywhere, including safety nets.
5. Color the combined contact/joint graph; reuse serial row kernels in a bounded worker
   pool. Compare a serial colored reference before benchmarking threads.
6. Add compound geometry preprocessing and deformable meshes without pretending GJK/EPA
   alone implements concavity or softness.
7. Serialize all persistent state and validate replay; determine deterministic platform
   scope before enabling parallel replay. UI redesign can proceed independently after
   baseline browser tests are portable; use the same exports, not a second physics engine.

Target pipeline: input/tick → cache validation/wake → force integration → swept
broadphase → distance/manifold generation → islands → local-anchor/soft-row preparation
→ colored temporal solves → TOI event advancement/contact-and-joint solves → remaining
pose integration → optional position repair → sleep/cache save → snapshot/checksum.
CCD event graph changes require rebuilding affected constraints, not reusing stale rows.
Forces apply once per tick; substeps must not multiply force or warm-start impulses.

## 1. TOI sub-stepping CCD

Current: conservative casts with rotation, event slicing and center-of-mass normal/
friction impulses; clamp/push-out nets. Off-center torque and joint re-solving absent.

For constant velocities, sample x(t)=x0+vt, θ(t)=θ0+ωt. A unit-axis separation g is a
**lower** bound on Euclidean distance d: g≤d. With bounding radii R,
V=|vB−vA|+|ωA|RA+|ωB|RB bounds all relative point speeds; advance by max(g,0)/V.
A witness contact requires K=invMassA+invMassB+invIA(rA×n)^2+invIB(rB×n)^2,
Δλn=(target−Jv)/K and clamping total λn≥0. Tangent impulses use their own Kt and
|λt|≤μλn; apply at both witness arms, not at an invented midpoint.

Algorithm: swept candidates → earliest converged event with stable tie order → advance
poses to event → build witness manifold → solve affected contact/joint island → recompute
casts using new velocities → finish remainder. Fractional rewind means recovering the
pose at the event from the tick's saved start pose, never integrating forces twice.

- [ ] Witness-based off-center/oblique torque and moving/rotating target tests.
- [ ] Connected-body exclusions, simultaneous impacts, initially touching/separating
      pairs, jointed bullets, friction/restitution, and event-budget exhaustion tests.
- [ ] No crossing at 200/1000/10000 m/s for supported cases; report every unresolved,
      clamped, pushed-out or capped case. “All tunneling eliminated” is not a universal
      guarantee for a finite-iteration floating-point solver.
- [ ] Swept-tree candidate completeness against all-pairs; bounded reusable event scratch.
- [ ] Exact layouts/capacities and zero-allocation **whole CCD advance**, not just a cast.

Edges: zero V is no future event; tolerance stops asymptotic progress; grazing/degenerate
witnesses and exhaustion return explicit status. Re-evaluate after each impulse; a
motion nudge must not bypass a newly created impact.

## 2. TGS and soft constraints

Current: PGS with warm start/block normals; soft distance/mouse joints only, not TGS.
Let C be constraint error, K=JM⁻¹Jᵀ, h=dt/N, stiffness k (N/m), damping c (N·s/m).
Implicit softness gives γ=1/[h(c+hk)] (inverse mass), ERP=hk/(c+hk),
bias=(ERP/h)C, Δλ=−(Jv+bias+γλ)/(K+γ).
Compliance α=1/k is m/N; distinguish physical α from the timestep-scaled γ/CFM.
For frequency f and damping ratio ζ: k=meff(2πf)^2, c=2ζmeff(2πf).

Algorithm: build local contact anchors once → for each h update separation from current
poses → solve compliant rows and joints → advance poses → repeat. Store original
separation and anchor offsets so rotation updates are explicit. Warm start once with a
defined h/dt policy; restitution uses pre-impact velocity, not per-substep rebounces.

- [ ] Selectable serial TGS with existing PGS reference preserved.
- [ ] 1000:1 loaded stacks and joint chains: bounded penetration/anchor error, finite
      state and energy over a declared duration; compare PGS at equal cost.
- [ ] Soft contacts/materials with timestep/frequency/damping sweeps, measured decay and
      stiffness; no claim that compliant rigid contacts are deformable materials.
- [ ] Fixed-layout local-anchor/row structures, measured capacities, zero-allocation solve.

Edges: static/static K=0, rigid k→∞ handled explicitly, zero frequency means rigid policy,
valid h>0, clamp total unilateral impulse and friction using current normal support;
near-singular blocks fall back to scalar rows without NaNs.

## 3. Parallel graph solver

Current: single-threaded joint/contact sweeps; Islands groups only sleep/wake.
Constraints are graph edges; edges in a color may share static bodies but never a
writable dynamic body. In a color, disjoint velocity updates commute; colors remain
sequential with a barrier. This changes PGS order and must be checked against a serial
**colored** reference, not assumed identical to the original sweep.

Algorithm: stable indexed contact/joint edges → deterministic greedy coloring → compact
color ranges → dispatch each range to persistent workers → barrier → next color. Keep
constraints whole (two-point normal blocks are one edge). Static writes must be removed
from row kernels before treating a shared floor as conflict-free.

- [x] Coloring proof/tests: [Schedule.hpp](../Schedule.hpp) is validated over body conflicts by
      [test_rows.cpp](../test_rows.cpp) (24 rows sharing one immovable floor fit 1 colour;
      the naive conflict treatment needs 24), and contact+joint rows are both covered.
- [ ] Overflow policy and a race sanitizer run.
- [ ] Bounded persistent pool and lock-free task queue with documented atomic
      acquire/release publication, ownership and shutdown; no per-step task allocation.
- [ ] Exact queue/row/color layouts; independently written worker metadata aligned to
      measured cache lines (do not assume padding alone proves no false sharing).
- [ ] Repeatable correctness comparison, worker-count sweeps, active 5k and **10k+**
      sustained contact/joint workloads, median/p95/p99 simulation timing and scaling.
      The serial half of the 5k workload is now measured with a per-stage split,
      median/p95/p99/max and three trials ([active.csv](../build/active.csv),
      [BENCHMARKS](BENCHMARKS.md#active-interaction-load-5000-bodies-awake)); 10k+, scaling
      and any worker-count sweep remain unmeasured.
- [ ] Rendered 60 FPS verified separately on declared hardware (whole-frame 16.67 ms
      budget), awake/contact counts maintained throughout. Report shortfalls honestly.
- [ ] Browser threaded mode tested with required isolation headers; retain tested serial
      mode where threads are unavailable without claiming equivalent performance.

Edges: many colors/high-degree piles can serialize; queue overflow cannot drop work;
never resize bodies while workers hold pointers. Dense candidate output may be quadratic.

## 4. GJK/EPA, concave decomposition and soft meshes

Current: circles/convex polygons only; no compounds or deformables.
Minkowski difference support s(d)=supportA(d)−supportB(−d). GJK iterates supports toward
the origin, reducing a simplex (≤3 points in 2D) and retaining barycentric witness points.
For overlap, EPA expands a CCW polytope at its closest edge until support distance minus
edge distance ≤ tolerance; interpolate witnesses on that edge for penetration response.

Algorithm: support mapping → bounded GJK simplex → EPA for overlap, or distance witnesses
for casts → existing contact solver. Keep feature identity and stable contact clipping;
a single EPA point is not a replacement for stable stacking manifolds.
Concavity requires preprocessing: HACD-style adjacency clustering with a declared
concavity metric, convex hulls and merge threshold → immutable compound pieces/local
transforms → child broadphase/narrowphase → deduplicate seam contacts, sum mass/inertia.
Soft meshes are separate dynamic vertices/edges/triangles: e.g. XPBD uses
Δλ=−(C+α̃λ)/(Σwi|∇iC|²+α̃), α̃=α/h², then Δxi=wi∇iCΔλ.

- [ ] GJK distance/EPA witness correctness against circle analytics/SAT, swapped order,
      coincident supports, skinny/degenerate shapes and randomized convex oracles.
- [ ] Actual concave decomposition with cavity collision tests, area/mass/inertia
      preservation and seam behavior; triangulation alone is not HACD.
- [ ] Soft meshes with stretch/area/bending policies, rigid coupling, self-collision
      strategy and measured material/energy behavior—not just connected rigid boxes.
- [ ] Exact bounded simplex/EPA/compound/mesh layouts and capacity failure statuses;
      preprocessing allocation separated from allocation-free runtime.

Edges: duplicate supports, zero search direction, collinear simplices, nonconvex or
self-intersecting source input, EPA capacity/iteration caps and inverted triangles.

## 5. Deterministic rollback

Current: repeated single-binary trajectories tested; no serialization/restore/networking.
S(t+1)=F(S(t),orderedInputs(t),dt). Replay requires all persistent inputs to F, including
solver config, previous dt, feature impulses, joint accumulators, sleep state, shape
identity, forces/drives and stable ordering; saving only x,v cannot restore the engine.

Algorithm: explicit versioned little-endian field encoding (not memcpy of C++ structs)
→ ring of tick snapshots → restore authoritative tick → apply ordered inputs → resimulate
→ canonical state hash per tick → compare with uninterrupted headless run. Rebuild
transient pointer/tree structures deterministically or encode their relevant state.

- [ ] Exact binary magic/version/count/field offsets, bounded byte size, corruption and
      incompatible version rejection; restore validates before mutating World.
- [ ] Mid-contact/mid-joint, sleeping, bullet and soft-mesh restores produce bit-identical
      continuation over many ticks and late-input rollback.
- [ ] Allocation-free capture/restore/resimulation after preparation, fixed ring capacity.
- [ ] Explicit platform/compiler/FP contract, no fast-math, specified FMA policy,
      deterministic trig and reductions where necessary; native/WASM comparisons.
- [ ] Serial/parallel worker-count comparisons and network-ready input/tick protocol.

Edges: NaNs/signed zero canonicalization policy, shape mismatch, stale handles, integer
length overflow, tick wrap, packet reordering and snapshot truncation. Cross-platform
bit determinism is not promised until cross-platform hashes actually agree.

## Exact memory and allocation evidence

C++ ABI layouts of vector-bearing existing types are compiler-dependent; serialization
must never use their object representation. Measure `sizeof`/`alignof`/legal `offsetof`
on native and WASM and preserve receipts before claiming a portable layout.

The first prerequisite adds `SweepPose` only: position at byte 0 (16 bytes), cached
Mat22 at byte 16 (32 bytes), size 48/alignment 8 on the enforced double ABI. Compile-time
assertions enforce offsets/size/alignment. Each cast samples two poses (96 bytes) and
borrows immutable original geometry. No copied Shape vectors, heap ownership or shape
capacity limit is introduced. This does **not** make World allocation-free: map caches,
per-contact vectors, island scratch, CCD clamp scratch and growing pair/manifold buffers
remain. Other tracks' exact layouts are pending their real implementations; no speculative
structure table is presented as delivered code.

## Frontend and inherited completion gates

Current frontend already has responsive styling, framing, controls and evidence notes.
The new request calls for a substantial improvement, not merely changing its title.

- [ ] Portable source-controlled browser regression harness and reproducible native,
      current checked-container debug, ASan/UBSan, WASM builds with pinned inputs.
- [ ] Sustained active-load baseline with correctness and separate render timing.
- [ ] Extensive visual redesign: coherent typography/spacing/color, improved experiment
      discovery and stage/telemetry hierarchy while preserving all scenes/exports.
- [ ] Keyboard/focus/contrast/reduced-motion, narrow-screen/DPR/resize and error/loading
      states; actual browser screenshots and spawn/drag/pause/step/restart tests.
- [ ] Other browser engines and physical touch; full SFML mouse/overlay checks and
      portable release bundle; clean-checkout learning/build walkthrough.
- [ ] Curated public receipts, ignore rules, license/provenance decision and source-only
      Git staging review; reconcile target repository history before commit/push.

## Audit and blockers (this pass)

- Guide read fully; World, CCD, contact/joint solvers, collision, broadphase/islands,
  body/math layouts, native test/benchmark and browser owners inspected.
- Existing receipts: 143 optimized assertions, older 76 debug assertions, browser
  interaction/framing receipts, narrow CCD CSV and settle/sleep showcase CSV. Sanitizer
  CI is configured, not executed evidence; active/rendered 10k and rollback absent.
- No `.git` in this workspace: no branch, diff, tracked ownership or remote configuration
  can be inspected. `git ls-remote` on the supplied URL returned success with no refs;
  that alone does not establish repository contents, visibility or push permission.
  Do not initialize/stage this mixed source/build/toolchain directory as a release.
- MinGW GCC 13.2 is available; CMake/formatter are not on PATH (inspect local tools before
  declaring unavailable). Prior sanitizer linking lacked runtimes. New receipts must
  distinguish checks actually executed from these inherited results.

### Fresh verification receipts

Local outputs under [mission-audit](../build/mission-audit/release.log) are disposable,
not release artifacts. GCC 13.2 / Windows MinGW; Emscripten 6.0.12; local CMake 4.4.4.

- Strict `-O2 -Wall -Wextra -Wpedantic -Werror` engine run: 143 assertions passed.
- [Allocation regression](../test_ccd_allocations.cpp), linked with
  [counter instrumentation](../test_allocation_counter.cpp): 200 casts, zero allocations,
  exact copy-oracle results (160 hits, 20 misses, 20 initially touching). The counter
  also checks it detects the oracle's owning polygon copies. Native optimized and
  checked-container builds pass; layout size 48/alignment 8. Exhausted casts are not
  represented by this corpus. No timing speedup is claimed.
- Fresh checked-container full engine run: **one failure**, inherited sleeping timing
  assertion (69.9131 → 92.1576 ms); behavioral assertions passed. It overlapped build/
  test work on this shared host, so this is not controlled performance evidence.
- CMake built all nine default executables with warnings-as-errors; CTest allocation
  test passed, engine failed the same timing assertion (12.1306 → 13.0939 ms).
  Tests/assertions were not removed or loosened. Timing methodology remains a blocker;
  these failed receipts are not superseded by the separate passing optimized run.
- ASan/UBSan compilation/link attempt failed: missing `-lasan` and `-lubsan` runtimes.
  No sanitizer execution is claimed.
- WASM rebuilt successfully with layout assertions; all five HTTP assets returned
  expected content types. Existing Chromium harness passed 221 interaction assertions
  with no runtime errors. This harness still lives in build output and has a hardcoded
  Chrome path; portability remains open. This pass does not redesign the frontend.
- No repository formatter configuration or installed clang-format was found; source
  whitespace and documentation links are checked, not a clang-format certification.
