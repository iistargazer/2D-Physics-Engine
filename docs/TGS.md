# Serial TGS and compliant contacts

[World](../World.hpp) selects `SolverMode::TGS`; PGS remains the default and unchanged
reference. [TemporalSolver](../TemporalSolver.hpp) owns transient contact-row mathematics;
World owns its prepared row buffer. [Joint](../Joint.hpp) remains the sole joint impulse
owner and refreshes its local anchors, Jacobians, bias and effective mass each slice.

```cpp
world.solver.config().mode = phys::SolverMode::TGS;
world.solver.config().substeps = 8;
world.solver.config().iterations = 20; // sweeps per slice, not per tick
world.solver.config().compliance = 0.001; // m/N per normal point; zero = rigid
world.solver.config().damping = 40;      // N*s/m per normal point
world.prepare({64, 64, 2016, 2016, 256});
auto result = world.tryStep(1.0 / 60);
```

TGS consumes the same [solver row](PARALLEL.md) identity as PGS, and its substep kernels
write only through the guarded `RigidBody` helpers, so its rows are a subset of the live row
list and are inert on immovable bodies.

Compliance/damping are uniform contact-row settings, not new deformable materials or
per-body material maps. Existing frequency/damping-ratio distance and mouse springs
operate with the actual slice duration. Other joint types remain rigid constraints.
No parallel execution, throughput promise, shape decomposition or rollback is implied.

## Equations and row layout

At initial contact p, let a=RAᵀ(p−xA), b=RBᵀ(p−xB), s0=−depth. With normal n fixed
for this tick, refresh rA=RA a, rB=RB b and

`s = s0 + dot(xB+rB − xA−rA, n)`.

Initially the two world anchors coincide, so the offset is zero. Translation and
rotation update separation without another narrowphase call. Effective inverse masses:

`Kn = invMassA + invMassB + invIA*(rA×n)^2 + invIB*(rB×n)^2`,

and similarly Kt for t=(ny,−nx). Anchors and geometry are transient; ContactPoint
continues to own accumulated normal/tangent impulses. There is no second impulse cache.

For h=dt/N, compliance α=1/k (m/N), damping c (N*s/m), implicit spring response is

`gamma = α / [h*(h+c*α)] = 1/[h*(c+h*k)]`,

`ERP = h/(h+c*α)`, `bias = ERP*C/h`.

Rigid contacts use gamma=0 and slice ERP=baumgarte/N. Using full tick ERP every slice
injected excessive corrective velocity in the 1/30 s, N=16 resting-stack test; this
normalization keeps rigid correction on the tick timescale. For negative separation,
C=min(s+slop,0), target=−ERP*C/h. For positive separation target=−s/h (speculative
closure limit). Normal increment:

`deltaLambda = (target − Jv − gamma*lambda)/(Kn+gamma)`.

Clamp total lambda≥0; apply the increment at both refreshed arms. Two-point contacts
solve the 2×2 unilateral block including gamma on its diagonal. Enumerate both active,
each single active and none; singular/ill-conditioned blocks use scalar rows. Friction
follows normals, uses refreshed Kt and clamps total |lambdaT|≤mu*lambdaN using current
slice support. Positive gap cannot generate tensile normal force.

Steady compliant load at a single circle contact satisfies penetration−slop=α*m*g.
Two normal points act as two springs in parallel: total stiffness depends on manifold
point count. This is an explicit per-point compliance model, not contact-area elasticity.

| Structure | Members/offsets | Size/alignment |
|---|---|---|
| TemporalPoint | localA 0 (16 bytes), localB 16 (16), separation 32 (8), bounce 40 (8) | 48/8 |
| TemporalContact | points[2] at 0 | 96/8 |
| TemporalSoftness | gamma 0, beta 8 (two doubles) | 16/8 |

Compile-time assertions enforce contact-row sizes/offsets on native and WASM. World
reserves one TemporalContact per contact-capacity slot, adding 96*C bytes to the
[prepared storage](STORAGE.md) budget. Existing logical Contacts exhaustion applies;
no additional hidden per-substep allocation or capacity limit exists.

## Tick order and policies

1. Validate dt, indices, limits and temporal configuration before simulation mutation.
   Stage broadphase/manifolds and capacity checks as in prepared PGS.
2. Invalidate warm caches on external body edits or PGS↔TGS mode switch, wake/build
   islands, swap staged contacts. PGS position-correction settings remain PGS-only;
   TGS uses refreshed velocity-level contact and joint bias, not extra NGS projection.
3. Restore cached normal/tangent impulses with h/previousH (previous duration is dt for
   PGS, h for TGS). Mode switches invalidate rather than reuse incompatible estimates.
4. Per slice: integrate forces/gravity/damping by h. On the first slice capture local
   anchors and restitution from pre-warm velocities for **all contacts**. Refresh rows,
   seed warm impulses once, refresh/seed joints, then interleave joint and contact sweeps.
5. Advance poses by h (or existing CCD by h). Keep input force/torque until tick end;
   the CCD scratch overload now optionally defers clearing so bullet slices do not lose
   all but the first slice of external force. Aggregate CCD counters across slices.
6. Clear forces/torques once, sleep, save last-slice impulses with previousH=h and
   snapshots. Next slice/tick warm seeding reapplies an estimate of that slice's support;
   it never applies once per iteration. With warmStarting=false, seed totals reset for
   each slice and joint ratio is zero.

Restitution is captured once and enabled only on the first slice; subsequent slices
use separation targets, avoiding repeated rebound. Contact normals/topology remain
fixed within the tick; new contacts are generated next tick. This is temporal anchor
solving, not N complete narrowphase simulations or general no-tunneling certification.
CCD retains its known center-of-mass torque/joint/filter limitations.

## Edge cases and tests

[Temporal regression](../test_temporal.cpp) exercises real `World::tryStep` with the
allocation counter around every step, including first steps and contact-capacity failure.

- Five-box resting heights/velocities for dt=1/30,1/60,1/120 and N=1,4,16.
- Active heavy-on-light **1000:1** contact load, depth<0.08 m, heavy height>1.4 m,
  speed<0.08 m/s across the same matrix. This hard configuration uses **1000 sweeps per
  slice**; stability is proven at that cost, not cheap 1000:1 convergence or 60 FPS.
- Driven rigid/soft 1000:1 distance joints, bounded length error and exact total input
  momentum over 600 ticks. Joint force is not multiplied by N.
- Compliance 0.0002/0.001, dt=1/30,1/120, N=1/8: settled compression within 0.004 m
  of α*m*g, speed<0.005 m/s and distinguishably softer response.
- Empty world; static/static rows and joints; coincident/duplicate anchors; local-anchor
  translation/rotation refresh; first-only elastic restitution and separating motion;
  bullet force lifetime; sleep/wake, mode/timestep changes and capacity failure/retry.

Zero/static effective mass produces massN=massT=0 and no impulse. Coincident distance
anchors use the existing deterministic x-axis fallback; near-singular blocks use scalar
solves. Negative/nonfinite compliance/damping and invalid N/iterations or unrepresentable
softness are rejected before stepping. Valid input exceptions are outside the
allocation-free contract. TGS does not turn zero radius/inertia malformed shapes into
valid input; shape-factory validation remains an inherited issue.

## Measured receipt (final revision)

Native `test_temporal` — **0 failed**, 142 `ok:` PGS assertions unaffected:

| Case | Measured | Bound |
|---|---|---|
| Five-box resting height, dt=1/30,1/60,1/120 × N=1,4,16 | max abs y−4.5 = **0.0496** (1/30,N=1), min 0.0055 (1/120,N=16); all speeds 0.000000 | <0.12 m / <0.05 m/s |
| Active 1000:1 heavy-on-light load, same matrix | depth 0.016460…0.010001 m, heavy y 1.480000…1.480002 m, speed 0.000000 | depth<0.08, y>1.4, |v|<0.08 |
| Damping decay vs off, α=0.001 | late error 0.000000000 (c=40) vs 0.000811010 (c=0); peak energy 0.194646 vs 0.196565 J | on < 10% of off |
| Soft contact, α=0.0002/0.001 × dt=1/30,1/120 × N=1,8 | compression 0.019620 / 0.098100 = α·m·g exactly; residual speed <0.005 | within 0.004 m of α·m·g |
| Driven 1000:1 distance joint | length error <0.004 (rigid), <0.012 (soft); \|p−20\| <1e-7 | bounded; force not ×N |
| Warm-start on/off, alternating dt | resting height/velocity bounds hold both ways | <0.12 m / <0.05 m/s |
| Static, degenerate, empty, capacity-failure and mode-switch inputs | rejected/complete with zero allocations; state untouched on rejection | as documented above |

The prepared-storage regression now covers these rows too: `temporalTrajectory` in
[test_world_storage](../test_world_storage.cpp) drives prepared TGS worlds against an
unprepared dynamic reference for 300 frames (sleeping on and off), proving identical bodies,
joints and contacts with zero allocations per step, an observable allocation-free `Contacts`
exhaustion that leaves state untouched, and a clean PGS↔TGS switch. Layout assertions report
`TemporalPoint 48/8` and `TemporalContact 96/8` on both x86-64 and wasm32.
All four translation units compile clean under `-Wall -Wextra -Wpedantic -Werror`. CTests: `engine`,
`ccd_allocations`, `world_storage`, `temporal` — 4/4 pass.

**Default-off PGS is unchanged.** The reference receipt captured before the temporal pass has 142
`ok:` assertions and no failure; the current build reproduces it byte for byte, differing only in the two
sleeping-throughput lines that now belong to [benchmark_sleeping](../benchmark_sleeping.cpp)
(`test_engine.cpp` keeps the correctness half: 5000/5001 asleep, max |v| < 0.05). `SolverConfig{}`
defaults to `SolverMode::PGS`, and the browser entry point never writes `SolverConfig::mode`, so
the temporal path stays unreachable from the page. The showcase tuning panel does write
`iterations`, `positionCorrection` and `World::sleeping` on the running world — it is the first
browser surface to mutate solver configuration at all, and it mirrors each value back from the
engine after every change.

**WASM and browser.** `test_temporal` recompiled with exception support runs in Node with the same
`0 failed` result and identical 9-decimal values, so local-anchor refresh and exception rejection are
not host-arithmetic artifacts. The showcase WASM rebuilt from the same headers, and the Chromium
checks pass 221 + 50 assertions with an empty page-error list. The later presentation pass
added a third suite (87 assertions) and rebuilt again; that pass touched only the JS/HTML/CSS
around the engine plus additive instrumentation, and not the solver.

### Remaining limits

- The 1000:1 configuration is stable only at **1000 sweeps per slice**. This is evidence of reached
  bounds, not cheap extreme-ratio convergence, and it is not a throughput result.
- No sanitizer receipt: ASan/UBSan runtimes are unavailable on this host, so memory- and
  UB-correctness rest on the allocation counter and behavioral bounds above.
- Overflowing softness is *rejected* (verified with a subnormal slice duration), never clamped;
  callers must lower `substeps` or raise `compliance` themselves.
- No browser frame-performance claim, and TGS is not exercised inside the showcase — only compiled
  and cross-run in Node for wasm32.

## Reproduce

```bash
set -euo pipefail
cmake -S . -B build/temporal/cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build/temporal/cmake
ctest --test-dir build/temporal/cmake --output-on-failure
source build/emsdk/emsdk_env.sh
bash web/build.sh
```

The browser showcase stays on default PGS; rebuilding its WASM verifies integration
without secretly changing existing scenes. Native tests exercise TGS. WASM TGS execution
needs a headless test artifact as well as the showcase build; receipts distinguish these
from merely compiling code and from browser frame-performance evidence.
