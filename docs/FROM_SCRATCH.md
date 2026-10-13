# phys2d from scratch

You know C++ and school algebra. You do not know how a physics engine works. This
document takes you from that starting point to changing `phys2d` yourself, without an
assistant: it derives the mathematics, names the exact symbol, file and line that
implements each idea, and ends with a worked change you can make and test.

It is written for one reader at a time and it is meant to be *edited* as the engine grows
(§11). Every section has the same shape:

| Part | What it gives you |
| --- | --- |
| **Intuition** | the physical picture in plain words |
| **Derivation** | the algebra, done properly, in this repository's conventions |
| **Code map** | the symbol, the file, the line, and the data layout it owns |
| **Exercise** | something to compute or change, with a hint |
| **What is not proven** | the honest boundary of the claim |

Prerequisites: dot and cross products, the idea of a derivative, Newton's laws, and
enough C++ to read `struct`s, `std::vector` and templates. No prior physics engine, no
graphics, no linear algebra beyond 2×2.

Companion documents (this one is the narrative; they are the ledgers):

| Document | Read it for |
| --- | --- |
| [../PHYSICS_ENGINE_GUIDE.md](../PHYSICS_ENGINE_GUIDE.md) | current status, conventions, ownership, known limitations |
| [../README.md](../README.md) | short build entry point and the reading path |
| [math-notes/README.md](math-notes/README.md) | one-topic derivations (SAT, clipping, warm starting, joint Jacobians, ...) |
| [BENCHMARKS.md](BENCHMARKS.md) | raw measured tables and how to reproduce them |
| [STORAGE.md](STORAGE.md) | prepared capacity, allocation contract and byte layouts |
| [TGS.md](TGS.md) | the temporal substepped solver, its equations and its measured limits |
| [PARALLEL.md](PARALLEL.md) | solver-row identity, the deterministic colouring, residual blockers |
| [SCENES.md](SCENES.md), [SHOWCASE.md](SHOWCASE.md), [APPLICATIONS.md](APPLICATIONS.md) | scene catalogue, browser demo, practical fit |
| [FEATURE_ACCEPTANCE.md](FEATURE_ACCEPTANCE.md) | the multi-track acceptance ledger this project is judged against |

## 0. Two reading paths

**The linear path (recommended first time).** Read §1, build the project in §2, then
read §3 and §4 in order, then §5 in order, then §9 and make the change it describes. §6
to §8 are reference: read them when you need the allocation contract, the scenes or the
verification ledger.

**The derivation-first path (Mathematics & Computing background).** Treat the engine as
an executable derivation: §3.1 and §5.1 (vectors, rotation), §5.2 (integration), §5.3
(geometry), §5.4 to §5.8 (constraints), §5.11 to §5.13 (sleeping, substepping, CCD), and
run the existing studies next to each derivation:

1. Conventions and mass properties: §3.1 to §3.4 and `testMassProperties` in
   [../test_engine.cpp](../test_engine.cpp#L22). Explain why torque uses a lever arm
   from the centre of mass and why normals always point A → B.
2. Integration: §5.2 and [IntegratorStudy.hpp](../IntegratorStudy.hpp).
   Reproduce the step-halving ratios with `study_integrators` (§2) and explain why a
   fourth-order smooth-force method is not a fourth-order contact solver.
3. Geometry: §5.3 with [math-notes/sat.md](math-notes/sat.md) and
   [math-notes/clipping.md](math-notes/clipping.md). Predict why two support points stop
   a box rotating, then check it in the browser with **Contacts** on.
4. Constraints: §5.4 to §5.8 with [math-notes/effective-mass.md](math-notes/effective-mass.md),
   [math-notes/accumulated-clamping.md](math-notes/accumulated-clamping.md) and
   [math-notes/baumgarte-vs-ngs.md](math-notes/baumgarte-vs-ngs.md). Reproduce the
   warm/cold and scalar/block rows the benchmarks retain.
5. Islands and joints: §5.10 and §5.11. Compare a pendulum period with `2π√(L/g)` and
   then watch the awake count fall in the stress scene.
6. CCD and applications: §5.13, then [APPLICATIONS.md](APPLICATIONS.md). State one
   useful educational/sandbox use and the blockers for a production game or robot.

For an interview: be able to trace one frame without skipping the cache validation step,
explain one failure row from [BENCHMARKS.md](BENCHMARKS.md), and say what was *measured*
versus what was merely implemented.

---

## 1. What the project is, and the rules it plays by

`phys2d` is a **2D impulse-based rigid-body engine written from first principles in
C++20**. The engine is **header-only**: the root-level `.hpp` files are the library, and
`-I.` is the whole build requirement for them. There is **no third-party physics
dependency**: linear algebra, manifold generation, the constraint solver, sleeping,
continuous collision and the prepared-storage layer are all hand-written. SFML appears in
exactly one optional file ([../demo/main.cpp](../demo/main.cpp), pixels only) and
Emscripten compiles the *same headers* for the browser ([../web/main.cpp](../web/main.cpp)).

The ground rules explain most design decisions you will meet:

1. **Every engine change keeps the regression suite green.** `test_engine.cpp` is the
   contract; a feature without a test does not count as done. Five executables are
   registered with CTest (§8).
2. **Report fallbacks.** `CCDStats` exposes event caps, clamps, push-outs and unresolved
   casts. A silent safety net is treated as a bug.
3. **Conventions are fixed** (table below) and never changed quietly.
4. **Clarity over micro-optimization**, except where a measurement justifies the
   complexity. The dynamic tree, the frozen-pair cache, sleeping and the prepared-capacity
   layer all exist because something was measured.
5. **Explain the mathematics next to the code.** The comment blocks at the top of
   [Solver.hpp](../Solver.hpp), [Collision.hpp](../Collision.hpp) and
   [Continuous.hpp](../Continuous.hpp) are part of the deliverable, not decoration.

### 1.1 Fixed conventions

| Topic | Convention | Where it is enforced |
| --- | --- | --- |
| Coordinates | Right-handed, **y-up**, angles **CCW-positive** | [Vec2.hpp](../Vec2.hpp#L15); flip y only in renderers |
| Scalar | `phys::real`, currently `double` | [Vec2.hpp:13](../Vec2.hpp#L13) |
| Body origin | `position` is the **centre of mass** | [RigidBody.hpp](../RigidBody.hpp#L30) |
| Polygon data | local space, **CCW**, centroid at the origin | [`makePolygon`](../RigidBody.hpp#L137) |
| Static bodies | `invMass == 0` (and `invInertia == 0`) — never special-cased in impulse math | [`isStatic`](../RigidBody.hpp#L63) |
| Contact normal | always **A → B**; `+λn` on B, `−λn` on A | [`collide`](../Collision.hpp#L212) |
| Tangent | `t = (n.y, −n.x)` | [Solver.hpp](../Solver.hpp#L70) |
| Material mixing | friction `sqrt(μA·μB)`, restitution `max(eA, eB)` | [Solver.hpp](../Solver.hpp#L70) |
| Rotation | cached `rot = R(angle)`; call `setAngle()`, never assign `angle` | [RigidBody.hpp](../RigidBody.hpp#L65) |
| Time step | fixed; the showcase uses an accumulator and clamps frame time | [../web/main.cpp](../web/main.cpp) |

### 1.2 Notation used in this document

| Symbol | Meaning |
| --- | --- |
| `x`, `v`, `a` | position, velocity, acceleration (2D, y-up) |
| `θ`, `ω`, `α` | angle (rad), angular velocity (rad/s), angular acceleration |
| `m`, `I` | mass, moment of inertia about the centre of mass |
| `m⁻¹`, `I⁻¹` | inverse mass/inertia as *stored* by the engine |
| `r` | lever arm: a vector from a body's centre of mass to a point on it |
| `n`, `t` | unit contact normal (A → B), unit tangent `(n.y, −n.x)` |
| `λ` | impulse magnitude along an axis (`P = λn`) |
| `K` | effective inverse mass of a constraint row (`λ = Δv / K`) |
| `β` | Baumgarte / position-projection factor (`baumgarte`, `positionBeta`) |
| `C` | constraint position error (for contacts, `C = −depth`) |
| `h` | a *slice* of the step in TGS, `h = dt / substeps` |

---

## 2. Build and run from zero

Every command assumes the repository root and a Bash-like shell (Git Bash on Windows).
There is no configure step for the fast path.

**Fastest path — the regression suite.** This is the loop you will use all day:

```bash
mkdir -p build
g++ -std=c++20 -O2 -Wall -Wextra -Wpedantic -Werror -I. test_engine.cpp -o build/test_engine.exe
./build/test_engine.exe
```

Expected output: one `[section]` banner and one `  ok:   ...` line per assertion, then
`ALL TESTS PASSED (0 failed)`. The current tree prints **142** `ok:` lines on a clean
optimized run (verified on GCC 13.2.0 MinGW x64; see §8 for receipts and for the two
historical extra lines). If your count differs by more than the change you just made,
something is wrong — the suite is deterministic by construction (§5.9).

**CMake path** — the suite, every benchmark, the HTML replay exporter and the optional
SFML demo (needs CMake/CTest installed; see the caveat just below):

```bash
cmake -S . -B build/cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build/cmake
ctest --test-dir build/cmake --output-on-failure
```

**Measured caveat, this checkout's host:** neither `cmake` nor `ctest` is on `PATH` here
(checked on 2026-10-13), so the CMake path below is the *documented* path — it was used on
the host that produced the [STORAGE](STORAGE.md) and [TGS](TGS.md) receipts, and it is safe
to run wherever CMake is installed, but it cannot be run in this session. The by-hand
compile above is the verified path on this machine, and it is equivalent evidence for the
same sources. If `cmake --build` succeeds but `ctest` is "not recognized", compile the five
suites by hand the same way.

| Program | Source | What it does |
| --- | --- | --- |
| `test_engine` | [../test_engine.cpp](../test_engine.cpp) | all native regression assertions; non-zero exit on any failure |
| `test_ccd_allocations` | [../test_ccd_allocations.cpp](../test_ccd_allocations.cpp) | 200 casts against a copying oracle, allocation-counter checked |
| `test_world_storage` | [../test_world_storage.cpp](../test_world_storage.cpp) | prepared capacity, zero-allocation stepping, byte layouts |
| `test_temporal` | [../test_temporal.cpp](../test_temporal.cpp) | TGS resting/compliance/1000:1 matrices, rejection paths |
| `test_rows` | [../test_rows.cpp](../test_rows.cpp) | row identity/order, immovable-body immutability, colouring |
| `benchmark_broadphase` | [../benchmark_broadphase.cpp](../benchmark_broadphase.cpp) | tree vs all-pairs, sparse/dense grids |
| `benchmark_stability` | [../benchmark_stability.cpp](../benchmark_stability.cpp) | warm/cold, scalar/block stack and pyramid comparison |
| `benchmark_joints` | [../benchmark_joints.cpp](../benchmark_joints.cpp) | chain anchor error vs chain length |
| `benchmark_ccd` | [../benchmark_ccd.cpp](../benchmark_ccd.cpp) | bullet cast cost, and whether a trial tunnelled |
| `benchmark_showcase` | [../benchmark_showcase.cpp](../benchmark_showcase.cpp) | per-scene headless step cost, joint error, CCD counters, awake count |
| `benchmark_sleeping` | [../benchmark_sleeping.cpp](../benchmark_sleeping.cpp) | five-trial sleep-on/off medians; exits non-zero if its criterion fails |
| `benchmark_active` | [../benchmark_active.cpp](../benchmark_active.cpp) | active-5k per-stage split, median/p95/p99, three trials |
| `study_integrators` | [../study_integrators.cpp](../study_integrators.cpp) | symplectic / Verlet / RK4 energy study |
| `demo_broadphase` | [../demo_broadphase.cpp](../demo_broadphase.cpp) | writes a self-contained HTML frame replay |

The benchmark programs print CSV to stdout; redirect to `build/*.csv` to reproduce the
tables in [BENCHMARKS.md](BENCHMARKS.md). `benchmark_sleeping` and `benchmark_ccd` have
failing exit criteria on purpose: keep the exit status.

**Optional SFML demo** (SFML 2.5+; 2.6.2 was built locally under `build/deps` to avoid an
ABI mismatch):

```bash
cmake -S . -B build/demo -DPHYS_BUILD_DEMO=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build/demo --target demo
```

**Browser build (WebAssembly).** Same physics headers, JavaScript renderer:

```bash
git clone https://github.com/emscripten-core/emsdk.git build/emsdk
(cd build/emsdk && python emsdk.py install 6.0.12 && python emsdk.py activate 6.0.12)
source build/emsdk/emsdk_env.sh
bash web/build.sh
# -> build/web/{index.html,index.js,index.wasm,showcase.css}
#                + {core.js,scenes.js,render.js,instrument.js,showcase.js}
python -m http.server 8777 --bind 127.0.0.1 --directory build/web
```

Open `http://127.0.0.1:8777/index.html` (use a free port). Keep all nine assets together;
WASM is not loaded from `file://`. Version 6.0.12 is locally verified, not a clean-clone CI
guarantee. [SHOWCASE.md](SHOWCASE.md) documents the controls and the dashboard.

---

## 3. The data model

Read this section with the headers open. Every claim below is a claim about the file named
next to it, and the code maps give you the line to check.

### 3.1 `Vec2` — vectors, and why 2D has two different "cross products"

**Intuition.** A vector is `(x, y)`. Two operations do all the work in this engine:
projection (`dot`, how much of one vector lies along another) and rotation by 90°
(`perp`), which is how a spin turns into linear motion.

**Derivation.** For vectors in the xy-plane, the 3D cross product `a × b` is *always*
along z, so its x and y components are zero and only the scalar survives:

```
a × b = (a.x, a.y, 0) × (b.x, b.y, 0) = (0, 0, a.x*b.y − a.y*b.x)
```

The engine therefore defines `cross(Vec2, Vec2)` to return that scalar, which equals
`|a||b| sin θ` — the signed area of the parallelogram spanned by `a` and `b`.

A rigid body spins about z with angular velocity `ω = (0, 0, ω)`. The velocity of the
material point at lever arm `r` is `ω × r`, which with the z-only vector is:

```
ω × r = (0, 0, ω) × (r.x, r.y, 0) = (−ω·r.y, ω·r.x) = ω · perp(r)
```

so `perp` is not a convenience: it *is* the operator that turns angular velocity into
linear velocity. The other direction is needed too — the torque of a force `F` applied at
lever arm `r` is `r × F = (r.x·F.y − r.y·F.x)` — and the negation of `ω × r`:

```
cross(Vec2 a, Vec2 b)  = a.x*b.y − a.y*b.x     // signed area / z of a × b
cross(real s, Vec2 v)  = (−s·v.y,  s·v.x)      // = s · perp(v)          [ω × r]
cross(Vec2 v, real s)  = ( s·v.y, −s·v.x)      // = −cross(s, v)         [r × ω]
```

**Code map.**

| Symbol | File / line | Notes |
| --- | --- | --- |
| `phys::real`, `phys::kEps` | [Vec2.hpp:13](../Vec2.hpp#L13) | `real = double`; `kEps = 1e-9` is the engine's "zero" |
| `struct Vec2` | [Vec2.hpp:15](../Vec2.hpp#L15) | two `real`s, `constexpr` arithmetic, `lengthSq`, `length` |
| `Vec2::normalized` | [Vec2.hpp:30](../Vec2.hpp#L30) | returns `{}` for a near-zero input instead of NaN |
| `Vec2::perp` | [Vec2.hpp:36](../Vec2.hpp#L36) | `(x,y) -> (−y,x)`, 90° CCW |
| `cross(Vec2, Vec2)` | [Vec2.hpp:54](../Vec2.hpp#L54) | the scalar above |
| `cross(real, Vec2)` | [Vec2.hpp:55](../Vec2.hpp#L55) | `ω × r`; every `velocityAt` uses it |
| `cross(Vec2, real)` | [Vec2.hpp:56](../Vec2.hpp#L56) | `r × P` torque |
| `clamp` | [Vec2.hpp:58](../Vec2.hpp#L58) | used by the friction cone and position clamps |

There is no `Vec3` anywhere: 2D bodies have three degrees of freedom (x, y, θ) and the
"third" is a scalar, so a `Mat22` plus a `real` covers all of it.

**Exercise 1.1.** Show that `cross(s, v) = −cross(v, s)` and that
`cross(s, v) = s · perp(v)`. Then explain why `velocityAt` can be written either as
`cross(ω, r)` or as `ω * perp(r)`, and find which one [RigidBody.hpp:103](../RigidBody.hpp#L103)
uses.

**Exercise 1.2.** `perp` is defined by `(x,y) -> (−y,x)`. Compute `perp(perp(v))` and
`cross(a, perp(a))`, then say in one sentence what each result means physically.

### 3.2 `Mat22` — the rotation matrix, and why you rarely invert it

**Intuition.** Body-local geometry (the vertices you typed) must be mapped into world
space every time a body moves. That map is a rotation, and rotations are cheap to invert.

**Derivation.** `R(θ)` maps a local vector to world:

```
R(θ) = | cos θ  −sin θ |        R(θ) · (1,0) = (cos θ, sin θ)
       | sin θ   cos θ |        R(θ) · (0,1) = (−sin θ, cos θ)
```

so its **columns are the rotated basis vectors**, matching the "right-handed, y-up, CCW"
convention. Because `Rᵀ R = I` and `det R = +1`, `R⁻¹ = Rᵀ`: inverting a rotation is a
transpose, and the engine uses `rot.transposed()` wherever it needs to push a world vector
into body-local space (for example in `collidePolygonCircle`). The general
`inverse()`/`solve()` exist for the *other* 2×2 matrices this engine builds: the effective
mass matrix `K` of a point-to-point joint and of a two-point contact block. Those are not
orthogonal and must be inverted numerically. `solve()` uses Cramer's rule, which for a
2×2 system is both cheaper and more accurate than `inverse() * b`.

**Code map.**

| Symbol | File / line | Notes |
| --- | --- | --- |
| `struct Mat22` | [Mat22.hpp:19](../Mat22.hpp#L19) | row-major `m00 m01 / m10 m11`, defaults to identity |
| `Mat22::rotation` | [Mat22.hpp:23](../Mat22.hpp#L23) | returns `{c, −s, s, c}` = the matrix above |
| `Mat22::transposed` | [Mat22.hpp:37](../Mat22.hpp#L37) | the inverse *for rotations only* |
| `Mat22::det` / `inverse` | [Mat22.hpp:38](../Mat22.hpp#L38) | `inverse()` asserts on a singular matrix |
| `Mat22::solve` | [Mat22.hpp:50](../Mat22.hpp#L50) | Cramer's rule; asserts on a singular matrix |

The `assert` matters: a singular `K` in a *joint* would abort a debug build, which is why
`Joint::safeSolve` checks the determinant first and returns `{}` instead (§5.10).

**Exercise 2.1.** Verify that `R(θ) * R(−θ) = I` and `det R(θ) = 1` from the entries
above. Then write down `rot.transposed() * (p − x)` in words: which space is `p` in, and
which space is the result in?

### 3.3 `AABB` — bounds, fat proxies and the perimeter heuristic

**Intuition.** Before testing two shapes properly you want a cheap reject. A box around
each shape gives that. In the broadphase the box is deliberately *fattened* so a slowly
moving body does not have to be reinserted into the tree every frame.

**Derivation.** For a circle the bounds are `x ± r`. For a polygon they are the min/max
over the world vertices, obtained from `position + rot * localVertex`. Two properties are
load-bearing:

* **Inclusive overlap.** `overlaps` uses `<=` on every comparison, so boxes that merely
  touch count as overlapping. Disjoint bounds *imply* disjoint shapes, but overlapping
  bounds do not imply contact — the narrowphase decides that. Being inclusive on the
  boundary can therefore never discard a real contact.
* **Perimeter is the 2D surface-area heuristic.** In 2D the "surface" of a box is its
  boundary length, `2(w + h)`, and the tree insertion cost uses growth of that length as
  a proxy for how much query traffic a subtree will attract.

**Code map.**

| Symbol | File / line | Notes |
| --- | --- | --- |
| `struct AABB` | [AABB.hpp:10](../AABB.hpp#L10) | `min`, `max`; `overlaps` inclusive; `contains`; `unite` |
| `AABB::fatten` | [AABB.hpp:21](../AABB.hpp#L21) | pads both corners by a margin (the tree uses 0.1) |
| `AABB::perimeter` | [AABB.hpp:28](../AABB.hpp#L28) | `2(w + h)`; the SAH cost in 2D |
| `computeAABB` | [AABB.hpp:35](../AABB.hpp#L35) | circle shortcut, otherwise min/max over world vertices |

**Exercise 3.1.** A unit box (1×1) and a unit circle (r = 0.5) at the same position: which
has the larger AABB perimeter, and by how much? Then explain why the tree's insertion cost
formula in [DynamicTree.hpp:131](../DynamicTree.hpp#L131) multiplies the parent's cost
growth by 2 before adding a child term.

### 3.4 `RigidBody` and `Shape` — state, mass properties, and the static policy

**Intuition.** A body is a position, an angle, and the rates of change of those two. Mass
and inertia say how hard it is to change them. A static body is one that cannot be
changed at all — and the engine implements that with *inverse* mass, so no formula needs
an `if`.

**Derivation — why inverse mass is stored.** An impulse `P` applied to a body of mass `m`
changes its velocity by `P/m`. If `m` is infinite (an immovable floor) the change is zero.
Rather than branching on `isStatic()` inside every formula, the engine stores `invMass =
1/m` and `invInertia = 1/I`: a static body simply stores `0`, and `velocity += P * 0` is a
no-op in exact arithmetic. Every impulse, Jacobian and effective-mass expression is then
linear in `invMass`, `invInertia` only.

**Derivation — polygon mass properties.** `makePolygon` computes area, centroid and
inertia from the vertices as a triangle fan from the origin. For CCW vertices
`p₀ … pₙ₋₁` with `pₙ = p₀`, each triangle `(0, pᵢ, pᵢ₊₁)` has signed area
`½ cross(pᵢ, pᵢ₊₁)`, and its centroid is `(pᵢ + pᵢ₊₁)/3`. Summing with the parallel-axis
theorem gives, for the *unit-density* body:

```
A  = ½ Σ cross(pᵢ, pᵢ₊₁)                                  (signed area; > 0 for CCW)
c  = (1 / 6A) Σ cross(pᵢ, pᵢ₊₁) (pᵢ + pᵢ₊₁)               (centroid)
I₀ = (1 / 12) Σ cross(pᵢ, pᵢ₊₁) (pᵢ·pᵢ + pᵢ·pᵢ₊₁ + pᵢ₊₁·pᵢ₊₁)   (about the origin)
```

Then, because the body must rotate about its *centre of mass* and not about the origin,
the parallel-axis theorem removes the offset, and the density `ρ = m/A` scales it:

```
I_centroid = I₀ − A·|c|²          I = ρ · I_centroid = (m / A) · I₀ − m·|c|²
```

The code computes `iCentroidUnit = ixx - area * c.lengthSq()` before rescaling and then
`inertia = (m / area) * iCentroidUnit`. Vertices are recentred on `c` afterwards, so the
stored polygon is centroid-at-origin and `position` is the centre of mass. That is why
`F = m·a` and `τ = I·α` hold with no correction terms anywhere else in the engine.

For the built-in factories the closed forms are:

| Factory | Mass properties |
| --- | --- |
| [`makeCircle(r, m, pos)`](../RigidBody.hpp#L109) | `I = ½ m r²` (solid disc) |
| [`makeBox(w, h, m, pos, angle)`](../RigidBody.hpp#L124) | a 4-gon through `makePolygon`, then `setAngle(angle)` |
| [`makePolygon(pts, m, pos)`](../RigidBody.hpp#L137) | the fan above; also forces CCW winding and builds outward normals `(e.y, −e.x)` |

`m <= 0` means static: the factories simply leave `mass = invMass = inertia = invInertia = 0`.
`setStatic()` zeroes all four *and* the velocities. `isStatic()` tests `invMass == 0` only,
which is sound because `setStatic` and both factories always clear `invInertia` together
with it.

**Derivation — `velocityAt` and `applyForce`.** The velocity of the material point at
lever arm `r` is `v + ω × r`; that single expression is what makes contact and joint rows
work, because the relative velocity of a contact is a difference of two of them:

```
v_P = velocityAt(r) = v + ω × r                    [RigidBody.hpp:103]
```

A world-space force `F` applied at world point `p` contributes `F` to the linear force and
`(p − x) × F` to the torque, because torque is defined about the centre of mass.

**Code map (`RigidBody.hpp`).**

| Symbol | Line | Notes |
| --- | --- | --- |
| `struct Shape` | [23](../RigidBody.hpp#L23) | `type`, `radius` (circle), local `verts` + outward unit `normals` (polygon) |
| `struct RigidBody` | [30](../RigidBody.hpp#L30) | the field list below |
| `isStatic()` | [63](../RigidBody.hpp#L63) | `invMass == 0`; the policy, not the guard |
| `setAngle(real)` | [65](../RigidBody.hpp#L65) | assigns `angle` **and** refreshes `rot` |
| `worldVertex` / `worldNormal` | [68](../RigidBody.hpp#L68) | `position + rot * v`, `rot * n` |
| `applyForce(F, p)` | [71](../RigidBody.hpp#L71) | wakes the body, adds `F` and `(p − x) × F` |
| `applyImpulse(P, r)` | [84](../RigidBody.hpp#L84) | `v += P·m⁻¹`, `ω += (r × P)·I⁻¹`; early-returns when static |
| `applyAngularImpulse` | [91](../RigidBody.hpp#L91) | z-only row write, same guard |
| `translate(d)` | [97](../RigidBody.hpp#L97) | the NGS position write, same guard |
| `velocityAt(r)` | [103](../RigidBody.hpp#L103) | `v + ω × r` |
| `wake()` | [54](../RigidBody.hpp#L54) | `awake = true; sleepTime = 0` |

The state is split linear/angular exactly as the physics is:

```cpp
Vec2 position, velocity, force;  real mass = 0, invMass = 0;
real angle, angularVelocity, torque, driveTorque, inertia, invInertia;
real friction = 0.4, restitution = 0.2, linearDamping = 0, angularDamping = 0;
bool bullet = false, awake = true;  real sleepTime = 0;
Mat22 rot;  Shape shape;
```

Two subtleties worth memorising:

* `driveTorque` is **persistent**. `torque` is cleared by `integrateVelocities` at the end
  of a step; `driveTorque` is not, so a wheel motor needs no per-frame re-application.
  This is also why the island pass treats a non-zero `driveTorque` as a disturbance.
* `force` is **excluded** from cache validation (§5.9): applying a force is a normal
  input, not an external edit.

**The immovable-body guard is load-bearing, not an optimization.** Every write helper
returns early when `isStatic()`. Writing `velocity += P * 0` would be numerically a no-op
but still a *write*, and two solver rows that share one floor would race on it under any
future parallel schedule. Keeping the floor untouched is what lets thousands of contact
rows share a single static body without spending a colour each (§5.14).

**Exercise 4.1.** Derive the row-fan formula for `I₀` for a rectangle of width `w`, height
`h` and unit density centred at the origin, and check it against the closed form
`(w²+h²)/12 · ρ·w·h`. The native test `testMassProperties` asserts `box I =
m(w²+h²)/12 = 1.2500` for the 1×1 unit-mass box.

**Exercise 4.2.** A 2×1 box has mass 4. Compute `invMass`, `inertia` and `invInertia`
using the formulas above. Then work out the velocity change of the material point at
`r = (1, 0)` when you apply `applyImpulse({0, 2}, {1, 0})`, and say which part of it is
linear and which is spin.

### 3.5 `Manifold` — contacts as data

**Intuition.** A contact is a place where two shapes touch, a direction, and how deep.
The solver also needs per-point scratch (lever arms, effective masses, accumulated
impulses), and that scratch is what makes the solver warm-startable.

**Derivation — what a manifold must carry.** The velocity-level constraint for one
contact point is `vn' ≥ 0` (not approaching). Solving it needs four numbers that do not
change during the sweeps and two that do:

* fixed per step: `rA`, `rB` (lever arms) and `massN`, `massT` (effective masses, §5.4),
  plus `targetVn` (the post-solve normal speed the row aims for, §5.7 and §5.8);
* varying per sweep: `accN`, `accT` — the *accumulated* impulses, which are the entire
  persistent state of the solver.

**Code map.**

| Symbol | File / line | Notes |
| --- | --- | --- |
| `struct ContactPoint` | [Manifold.hpp:9](../Manifold.hpp#L9) | `position`, `featureId`, `depth`, `rA`, `rB`, `massN`, `massT`, `targetVn`, `accN`, `accT` |
| `struct ContactManifold` | [Manifold.hpp:26](../Manifold.hpp#L26) | `bodyA`, `bodyB`, `normal` (A → B), `penetration`, `contactPoints[2]`, `indexA`, `indexB`, `count`, `friction`, `restitution` |

`count` is `1` or `2`: circle contacts have one point, polygon face contacts have two.
`penetration` is the deepest point's depth. `friction` and `restitution` are the *mixed*
material values (§5.6, §5.7) filled by the solver, not by the narrowphase.
`featureId` is geometric identity — which reference face, which incident edge, which
endpoint or clip intersection — and never a hash of a world position, so it survives
sliding (§5.9).

**The bit layout of `featureId`** (from [Collision.hpp:154](../Collision.hpp#L154)):

| Bits | Meaning |
| --- | --- |
| 63 | `flip`: whether B supplied the reference face |
| 32–62 | reference face index |
| 3–31 | incident face index |
| 2 | endpoint flag off / clip-side marker base |
| 0–1 | `0/1` = incident endpoints, `2/3` = the two clipping half-planes |

Polygon-vs-circle uses a smaller scheme: `(face << 2) | 1` for a face region and
`(vertex << 2) | 2` for a vertex region.

**Exact native layouts** (GCC 13.2 MinGW x64; `double` 8 bytes). These are *in-process*
layouts, not file formats — see §6 for why a WASM layout must be measured separately:

| Record | Offsets (bytes) | Size / align |
| --- | --- | --- |
| `ContactPoint` | position 0, featureId 16, depth 24, rA 32, rB 48, massN 64, massT 72, targetVn 80, accN 88, accT 96 | 104 / 8 |
| `ContactManifold` | bodyA 0, bodyB 8, normal 16, penetration 32, contactPoints 40, indexA 248, indexB 256, count 264, friction 272, restitution 280 | 288 / 8 |

**Exercise 5.1.** Two contacting boxes produce `featureId` values whose bits 32+ differ
but whose bits 0–1 are `0` and `1`. What can you conclude about the contact, and which
number tells the solver that a reused impulse from last frame belongs to the same physical
feature?

### 3.6 `World` — the container and its ownership

**Intuition.** `World` is the object you actually animate: bodies in a vector, joints in a
vector, one solver, one broadphase, one contact cache. `World::step(dt)` is the whole
engine in one call.

**Code map.**

| Symbol | File / line | Notes |
| --- | --- | --- |
| `class World` | [World.hpp:28](../World.hpp#L28) | public knobs and the step entry points |
| `gravity` | [World.hpp:30](../World.hpp#L30) | `{0, −9.81}`, y-up |
| `bodies`, `joints` | [World.hpp:31](../World.hpp#L31) | public `std::vector`s; indices are the identity |
| `solver` | [World.hpp:32](../World.hpp#L32) | the `ContactSolver`, which owns `SolverConfig` |
| `sleeping` | [World.hpp:34](../World.hpp#L34) | **default `false`**: sleeping is opt-in |
| `ccdStats` | [World.hpp:35](../World.hpp#L35) | per-step `CCDStats` report (§5.13) |
| `struct Timings` | [World.hpp:40](../World.hpp#L40) | `broadphase, narrowphase, islands, solve, integrate, ccd, bookkeeping` |
| `profiling` | [World.hpp:44](../World.hpp#L44) | default `false`; fills `timings` at wall-clock cost |
| `add(RigidBody)` | [World.hpp:48](../World.hpp#L48) | returns the new body's index |
| `struct Capacity` | [World.hpp:50](../World.hpp#L50) | logical limits: `bodies, joints, pairs, contacts, vertices` |
| `enum class StepStatus` | [World.hpp:53](../World.hpp#L53) | `Complete, Bodies, Joints, Vertices, Pairs, Contacts` |
| `prepare(Capacity)` | [World.hpp:56](../World.hpp#L56) | reserves every buffer; setup only (§6) |
| `step(real dt)` | [World.hpp:70](../World.hpp#L70) | throws `std::length_error` if a prepared capacity runs out |
| `tryStep(real dt)` | [World.hpp:75](../World.hpp#L75) | the allocation-free, status-returning entry point |
| `rows()` | [World.hpp:187](../World.hpp#L187) | the per-tick `RowSet` (indices only, §5.14) |
| `movableBodies()` | [World.hpp:190](../World.hpp#L190) | `char` per body: may the solver write it? (allocates; §5.14) |
| `manifolds()` | [World.hpp:197](../World.hpp#L197) | **raw `RigidBody*`** — do not add bodies during a step |
| `findPairs(bool)` | [World.hpp:261](../World.hpp#L261) | frozen pairs + broadphase staging (§4.3) |
| `correctPositions()` | [World.hpp:334](../World.hpp#L334) | the opt-in NGS pass (§5.8) |
| `CachedManifold`, `Snapshot` | [World.hpp:367](../World.hpp#L367) | pointer-free previous contacts; cache validation records |
| `cacheChanged()` | [World.hpp:387](../World.hpp#L384) | external-edit detection (§5.9) |
| `restoreImpulses` / `saveImpulses` | [World.hpp:402](../World.hpp#L402) | warm-start merge, then persist current contacts + snapshots |

The ownership rule is worth saying plainly, because it explains several APIs:
**impulses have exactly one owner.** Contact impulses live in `ContactPoint`, joint
impulses live in `Joint`, and nothing else keeps a second copy. `RowSet` holds indices and
ordinals only. `cache_` holds a *previous-frame* copy for warm starting, and `snapshots_`
plus the flat `vertices_` buffer exist solely to detect external edits without touching
the bodies themselves.

`World` also rejects bad input before it mutates anything: non-positive or non-finite `dt`
(`std::invalid_argument`), joint indices outside `bodies` (`std::out_of_range`), reversed
joint limits, and an invalid TGS configuration.

**Exercise 6.1.** Read `tryStep` ([World.hpp:75](../World.hpp#L75)) and list, in order,
every `return` that can happen before any body state is written. Then explain why a caller
is allowed to retry with a bigger `prepare()` after such a return (see §6.1).

---

## 4. One step, stage by stage

`World::step(dt)` is `tryStep(dt)` plus a throw on any non-`Complete` status. Here is
the pipeline with the exact code order, because the order is part of the physics. The
frame below is the default PGS path; the TGS branch replaces stages 7–9 and is described
in §5.12.

```
0. validate                       dt, joint indices, joint limits, TGS config, capacity
1. changed = mode switch || cacheChanged()        (external edit detection)
2. findPairs(!changed)             stage nextManifolds_   [frozen pairs, then tree pairs]
3. if (changed)                    clear cache, reset joint impulses, wake all (if sleeping)
4. if (sleeping) islands_.build(previous manifolds, joints); islands_.wakeConnected()
5. manifolds_.swap(nextManifolds_)
6. rows_.build(manifolds_, joints, bodies)          (the addressing seam)
7. integrateForces(each awake body, gravity, dt)
8. restoreImpulses(dt); solver.prepare(manifolds_, dt); joint.prepare(...)
9. for iterations: { joint.solve(...); solver.solveIteration(manifolds_) }
10. bullets ? advanceContinuous(bodies, dt)  :  integrateVelocities(each awake body, dt)
11. if (positionCorrection) correctPositions()
12. if (sleeping) islands_.sleep(bodies, dt)
13. saveImpulses(dt); previousMode_ = mode
```

**Why this order, in five notes.**

* **Stage 2 runs before stage 4 on purpose.** The island pass must see the *previous*
  frame's manifolds and the *previous* frame's post-solve velocities. If it ran after the
  force pass it would see gravity's `g·dt ≈ 0.16 m/s` increment on every resting body and
  wake every island forever. This was a real bug; see [Islands.hpp:16](../Islands.hpp#L16).
* **Stage 8 is two loops for a reason.** `restoreImpulses` only *seeds* `accN`/`accT`
  numbers. `solver.prepare` then computes every `targetVn` from the *pre-impulse*
  velocities, and only after all of them exist does a second loop actually apply the
  seeded impulses. Seeding one contact must not contaminate another contact's pre-solve
  closing speed (that speed drives restitution, §5.7).
* **Stage 9 interleaves joints and contacts** in one sweep loop, joints first. Sequential
  impulses is Gauss-Seidel, not a simultaneous solve: each row sees the corrections the
  previous rows already made.
* **Stage 10 is a fork, not an addition.** If *any* body has `bullet = true`, the whole
  world advances under the event loop of `advanceContinuous` instead of the plain loop, so
  that impacts are resolved inside the step (§5.13).
* **Stage 13 saves after stage 11/12**, so the cached geometry and impulses describe the
  *final* poses of the step. The comment in the code — "ordering is load-bearing: projection,
  then sleep, then snapshot" — is exactly this.

**Timing buckets.** With `profiling = true`: `broadphase` is the tree/proxy sync, `narrowphase`
is the `collide` loop, `islands` is both island passes, `solve` covers joint prepare plus all
sweeps, `integrate` is force integration plus pose/velocity integration, `ccd` is the sweep,
and `bookkeeping` absorbs the row build, the NGS pass and the impulse-cache save. `timings`
is reset at the top of `findPairs`, so a failed capacity return leaves partial derived
timings, not a committed tick.

**Exercise 7.1.** Trace one step of `Scenes.hpp::makeStack(3)` by hand on paper: how many
manifolds, how many contact points, how many solver rows (§5.14), and which stage writes
`velocity` for the bottom box.

**Exercise 7.2.** Set `world.profiling = true` on the 100-plank bridge scene
(`Showcase.hpp::makeBridge(100)`), run 300 steps, and print the seven `timings` fields.
Which bucket dominates, and does that match the cost table in §4.1?

### 4.1 Cost summary

| Stage | Cost driver | Where |
| --- | --- | --- |
| Broadphase | proxy sync + queries + `O(P log P)` sort of `P` candidate pairs | [`Broadphase::findPairs`](../Broadphase.hpp#L21), [`DynamicTree::query`](../DynamicTree.hpp#L42) |
| Narrowphase | one `collide` per candidate pair | [Collision.hpp](../Collision.hpp#L212) |
| Prepare | once per contact point | [Solver.hpp](../Solver.hpp#L70) |
| Solve | `iterations × (joints + contacts)`, joints first | [Solver.hpp](../Solver.hpp#L136), [Joint.hpp](../Joint.hpp#L112) |
| Integrate | `O(N)` | [Integrator.hpp](../Integrator.hpp#L24) |
| NGS | `positionIterations × (joints + contacts)` **with a second `collide`** | [World.hpp](../World.hpp#L334) |
| Cache | `O(C)` merge/rebuild, `O(N + V)` snapshots | [World.hpp](../World.hpp#L402) |

---

## 5. The mathematics

Everything below is derived in the engine's own conventions (§1.1) and then tied to the
symbol that implements it. Where a number appears, it is a measurement from this tree and
the receipt is in [BENCHMARKS.md](BENCHMARKS.md) or the section it is quoted in.

### 5.1 Rigid-body kinematics: one state, three degrees of freedom

**Derivation.** A rigid body in 2D has three degrees of freedom: `x ∈ ℝ²` and `θ ∈ ℝ`.
Rigidity means every material point `p` keeps a fixed body-local offset `a = Rᵀ(p − x)`, so
at time `t`:

```
p(t) = x(t) + R(θ(t)) · a
```

Differentiating with respect to time, and using `Ṙ(θ) = ω · R(θ) · J` where
`J = perp` (a 90° rotation), gives the velocity of that material point:

```
v_P = ẋ + Ṙ a = v + ω · R J a = v + ω × (R a) = v + ω × r,     r = R a
```

That is [`velocityAt`](../RigidBody.hpp#L103). Two consequences the whole engine rests on:

* the **lever arm** `r` is a world vector, not a local one, and it must be recomputed when
the pose changes. PGS recomputes `rA`/`rB` once per step in `prepare`; TGS recomputes them
once per *slice* in `refreshTemporal` (§5.12), because a slice moves the pose.
* a solver row only ever needs velocities, never positions, to compute a *velocity*
  error — which is why the PGS sweep in §5.4 is a pure function of `v` and `ω`.

**Exercise 8.1.** Show that the kinetic energy of a rigid body is
`½ m|v|² + ½ I ω²` and that an impulse pair `(−P, +P)` applied at the same world point on
two bodies conserves total momentum but not kinetic energy. Which term in
[`resolveImpact`](../Continuous.hpp#L214) decides how much energy a bullet keeps?

### 5.2 Integration: why symplectic Euler, and why the step is split in two

**Intuition.** Advance velocity, then position, using the *new* velocity. It costs the same
as the naive alternative and it does not blow up.

**Derivation — the two Euler variants.** Write the state as `(x, v)` and a force law that
depends only on position, `a = F(x)/m`:

```
Explicit Euler:   v' = v + a(x)·dt      x' = x + v·dt        (position uses the OLD v)
Symplectic Euler: v' = v + a(x)·dt      x' = x + v'·dt       (position uses the NEW v)
```

For a harmonic oscillator `a(x) = −ω²x`, symplectic Euler gives
`x' = x + v·dt − ω²x·dt²`, `v' = v − ω²x·dt`. Its Jacobian is

```
| 1 − ω²dt²    dt |
|   −ω²dt       1 |      det = (1 − ω²dt²) + ω²dt² = 1
```

so the map preserves phase-space area *exactly*: the numerical trajectory is the exact
trajectory of a nearby "shadow" Hamiltonian, and the energy error is bounded rather than
growing. Explicit Euler has `det = 1 + ω²dt² > 1`, which is why it spirals outward.

For an arbitrary position-only force the same cancellation happens (the `a'·dt²` term from
the position update cancels the `−a'·dt²` from the determinant), so `det = 1` is a property
of the *split*, not of the pendulum. Two things break it honestly:

* **velocity-dependent damping.** The engine uses the *implicit* form
  `v' = v / (1 + c·dt)`, derived from `v' = v − c·v'·dt`. That is unconditionally stable
  (the factor is `1/(1+c·dt) ∈ (0,1]` for all `dt > 0`) but it contracts area deliberately —
  energy *should* leave.
* **impulses.** Contact and joint impulses are non-smooth corrections applied between the
  two halves of the step. Order matters; §5.4 shows the impulse formulation.

**Derivation — why the step is split.** Sequential impulses must correct *velocities*
before the pose advances, otherwise the correction is applied one step late and appears as
position drift. So the engine exposes two functions and calls the solver in between:

```
integrateForces(b, g, dt)          // v += (F/m + g) dt, then implicit damping
… restoreImpulses / prepare / sweeps / (NGS or CCD) …
integrateVelocities(b, dt)         // x += v dt, θ += ω dt, clear force/torque
```

Gravity is an acceleration and must **not** be multiplied by `invMass`; `force` is
divided by mass. `integrateVelocities` advances the pose for non-static bodies and clears
`force`/`torque` for *every* body (including static and asleep ones, which never ran the
force pass). `driveTorque` is deliberately not cleared.

**Code map.**

| Symbol | File / line | Notes |
| --- | --- | --- |
| `integrateForces(b, gravity, dt)` | [Integrator.hpp:24](../Integrator.hpp#L24) | the velocity update, damping, no `invMass` on gravity |
| `integrateVelocities(b, dt)` | [Integrator.hpp:36](../Integrator.hpp#L36) | pose advance via `setAngle`, clears force/torque |
| `symplecticEuler(b, g, dt)` | [Integrator.hpp:46](../Integrator.hpp#L46) | the two calls together, for standalone use |
| `ParticleState`, policies | [IntegratorStudy.hpp](../IntegratorStudy.hpp#L6) | `SymplecticPolicy`, `VerletPolicy`, `RK4Policy` for the smooth-force study |

**Measured.** `testSymplecticEnergy` runs a harmonic oscillator for 1000 s and asserts
`late max == early max` in the receipt (`52.6316 vs 52.6316`, `E0 = 50.0`) with explicit
Euler shown diverging to `E/E0 = 4.4e8` in 20 s for contrast. `testIntegratorPolicies`
measures step-halving ratios `2.0403` (symplectic), `4.0031` (Verlet), `16.1013` (RK4):
first, second and fourth order as expected. RK4's order says nothing about contact
accuracy — it is a smooth-force result only.

**Exercise 9.1.** Derive the damping factor. Start from `v' = v − c·v'·dt`, solve for
`v'`, and find the value of `c` for which the velocity halves in 1 s at `dt = 1/60`.

**Exercise 9.2.** In `study_integrators.cpp`, run the study and re-derive the four
ratios by hand from the printed errors. Then switch the acceleration to depend on velocity
(`a = −ω²x − γv`) and explain which of the three policies is no longer time-reversible.

### 5.3 Contact generation: SAT, reference faces, and clipping

**Intuition.** Two convex shapes are apart if you can slide a straight line between them.
A box has four candidate lines (its faces); test all of both shapes' faces and keep the one
with the *smallest* overlap. That direction is the contact normal. Where the shapes touch
is found by clipping one edge against the other's sides.

**Derivation — the separating axis theorem in 2D.** Two convex sets are disjoint iff
there is a unit vector `n` (an axis) such that their projections onto `n` do not overlap.
For two convex polygons in 2D, the only axes that can separate them are the face normals of
their edges: the 3D edge×edge cross-product axes collapse into the face normals. For two
circles the only axis is the centre difference.

Rather than projecting both shapes onto every axis, the code uses the equivalent
*face-based* form. For each face `i` of A with outward unit normal `nᵢ` through vertex `vᵢ`,
the signed separation of B along that face is

```
sᵢ = minⱼ  nᵢ · (bⱼ − vᵢ)          (B's support point in the −nᵢ direction)
```

`sᵢ ≥ 0` means B lies entirely in front of face `i` — a gap — so the pair is separated.
Otherwise the *least negative* `sᵢ` is the face with the least penetration, and its normal
is the minimum-translation direction. The same computation runs for B's faces; whichever of
the two best separations is larger supplies the **reference** face, because the deeper
overlap means B's face is the more reliable witness of the contact. A `kFaceTolerance =
1e-4` bias prefers A's face on near-ties so that a resting box does not oscillate between
two reference decisions frame to frame (temporal coherence).

**Derivation — the incident face and the clip.** The **incident** face is the face of the
*other* polygon most anti-parallel to the reference normal: `argminᵢ n_ref · nᵢ`. Clipping
its edge against the two side planes of the reference face is Sutherland-Hodgman against a
single half-plane, `dot(n, p) − c ≤ 0`. Two cases per endpoint: if the point is already
inside the plane it is kept; if the edge crosses the plane at `d₁·d₂ < 0`, the intersection
is inserted at

```
alpha = d₁ / (d₁ − d₂)     p = p₁ + alpha · (p₂ − p₁)
```

which is just linear interpolation between the two signed distances. Both `d₁` and `d₂`
being positive yields no points and the pair is rejected by the `clip(...) < 2` early
return. Surviving points are kept only if they lie below the reference plane,
`depth = refC − n·p ≥ 0`, and `penetration` is the maximum such depth.

**Why the clip produces two points, and why that is the whole point.** A resting box on a
floor has a face-face contact; the clip returns both endpoints of the overlapping edge
span. Two contact points at the same normal give the solver two *independent* rows whose
lever arms differ, and therefore a restoring torque: a box that starts to tilt is pushed
back by the more deeply loaded corner. With a single point the box would balance on it and
rotate freely. `count == 2` is also what enables the 2×2 block solve of §5.5.

**Circle cases.** `collideCircles` is one point, depth `rA + rB − |d|`, with an arbitrary
normal chosen when the circles are concentric rather than dividing by zero.
`collidePolygonCircle` works in the polygon's local frame
(`rot.transposed() * (C.position − P.position)`, the cheap inverse rotation of §3.2), does
face-based SAT with a `radius` slack, and then chooses between the face region and the
vertex regions at the ends of the winning edge with two dot-product tests. The normal is
rotated back to world space before being returned.

**Convention.** `collide(A, B, m)` always fills `m.normal` pointing **A → B**; the
polygon-circle overload is called with the arguments swapped when needed and the normal is
negated. Getting this backwards makes the solver *pull* instead of push, which is one of
the fastest ways to produce an explosion.

**Code map.**

| Symbol | File / line | Notes |
| --- | --- | --- |
| `findMaxSeparation(A,B,faceIdx)` | [Collision.hpp:39](../Collision.hpp#L39) | `maxᵢ minⱼ nᵢ·(bⱼ − vᵢ)` for one polygon's faces |
| `detail::clip(n, c, face, ids, crossingId)` | [Collision.hpp:54](../Collision.hpp#L54) | Sutherland-Hodgman against one half-plane; carries feature ids through |
| `collideCircles` | [Collision.hpp:79](../Collision.hpp#L79) | one point, concentric fallback |
| `collidePolygonCircle` | [Collision.hpp:103](../Collision.hpp#L103) | local-frame SAT + Voronoi region choice |
| `collidePolygons` | [Collision.hpp:154](../Collision.hpp#L154) | reference/incident selection, clip, feature ids |
| `collide(A, B, m)` | [Collision.hpp:212](../Collision.hpp#L212) | dispatch; guarantees A → B |

**Exercise 10.1.** Two unit boxes overlap by 0.02 m, one rotated 0.1 rad. Without running
anything, decide which body supplies the reference face and by how much the two
separations differ. Then write a 20-line program that prints `m.penetration`, `m.count` and
both `featureId` values, and check your prediction.

**Exercise 10.2.** Why does `clip` return `0` when both endpoints are outside the same
half-plane, and what would a naive implementation do instead? What symptom would appear in
a running scene?

### 5.4 Impulses and effective mass

**Intuition.** To stop two bodies approaching, apply an impulse `P = λn` to B and `−P` to
A. The effective mass `K` tells you how much relative normal speed one unit of `λ` buys.

**Derivation.** Start from `velocityAt` (§5.1) and apply the impulse pair:

```
vB' = vB + mB⁻¹ P           ωB' = ωB + IB⁻¹ (rB × P)
vA' = vA − mA⁻¹ P           ωA' = ωA − IA⁻¹ (rA × P)
```

The relative normal speed is `vn = (vB + ωB × rB − vA − ωA × rA) · n`. Substituting and
using `(ω × r) · n = ω (r × n)`:

```
vn' = vn + λ · K,      K = mA⁻¹ + mB⁻¹ + IA⁻¹ (rA × n)² + IB⁻¹ (rB × n)²
```

so the impulse that achieves a target normal speed is

```
λ = (target − vn) / K,     target = −e·vn₀   (Newton restitution, §5.7)
```

and the classic formula `λ = −(1+e)·vn₀ / K` falls out. The engine stores `massN = 1/K`
instead of `K`, because the sweep then computes `λ = (target − vn) · massN` — a multiply
rather than a divide, per point, per sweep.

For a **two-point** manifold the two rows share the same bodies, so a single row's `K` is
not the whole story: the normal impulse applied at point 1 also changes the normal speed
at point 2. That coupling is the off-diagonal term

```
K12 = mA⁻¹ + mB⁻¹ + IA⁻¹ (rA₁ × n)(rA₂ × n) + IB⁻¹ (rB₁ × n)(rB₂ × n)
```

and the two-point row is the 2×2 LCP [`K = [[k11, k12], [k12, k22]]`](../Solver.hpp#L174).
Ignoring `K12` (solving each point independently) is what makes stacks *sway*: the two
supports fight each other over two sweeps instead of being solved together once.

**Code map.**

| Symbol | File / line | Notes |
| --- | --- | --- |
| `massN`, `massT` computation | [Solver.hpp](../Solver.hpp#L70) | `1/K` for the normal and tangent, per point, once per step |
| `k12` coupling and `K{d}` | [Solver.hpp](../Solver.hpp#L174) | two-point block matrix, `count == 2` only |
| `Joint` point block `K_` | [Joint.hpp](../Joint.hpp#L89) | the same 2×2 structure for revolute/weld/mouse anchors |

**Exercise 11.1.** Derive `K` for a circle resting on a static floor. Simplify until you
find that `K` is independent of the contact point's horizontal position, and explain why
this is *not* true for a box (hint: `r × n`).

**Exercise 11.2.** A 1 kg box (1×1, `I = 1/6`) touches the floor at `r = (±0.5, −0.5)` with
`n = (0, 1)`. Compute `k11`, `k22` and `k12`. What does the sign of `k12` mean physically?

### 5.5 The LCP, accumulated clamping, and why the running total is clamped

**Intuition.** Contacts are one-sided: they push, never pull. Solving each contact once
leaves the others wrong, so iterate — and remember the *total* impulse so far.

**Derivation — the complementarity problem.** For each contact row the physics is

```
λ ≥ 0        (impulses push, never pull)
vn' ≥ 0      (no penetration velocity after the solve)
λ · vn' = 0  (a contact with no load has no separating speed of its own)
```

Together these are a linear complementarity problem (LCP). Gauss-Seidel solves it by
visiting rows in turn; the trick that makes the iteration converge to the LCP solution
rather than to a per-increment clamp is to clamp the **accumulated** impulse:

```
newAcc = max(accN + λ, 0);   λ = newAcc − accN;   accN = newAcc;
```

Clamping each *increment* instead freezes overshoot: a row that has already pushed too
hard keeps its overshoot forever, and in a chain it can leave a supporter permanently
loaded. Clamping the running total lets a later sweep *unload* an over-corrected row
(`λ` becomes negative, `accN` decreases but stays ≥ 0).

**Derivation — why the ordering is Gauss-Seidel, not Jacobi.** Each row reads the
velocities as they are *right now*, including the corrections of the rows already visited
this sweep. That is why a single sweep is not a solve, why `iterations` exists, and why
the *order* of rows is part of the result (§5.14 makes that order canonical so a schedule
can be computed from it).

**Code map.**

| Symbol | File / line | Notes |
| --- | --- | --- |
| `solver.solve` | [Solver.hpp](../Solver.hpp#L132) | `iterations` sweeps of `solveIteration` |
| `solveIteration` | [Solver.hpp](../Solver.hpp#L136) | friction first, then the normal row(s) |
| accumulated clamp | [Solver.hpp:204](../Solver.hpp#L204) | `max(accN + λ, 0)` with the delta re-applied |
| block-solve guard | [Solver.hpp:174](../Solver.hpp#L174) | `det > 1e-6 · k11 · k22` before trusting the 2×2 solve |

**Measured.** With warm starting but *scalar* normal rows the 20-box stack fails: the
suite retains the failing configuration as a comparison row (`0.506678` late speed,
`0.492979` drift), while the block solve holds the same stack under `1e-6`. The row exists
precisely so the fix cannot be quietly forgotten.

**Exercise 12.1.** For a single box resting on the floor with `K = 2` and `vn₀ = −1`, run
two sweeps by hand with `target = 0`. What is `accN` after each sweep, and what happens if
you clamp the increment instead?

### 5.6 Friction: a cone, and why it is solved before the normal

**Derivation.** Coulomb friction limits the tangential impulse to `|λ_t| ≤ μ·λ_n`. Along
`t = (n.y, −n.x)` the increment is `λ_t = −vt · massT` (the same effective-mass recipe as
§5.4 with `t` in place of `n`), and the accumulated tangent impulse is clamped into the
cone:

```
newAccT = clamp(accT + λ_t, −μ·accN, +μ·accN)
```

The clamp is evaluated with the *current* `accN`, which is why the normal impulse from the
previous sweep is what a friction row uses as its cone bound. Friction runs **before** the
normal row within a sweep so that the non-penetration constraint has the last word on the
velocity: friction may not create penetration.

Material mixing is `μ = sqrt(μA·μB)` — the geometric mean, which is symmetric and gives
`μ = 0` when either surface is frictionless — and restitution uses `max(eA, eB)`, i.e. the
bouncier material wins.

What is **not** modelled: rolling friction, friction torque (the tangential impulse is a
scalar applied along `t`, with no separate angular row), and anisotropic friction. The
slope test in `testFrictionSlope` is the acceptance: on a 0.35 rad slope with `tan θ =
0.365`, `μ = 0.60` holds (slid 0.001 m in 3 s) and `μ = 0.05` slides 13.14 m.

**Code map.**

| Symbol | File / line | Notes |
| --- | --- | --- |
| material mixing | [Solver.hpp:82](../Solver.hpp#L82) | `sqrt(μA·μB)`, `max(eA, eB)` |
| friction sweep | [Solver.hpp](../Solver.hpp#L136) | per point, before the normal row |
| cone clamp | [Solver.hpp](../Solver.hpp#L136) | `± μ · accN` with the delta applied |

**Exercise 13.1.** Derive the *static friction* condition from the slope test: for a block
of mass `m` on a slope of angle `θ`, show that it holds iff `μ ≥ tan θ`. Then explain why
the same test needs an angle that is not exactly `atan(μ)` to be a meaningful assertion.

### 5.7 Restitution, and the speed threshold that kills jitter

**Derivation.** Newton restitution says the outgoing normal speed is `−e` times the
incoming: `vn' = −e·vn₀`. Substituting into §5.4 gives `target = −e·vn₀`, and the
important word is **pre-solve**: `vn₀` must be measured *before* any impulse is applied,
otherwise the bounce feeds on its own correction. That is why `prepare` computes all
targets in one loop and applies the warm-start impulses in a second loop.

Two refinements matter in practice:

* **Threshold.** Restitution is applied only when the closing speed exceeds
  `cfg.restitutionThreshold` (default `1` m/s): `bounce = vn₀ < −threshold ? −e·vn₀ : 0`.
  A resting box has a tiny negative `vn₀` every frame because of gravity; without the
  threshold it would bounce microscopically forever, which is visible as jitter.
* **Combination with Baumgarte.** The final target is `max(bounce, push)`, not
  `bounce + push`: the two are alternative reasons to separate, not additive ones.

**Code map.**

| Symbol | File / line | Notes |
| --- | --- | --- |
| `restitutionThreshold` | [Solver.hpp:61](../Solver.hpp#L61) | default `1` m/s |
| pre-solve `vn0` and `bounce` | [Solver.hpp:102](../Solver.hpp#L102) | measured before any impulse is applied |
| `push` and `targetVn = max(...)` | [Solver.hpp:103](../Solver.hpp#L103) | bounce vs Baumgarte, combined with `max` |
| bullet restitution | [Continuous.hpp:222](../Continuous.hpp#L222) | the sweep uses its own hard-coded `−1.0` threshold |

**Exercise 14.1.** A ball with `e = 0.8` is dropped from 5 m. Using energy conservation,
predict the first rebound height with no threshold, then with `restitutionThreshold = 1`.
Explain the difference between the two predictions. The suite measures a first rebound of
`3.27` m for its own drop against an ideal of `3.2`.

### 5.8 Position error: Baumgarte versus NGS

**Intuition.** The velocity solve removes the velocity that pushed the bodies together; it
does not undo the overlap that already exists. Two ways to fix that: push them apart using
*velocity* (cheap, injects energy) or move their *poses* directly (more expensive, adds no
energy).

**Derivation — Baumgarte.** Let `C = −depth` be the position error. Demanding that it decays
exponentially, `Ċ = −(β/dt)·C`, adds a bias to the velocity target:

```
target vn ≥ (β/dt) · max(depth − slop, 0)
```

`β` (default `0.2`) is how fast the error is removed; `slop` (default `0.01` m) is an
allowed overlap, which keeps contacts *persistent* frame to frame instead of flickering
on and off at the boundary. The cost is real: the bias is a velocity that would not
exist in the physical system, so it adds kinetic energy. On one overlapped zero-gravity box
the suite measures **+2.599200 J injected** with a residual depth of `0.162000`.

**Derivation — NGS (nonlinear Gauss-Seidel).** Leave velocities alone and update the poses
instead, using the same effective mass as a velocity row but in position units:

```
depth' = depth − K·λ_pos        ⇒     Δx_A = −mA⁻¹ n·λ_pos,  Δθ_A = −IA⁻¹ (rA × n)·λ_pos
λ_pos = β · clamp(depth − slop, 0, 0.2) / K
```

Because `K` and `rA` change as the poses move, the operation is repeated
`positionIterations` times (default 10), and **each iteration re-runs `collide`** — that is
what makes it nonlinear. The `clamp(..., 0.2)` cap bounds how far one iteration may move a
body, which is what stops an over-deep overlap from launching it. The same box measures
**0 J injected** and a residual depth of `0.022215`.

NGS is **opt-in** (`positionCorrection`, default `false`) and, when it is on, `prepare`
sets `push = 0` so the same overlap is not corrected twice by two different mechanisms.
In TGS the flag is ignored: TGS uses refreshed velocity-level bias (the TGS normalisation
of §5.12) and does not run the NGS pass at all.

**Code map.**

| Symbol | File / line | Notes |
| --- | --- | --- |
| `positionCorrection` (default `false`), `positionIterations = 10`, `positionBeta = 0.3`, `baumgarte = 0.2`, `slop = 0.01` | [Solver.hpp:56](../Solver.hpp#L56) | all of the above knobs |
| `push` suppressed under NGS | [Solver.hpp:103](../Solver.hpp#L103) | `positionCorrection ? 0 : baumgarte/dt` |
| `World::correctPositions` | [World.hpp:334](../World.hpp#L334) | joint projection + re-collide + pose update; velocities untouched |
| joint projection | [Joint.hpp:161](../Joint.hpp#L161) | distance/soft-distance, prismatic, revolute limits, weld |

**Exercise 15.1.** With `β = 0.3`, `dt = 1/60` and an initial depth of `0.2` m, compute the
depth after 1, 2 and 10 Baumgarte steps (ignore the impulse solve). Then explain why the
`slop` term makes the equilibrium depth *non-zero* rather than zero.

**Exercise 15.2.** Turn NGS on in `makeStack(20)` and measure the settled height with and
without it. Why does NGS `positionBeta = 0.3` need fewer `positionIterations` than the
default when velocities are already correct?

### 5.9 Warm starting, persistent manifolds, and cache invalidation

**Intuition.** Yesterday's answer is a good first guess for today's. Seed the accumulated
impulses from the previous frame and PGS starts near the solution, so a stack converges in
fewer sweeps — provided you can tell which impulse belongs to which contact.

**Derivation — why seeding helps.** PGS converges to the LCP solution (§5.5) regardless of
the starting point, but the *rate* depends on the start. A resting stack's solution changes
very little between frames, so seeding turns a 30-sweep solution into a handful. Seeding
must be an impulse that is *applied* exactly once per step: the engine multiplies the
previous impulse by `dt / previousDt` (so the magnitude still means "this much momentum"
when the timestep changes) and applies it in the second loop of `prepare`.

**Derivation — identity across frames.** Two problems must be solved before an impulse can
be reused:

1. **Which pair?** Cache entries are keyed by `(indexA, indexB)` and merged against the
   current manifold list with a single forward cursor over two lexicographically sorted
   lists, because a body-index pair is stable within a step ([`restoreImpulses`](../World.hpp#L402)).
2. **Which point?** A manifold has up to two points, and contact points are geometric
   events, not slots. The engine compares `featureId` (§3.5), which encodes the reference
   face, the incident face and which endpoint or clip intersection the point is. A point
   that slid along the same face keeps its id; a point on a different face gets a different
   id and its impulse is *not* reused. That is why `featureId` is never a hash of a world
   position.

A cached manifold is also rejected when its normal disagrees with the current one
(`dot(previous.normal, m.normal) < 0.95`): the contact has genuinely rotated, so the old
normal impulse is in the wrong direction.

**Derivation — when the cache is not trustworthy.** Any public edit to a body (position,
velocity, angle, angular velocity, inverse mass/inertia, shape type, radius, or any local
vertex) makes cached impulses meaningless, and a *reordered or erased* body makes the
`(indexA, indexB)` keys point at the wrong bodies. `cacheChanged()` therefore compares a
`Snapshot` of the last post-step state (plus a flat copy of every body's local vertices)
against the current bodies, and on any difference `tryStep` clears the cache, resets every
joint's impulses, and — if sleeping is on — wakes every non-static body.

Two deliberate exemptions:

* **`force` is not compared.** Applying a force is a normal input, not an external edit.
* **A sleeping body with unchanged `invMass` is skipped.** A sleeping body cannot have
  moved, and the documented rule is: *wake a body before editing its state or shape.*

**Persistent manifolds.** `saveImpulses` rebuilds the cache from the contacts present
*this* frame only, so a manifold that vanished does not resurrect a stale impulse. Contacts
survive as long as the shapes stay overlapped (helped by `slop`), which is what makes a
resting stack's contact graph stable across frames.

**Code map.**

| Symbol | File / line | Notes |
| --- | --- | --- |
| `restoreImpulses(dt)` | [World.hpp:402](../World.hpp#L402) | merge cache by `(indexA,indexB)`, `dot ≥ 0.95`, `featureId` match, `dt/previousDt` ratio |
| `saveImpulses(dt)` | [World.hpp:424](../World.hpp#L424) | rebuild cache from current contacts, refresh snapshots, set `previousDt_` |
| `cacheChanged()` | [World.hpp:387](../World.hpp#L384) | exact-value comparison + sleeping shortcut |
| `CachedManifold` | [World.hpp:367](../World.hpp#L367) | pointer-free previous contact record |
| `Snapshot` | [World.hpp:374](../World.hpp#L374) | state + flat vertex offsets for validation |
| `clearContactCache()` | [World.hpp:184](../World.hpp#L184) | the manual escape hatch |

**Measured.** `testWarmStarting` holds a 20-box stack for 20 simulated seconds at
`speed 0.00000, drift 0.00000, tilt 0.00000, depth 0.01000` (top at `19.3878` vs ideal
`19.5`) and a 15-row pyramid at `top 14.3791` vs `14.5`. The stacking depth equal to `slop`
is not an accident: it is the equilibrium of §5.8.

**Exercise 16.1.** Disable warm starting (`warmStarting = false`) on `makeStack(20)` and
run 20 s. Record the top box's height and residual speed, then re-enable it. Explain the
difference in terms of §5.5's starting point.

**Exercise 16.2.** Find the four conditions under which `restoreImpulses` refuses to seed
an impulse and explain, for each, what would go wrong physically if the refusal were
removed.

### 5.10 Joints: one struct, five constraints

**Intuition.** A joint is a constraint between two bodies: keep a length, keep a point
coincident, allow only sliding, or lock the relative angle. The engine implements them in
one `Joint` struct with a `JointType` tag so they interleave with contacts in the same
sweep loop.

**Derivation — the pattern.** Every joint row is the same three-step recipe as a contact
row: build the Jacobian (which lever arms and axes the constraint touches), form the
effective mass, then apply an impulse along the constraint direction so that the constraint
velocity becomes its target. What differs is the constraint function `C`:

| Type | Constraint `C` | Row shape | Notes |
| --- | --- | --- | --- |
| `Distance` | `\|pB − pA\| − L` | scalar along `axis` | bilateral, no clamping; optionally soft |
| soft distance | same, with `γ` and a bias | scalar | `frequency > 0` selects it |
| `Revolute` | `pA + rA − (pB + rB) = 0` | 2×2 point block | optional speculative limits and a torque-capped motor |
| `Prismatic` | offset along `perp(axis)` = 0 | scalar | includes the rotating-axis term; `lockAngle` decides rotation |
| `Weld` | revolute + angular lock | sequential point then angular | *not* a coupled 3×3 solve |
| `Mouse` | soft point constraint to `target` | 2×2 with `γ` | frequency-based spring, force-capped |

**Derivation — soft constraints (the distance spring and the mouse joint).** A spring of
stiffness `k` and damping `c` is not unconditionally stable if implemented explicitly (the
`k·dt²` term can exceed 2). The engine integrates it implicitly, which produces two derived
quantities used by the sweep:

```
ω = 2π f,   k = m_eff ω²,   c = 2 ζ m_eff ω
γ  = 1 / [ dt (c + dt k) ]          (the regularisation added to the effective mass)
β  = dt · k · γ                     (the bias used in the target velocity)
```

so `massN → 1/(K + γ)` and a violation `C` produces the bias `β·C`. With `ζ = dampingRatio`
(default `0.7`) and `frequency = f`, the joint behaves as a spring-damper independent of the
step size. `Mouse` uses the *body's* mass in the same formulas, and a force cap
(`dt · maxForce` per step) so a dragged body cannot be yanked with unbounded force: the
accumulated impulse is `limited(...)` in `prepare` and again in `solve`.

**Derivation — limits are speculative.** A revolute limit is a one-sided constraint, and
the engine treats it with the same trick as the contact gap: for a *positive* gap it uses
`bias = 1 · gap/dt`, which allows motion up to the limit but not through it;
for a violation it uses `bias = β · gap/dt` and restores the limit over a few frames. That
is the pair of expressions in [`Joint::solve`](../Joint.hpp#L112). Prismatic and weld joints
lock the relative angle with the same `0.2/dt` bias, and `lockAngle = false` deliberately
disables that locking so a wheel on a prismatic axle stays free to spin (the car scene
relies on it).

**Derivation — the rotating axis of a prismatic joint.** A prismatic joint constrains the
offset along `perp(axis)` where `axis` rotates with body A. Differentiating the constraint
`C = d · perp(axis)` adds a term that a naive implementation misses:
`Jω_A = −(rA + d) × perp(axis)`, which is why the code builds `angularA_` from `rA_ + d`
and not from `rA_` alone (see the prismatic block of [`Joint::prepare`](../Joint.hpp#L46)).

**Position projection.** `correctPosition` runs in the NGS loop for the rigid distance
joint, the prismatic joint (perpendicular offset only — motion *along* the axis is the
joint's purpose), revolute limits (angle error outside the limit) and the weld. Soft and
mouse joints are skipped: their spring is *supposed* to stretch. Scalar projection uses
`0.4 · clamp(error, ±0.2) / K`; the point block solves a 2×2 system and clamps the offset.

**Code map.**

| Symbol | File / line | Notes |
| --- | --- | --- |
| `enum class JointType` | [Joint.hpp:6](../Joint.hpp#L6) | `Distance, Revolute, Mouse, Weld, Prismatic` |
| `struct Joint` | [Joint.hpp:8](../Joint.hpp#L8) | state + factories + `prepare`/`solve`/`correctPosition`/`resetImpulses` |
| factories | [Joint.hpp:22](../Joint.hpp#L22) | `distance`, `revolute`, `mouse`, `weld`, `prismatic` — each validates its inputs |
| `prepare(bodies, dt, ratio)` | [Joint.hpp:46](../Joint.hpp#L46) | lever arms, effective masses, warm-start scaling by `ratio` |
| soft spring `γ`, `β` | [Joint.hpp:78](../Joint.hpp#L78) | the implicit spring formulas above |
| point block `K_` and `bias_` | [Joint.hpp:89](../Joint.hpp#L89) | revolute/weld/mouse anchors |
| prismatic `scalarMass_` | [Joint.hpp:94](../Joint.hpp#L94) | rotating-axis term included |
| motor | [Joint.hpp:125](../Joint.hpp#L125) | `clamp(old − K_ang⁻¹(ωB − ωA − motorSpeed), ±dt·maxMotorTorque)` |
| limits | [Joint.hpp:131](../Joint.hpp#L131) | speculative gap bias for revolute |
| angular lock | [Joint.hpp:146](../Joint.hpp#L146) | weld/prismatic; skipped when `lockAngle == false` |
| `correctPosition` | [Joint.hpp:161](../Joint.hpp#L161) | the NGS-half of each joint |
| `resetImpulses` | [Joint.hpp:200](../Joint.hpp#L200) | called when the cache is invalidated |

**Measured.** `testJoints` asserts a small-angle pendulum period of `2.8333 s` (theory),
a 10-link revolute chain anchor error of `0.000035`, a motor reaching its prescribed speed
and respecting an angular limit (`angle = 0.300000`), a mouse spring converging, and
prismatic/weld degrees of freedom. The **worst** joint in the project is honestly recorded
as the ragdoll: `info: ragdoll anchor error 0.034975 m`, and the 100-plank bridge's max
anchor error is `0.010674` *with* position projection. Joints are iterative constraints,
not exact rigidity — long chains are documented as not ship-quality.

**Exercise 17.1.** A pendulum of length 1 m and mass 1 kg is released at 0.1 rad. Compute
`2π√(L/g)` and compare with the suite's `2.8333 s`. Then halve the mass and explain why the
period does not change.

**Exercise 17.2.** Implement a joint whose constraint is `θA + θB = 0` (an equal-and-opposite
angle coupling) as an exercise *on paper first*: write `C`, its Jacobian `J` with respect to
`(ωA, ωB)`, the effective mass, and the sign of the impulse applied to each body. Then
implement it in `Joint.hpp` following the `Weld` branch as a template (§9 shows the full
loop for a smaller feature).

### 5.11 Islands and sleeping

**Intuition.** A pile that has stopped moving does not need to be simulated. But bodies in
contact must agree to sleep *together*, or a sleeper would be left floating when its
support wakes.

**Derivation — union-find on the constraint graph.** Build a graph whose vertices are
dynamic bodies and whose edges are the previous frame's contacts and the current joints;
union the endpoints of every edge. The connected components are *islands*. Static bodies
participate as constraints but **never as members**: if a shared floor joined islands, the
whole scene would be one island and nothing could ever sleep independently.

**Derivation — when an island sleeps.** With `kSleepTime = 0.5` s,
`kSleepLinearSpeedSq = 0.0025` (`|v| < 0.05` m/s) and `kSleepAngularSpeed = 0.05` rad/s:

```
slow(b)      = |v|² < 0.0025  and  |ω| < 0.05
sleepTime(b) = slow ? sleepTime + dt : 0
island sleeps iff every dynamic member is slow  and  min_b sleepTime ≥ 0.5
```

On sleeping, velocity and angular velocity are zeroed. Wake-up is the mirror image: an
island is active if any member is already awake, is moving, was disturbed from outside
(`force`, `torque`, `driveTorque` non-zero), or is attached to a joint with a motor or a
`Mouse` joint. **Gravity alone never wakes a sleeper** — that is the whole point.

**Why the sample time matters.** The island pass reads velocities *before* the force pass
(§4). After the force pass every resting body has `v = g·dt ≈ −0.16 m/s`, which is above
the 0.05 m/s threshold, so an island pass placed after it would wake everything forever.
The pass therefore runs on the previous frame's *post-solve* velocities, which for a
resting body are ~0.

**Consequences for the rest of the engine.** A sleeping pair is skipped by `prepare`, by
`solveIteration`, by the warm-start apply loop and by `correctPositions`, but its manifolds
**stay in the manifold list**, because they are the edges of the island graph. That single
rule is why `rowIsLive` is defined once and shared (§5.14 instead), why the broadphase
skips recomputing a sleeping body's tight AABB, and why the frozen-pair cache in
`findPairs` can re-emit a settled contact verbatim.

**Code map.**

| Symbol | File / line | Notes |
| --- | --- | --- |
| thresholds | [Islands.hpp:9](../Islands.hpp#L9) | `kSleepTime`, `kSleepLinearSpeedSq`, `kSleepAngularSpeed` |
| `Islands::build` | [Islands.hpp:26](../Islands.hpp#L26) | union-find; static bodies excluded from unions |
| `Islands::wakeConnected` | [Islands.hpp:50](../Islands.hpp#L50) | disturbance and motor/mouse rules |
| `Islands::sleep` | [Islands.hpp:70](../Islands.hpp#L70) | per-body `sleepTime`, island aggregate, zeroing |
| `Islands::island(i)` / `count()` | [Islands.hpp:94](../Islands.hpp#L94) | read-only access to the result |

**Measured.** `testStressSleeping` builds 5,000 dynamic boxes plus a static floor and
reports `settled field sleeps: 5000/5001 dynamic bodies asleep` with `max |v| = 0.000000`.
The denominator includes the floor because `World::bodies` includes it; all 5,000 *dynamics*
sleep. The cost claim is a wall-clock comparison (`21.5722 ms -> 15.1470 ms per step`) and
is owned by [benchmark_sleeping](../benchmark_sleeping.cpp), which exits non-zero if its
criterion fails. It is not a controlled sleep-on/off comparison at identical poses: see
[STORAGE.md](STORAGE.md#checks-and-performance) for the precise caveat.

**Exercise 18.1.** Two stacks stand on the same static floor, separated by 2 m. Sketch the
islands, then explain what happens to stack B if you nudge stack A, and why the floor must
not be a union member for that answer to hold.

**Exercise 18.2.** Set `kSleepLinearSpeedSq` to `0.01` and run `makeStress(2000)`. What
changes in the awake curve, and why is a *larger* threshold not automatically better?

### 5.12 The temporal substepped solver (TGS) and its per-slice policy

**Intuition.** One big step with a few sweeps is a coarse approximation of a stiff pile.
Take the *same* step but cut it into `N` smaller slices, solving the contacts in each. The
contact set is computed once per tick (cheap); only the solve gets finer (more accurate).

**Derivation — substepping.** For `h = dt / N` slices, the engine advances forces by `h`,
solves contact and joint constraints, then advances poses by `h`, `N` times per tick. Two
things follow immediately:

* **The impulse units change.** `previousDt_` is the duration an impulse represents: `dt`
  for PGS, `h` for TGS. `saveImpulses` is called with `h` at the end of a tick, so the warm
  cache means "one slice's support" and `restoreImpulses(h)` before the loop seeds with
  `h / previousH`.
* **The gravity increment per solve shrinks.** Resting penetration is governed by the
  *within-slice* impulse balance, not by the tick's `g·dt`, which is why stiff stacks are
  measurably tighter at the same sweep count.

**Derivation — local anchors, and why geometry is refreshed but not recomputed.** At first
contact the engine stores the contact point in *local* space, once per contact point:

```
a = RAᵀ (p − xA)        b = RBᵀ (p − xB)        s₀ = −depth
```

At every later slice the world anchors are `rA = RA·a`, `rB = RB·b`, and the separation is
recomputed without a narrowphase call:

```
s = s₀ + dot( xB + rB − xA − rA , n )
```

Initially `xB + rB = xA + rA`, so `s = s₀` even for a rotated body. Translation and rotation
move the anchors exactly (a rotation moves them the same way the body did), which is what
makes a slice-rate refresh correct for the *sampled* poses. Contact **normals and
topology stay fixed for the whole tick**; new contacts appear next tick. This is temporal
anchor solving, not `N` narrowphase simulations, and it is not a general no-tunnelling
proof.

**Derivation — softness.** For a compliant row (`compliance = α = 1/k`, `damping = c`, both
uniform per contact point, not per material) the implicit spring response gives

```
γ = α / [ h (h + cα) ] = 1 / [ h (c + h k) ]        (added to the effective inverse mass)
ERP = h / (h + cα)                                   (error reduction per slice)
bias = ERP · C / h
```

and for rigid rows (`compliance == 0`) `γ = 0` with `ERP = β / N`. That `/N` is the subtle
part: using the full-tick Baumgarte factor *every* slice injected excessive corrective
velocity in the 1/30 s, `N = 16` resting-stack test. Normalising by the slice count keeps
rigid correction on the tick timescale. The row increment is then

```
Δλ = ( target − Jv − γ·λ_old ) / (Kn + γ)
```

with `target = −C/h` (rigid bias or speculative closure) and `λ` clamped ≥ 0 after the
increment. For positive separation the target is `−s/h`: the row may close the gap but not
overshoot it, which is exactly the speculative principle of §5.13 applied to a resting
contact. **Positive gap can never generate tensile force**, because the total `λ` stays ≥ 0.

**The per-slice policy, in the order the code runs it** ([`World::stepTemporal`](../World.hpp#L200)):

| Step | Policy |
| --- | --- |
| before the loop | `restoreImpulses(h)` — seed impulses once for the whole tick |
| slice 0 | `prepareTemporal`: mix materials, capture `localA/localB`, `separation`, and the **bounce** from pre-warm velocities |
| every slice | `refreshTemporal`: refresh `rA`/`rB`/`depth`, `massN = 1/(Kn+γ)`, `massT = 1/Kt`, target; re-apply the accumulated impulse **once** (a seed, not a force per iteration) |
| every slice | `integrateForces(h)`; `joint.prepare(bodies, h, ratio)` with `ratio = 0` if warm starting is off |
| slice 0 ratio | `h / previousDt` (the tick-to-slice rescale) |
| slices > 0 ratio | `1` — the impulse already represents a slice |
| every slice | `iterations × { joint.solve; solveTemporal }`; the 2×2 block includes `γ` on its diagonal |
| restitution | `targetVn = first && bounce > 0 ? max(push, bounce) : push` — a rebound can only happen on slice 0, so a single impact cannot bounce `N` times |
| every slice | advance poses by `h`, or run CCD by `h` with force clearing **deferred** (`clearForces = false`) so a bullet keeps its external force across slices; CCD counters are summed and `capped` is OR-ed |
| after the loop | clear `force`/`torque` once; sleep (with the tick `dt`, not `h`); `saveImpulses(h)` |

With `warmStarting = false`, `refreshTemporal` zeroes `accN`/`accT` every slice and the
joint ratio is `0`, so a tick is a genuine cold start per slice.

**Validation, before any mutation.** `tryStep` rejects a TGS configuration with
`substeps < 1`, `iterations < 1`, a non-positive or non-finite `h`, negative or non-finite
`compliance`/`damping`, or an unrepresentable softness (`γ` or `β` non-finite) with
`std::invalid_argument`. Overflowing softness is **rejected, never clamped**: the caller
must lower `substeps` or raise `compliance`.

**Code map.**

| Symbol | File / line | Notes |
| --- | --- | --- |
| `SolverMode::TGS`, `substeps = 8` | [Solver.hpp:46](../Solver.hpp#L46) | opt-in; `SolverConfig{}` defaults to PGS |
| `compliance = 0`, `damping = 0` | [Solver.hpp:51](../Solver.hpp#L51) | per contact point, m/N and N·s/m |
| `struct TemporalPoint` | [TemporalSolver.hpp:8](../TemporalSolver.hpp#L8) | `localA`, `localB`, `separation`, `bounce` |
| `struct TemporalContact` | [TemporalSolver.hpp:12](../TemporalSolver.hpp#L12) | two points; one row per manifold |
| `prepareTemporal` | [TemporalSolver.hpp:19](../TemporalSolver.hpp#L19) | anchors, separation, materials, bounce capture |
| `temporalSoftness` | [TemporalSolver.hpp:39](../TemporalSolver.hpp#L39) | `γ`, `β` from `compliance`/`damping` |
| `refreshTemporal` | [TemporalSolver.hpp:47](../TemporalSolver.hpp#L47) | per-slice geometry, targets, single re-seed |
| `solveTemporal` | [TemporalSolver.hpp:80](../TemporalSolver.hpp#L80) | block 2×2 with `γ`, then scalar fallback, then friction |
| `World::stepTemporal` | [World.hpp:200](../World.hpp#L200) | the slice loop and the policy above |

**Exact layouts.** Enforced by `static_assert` in the header, on native and wasm32:
`TemporalPoint` = 48 bytes / align 8 with `separation` at 32 and `bounce` at 40;
`TemporalContact` = 96 bytes / align 8; `TemporalSoftness` = 16 bytes (two `double`s).
`World::prepare` reserves one `TemporalContact` per contact slot, so a prepared TGS world
adds `96 · C` bytes and allocates nothing per slice (§6).

**Measured** ([TGS.md](TGS.md) owns the table, `test_temporal` is the receipt): five-box
resting heights across `dt ∈ {1/30, 1/60, 1/120} × N ∈ {1, 4, 16}` stay within `0.0496` m
of ideal with zero speed; an active 1000:1 heavy-on-light load is stable with depth
`0.016460…0.010001` m; damping reduces the late error from `0.000811010` to `0.000000000`;
a compliant contact settles at exactly `α·m·g` (`0.019620 / 0.098100` for `α = 0.0002 /
0.001`); a driven 1000:1 distance joint keeps `|p − 20| < 1e-7` with force not multiplied
by `N`.

**What is not proven.** The 1000:1 configuration needs **1000 sweeps per slice** — evidence
of reached bounds, not cheap convergence, and not a throughput result. TGS is not exercised
in the browser showcase (it is compiled for wasm32 and cross-run in Node, but the page stays
on PGS and never writes `SolverConfig::mode`). No sanitizer receipt exists for it, and it
does not turn a malformed shape (zero radius or zero inertia) into valid input.

**Exercise 19.1.** With `dt = 1/60` and `N = 8`, compute `h`. Then take a contact with
`depth = 0.02`, `slop = 0.01`, `baumgarte = 0.2`, `compliance = 0`: write down `γ`, `β`,
`push`, and the first normal increment for a resting row (`Jv = 0`). Repeat with
`compliance = 0.001`, `damping = 40` and explain which target is larger and why.

**Exercise 19.2.** Run `test_temporal` locally, then set `substeps = 1` and re-run
`test_engine`. Explain why the PGS suite must be unaffected by every TGS change
(`SolverConfig{}` default and the browser entry point are the two answers).

### 5.13 Continuous collision detection: conservative advancement

**Intuition.** A body moving 16.7 m in a step cannot be caught by sampling only its start
and end. Instead, ask "how far can this pair move before they touch?" and advance exactly
that far, repeatedly, until the contact time is found.

**Derivation — why discrete steps tunnel.** A discrete step samples shapes at `x(t)` and
`x(t+dt)`. A 0.02 m wall and a body at 1000 m/s move 16.7 m per 1/60 s step, so the pair can
swap sides with no sample inside the overlap.

**Derivation — the advancement bound.** Let `d(A,B)` be the true distance and `g` a
*lower bound* on it. Every material point of a rigid body moves no faster than
`|v| + |ω|·r`, where `r = sweepRadius` is the farthest vertex radius (for a polygon,
`max|vertex|`; for a circle, `radius`). Therefore the relative closing speed is bounded by

```
V = |vB − vA| + |ωA|·rA + |ωB|·rB
```

so advancing by `g / V` **cannot** overshoot the first contact: the pair would have to
close more than `g` in that time, which is impossible at speed ≤ `V`. Repeating the test at
the new pose re-selects a separating axis and shrinks the interval, so progress is
guaranteed while `g > 0` and `V` is constant (constant-velocity window). `timeOfImpact`
caps this at **128 iterations** and reports `converged = false` if it runs out.

**Derivation — `g` must be a lower bound, not an estimate.** `separatingGap` computes, for
each candidate axis, the signed gap between the two shapes' projections, and keeps the
**maximum**. Projection gaps along any unit axis are always ≤ the true distance (the true
closest pair has a witness axis whose gap *equals* it), so the maximum is a valid lower
bound with no extra shrink term. Candidate axes are the face normals of both polygons, plus
centre/vertex directions when a circle is involved. If no axis was testable (degenerate
input) the fallback is `−(rA + rB)`, i.e. assume overlap.

**Touching is a contact, not a miss.** At `g ≈ 0` the advance `g/V` is zero, so the loop
would stall. The engine treats `g ≤ kContactTolerance` (`1e-7`) as the impact event, and
reports an *already overlapping* pair as `initiallyTouching` instead of casting it forward:
that pair belongs to the discrete solver this step (counted in `skipped`, not treated as
"separated").

**Impacts are resolved at the centre of mass (`r = 0`).** The sweep's impulse removes
relative normal motion and deliberately adds no angular velocity, because conservative
advancement bounds *when* a pair touches, not *where*: the stopping pose is only accurate to
the separation bound, so the lever arm is not known to sub-step accuracy. The code comment
records the failure that motivated this — a 3 mm plate tilted 0.12 rad against a wall picked
up ~`8e3 rad/s` from a `0.006 m` offset, then held a motionless contact point while its
centre drifted into the wall at `72 m/s`. Consequences: impact impulses are **push-only**
(`λ ≥ 0`), the normal is taken from the true narrowphase manifold whenever the pair already
overlaps, tangential friction is clamped to `μ·λ`, and restitution uses its own hard-coded
`vn < −1` threshold.

**Bounded work, honest reporting.** The event loop stops at `maxEvents = 64` and sets
`capped`. Candidate pairs are enumerated all-pairs and rejected by a bounding-circle
"reach" test (`rA + rB + |vB − vA|·remaining`), so cost is `O(N²)` casts in the worst case;
there is no swept tree. Two fallbacks exist, both **per body** and both counted:

* **motion clamp** — a bullet that is still touching something while still moving
  (`|v| ≥ 0.5`) has its remaining motion withheld (`clamped`); its velocity is untouched and
  the discrete solver resolves the contact next step.
* **positional safety net** — after the loop, any bullet still overlapping is separated
  positionally (`pushedOut`), with `depth = max(penetration − 0.005, 0)` capped at `0.2`,
  and velocities untouched.

Unrelated awake bodies keep integrating; **there is no global freeze**. That was the earlier
design and it was wrong: one unresolved rotating contact stopped the whole world for the
rest of the frame.

**Code map.**

| Symbol | File / line | Notes |
| --- | --- | --- |
| `kContactTolerance = 1e-7` | [Continuous.hpp:61](../Continuous.hpp#L61) | the touching test |
| `sweepRadius(b)` | [Continuous.hpp:68](../Continuous.hpp#L68) | farthest vertex / radius |
| `struct SweepPose` | [Continuous.hpp:78](../Continuous.hpp#L78) | `position` + `rot` only; **borrows geometry, never copies owning `Shape` vectors** — enforced by `static_assert` on size/offsets (48/8) |
| `projection` / `separatingGap` | [Continuous.hpp:105](../Continuous.hpp#L105) | per-axis gap, maximum = lower bound |
| `closingSpeedBound` | [Continuous.hpp:133](../Continuous.hpp#L133) | the `V` above |
| `struct TOI` | [Continuous.hpp:139](../Continuous.hpp#L139) | `hit`, `converged`, `initiallyTouching`, `time`, `separation`, `iterations` |
| `timeOfImpact` | [Continuous.hpp:149](../Continuous.hpp#L149) | 128-iteration advancement loop |
| `struct CCDStats` | [Continuous.hpp:183](../Continuous.hpp#L183) | `events, casts, skipped, unconverged, pushedOut, clamped, capped` |
| `advanceContinuous(bodies, dt, scratch, maxEvents, clearForces)` | [Continuous.hpp:194](../Continuous.hpp#L194) | the event loop, clamp, safety net |

**Measured.** Circle and nonspinning 0.1 rad polygon bullets at 200, 1000 and 10000 m/s
against a 0.02 m wall each report `events = 1, capped = 0, clamped = 0, pushedOut = 0,
unconverged = 0`, come to rest at the wall with `|v| = 0` and `|ω| = 0`, and never move the
static wall (the suite asserts both the outcome *and* that the fallbacks stay idle). The
separate spinning-plate case stops translation while preserving `ω = 60 rad/s`; the
analytic moving-circle TOI matches its closed-form root (`0.15000000`); an unrelated body
keeps integrating during a bullet step (`x = 0.050000000`).

**What is not proven.** Off-centre impact torque is absent by design, joint re-solving
during a cast is absent, connected-body exclusion during casts is absent, and the spinning
case is not a zero-spin rest test at all three speeds. Nothing here certifies arbitrary
geometry against tunnelling.

**Exercise 20.1.** A 1 m/s circle approaches a 0.02 m wall. Compute `V` and the first
advance `g/V` with `g = 0.5 m`. Then do the same for a body spinning at 100 rad/s with
`sweepRadius = 0.3 m` and explain why the spin term makes the cast *slower* (more
iterations) but not less accurate.

**Exercise 20.2.** Reproduce the `initiallyTouching` path: build a pair that overlaps at
`t = 0`, set `bullet = true` on one body, step once, and print `ccdStats.skipped`. Then set
`bullet = false`, step again, and explain what the discrete solver does with the same pair.

### 5.14 The parallel track: static-body and colouring rules

**Intuition.** If two solver rows never write the same *movable* body, they can run at the
same time. An immovable body is never written, so thousands of rows may share a floor for
free. Set up the rows and the colours correctly, and the parallel driver becomes a small
piece of work — the physics does not have to change.

**Derivation — the conflict graph.** Two rows *conflict* if they write a body in common. A
row writes body `A` only if `A` is movable; `RigidBody::applyImpulse`,
`applyAngularImpulse` and `translate` all return early for a static body (§3.4), so a static
body contributes **no** conflict edges and therefore **no** colours. Greedy colouring in a
canonical order then needs at most `max degree + 1` colours, where degree counts only
movable neighbours — and for a scene of N separate bodies on one floor that is **1 colour**,
not N.

**The four rules that make a schedule meaningful** ([SolverRows.hpp](../SolverRows.hpp),
[Schedule.hpp](../Schedule.hpp), [PARALLEL.md](PARALLEL.md)):

1. **Identity is indices plus an ordinal**, never a pointer: a schedule computed for one
tick stays valid across `bodies` reallocation and is reproducible across runs.
2. **A row is a solver *block*.** A manifold is one row (its two points solve together as a
   2×2 block); a joint is one row. Splitting a manifold into per-point rows would change
   the solve, so it is not a legal parallelization.
3. **The order is a total order**, `rowLess` = (kind, bodyA, bodyB, ordinal) with joints
   before contacts. The serial solve order already *is* this order because
   [`Broadphase::findPairs`](../Broadphase.hpp#L21) sorts candidate pairs and `World::findPairs`
   sorts frozen contacts; `isCanonical(rows)` therefore holds for the shipped pipeline, and tree
   traversal order cannot leak into a schedule.
4. **A row exists only if it will write.** `rowIsLive(A, B)` is false when both bodies are
   static or asleep; the row list is exactly the set of live blocks, so a driver never
   schedules work that a serial kernel would skip.

`colorRows` assigns the lowest colour not already spent by either body, in canonical order;
`validateColoring` then re-checks the definition directly — for every pair of rows sharing a
colour, no movable body may appear in both. Both are pure and thread-free: they compute and
validate a schedule, they do not execute one.

**Code map.**

| Symbol | File / line | Notes |
| --- | --- | --- |
| `struct SolverRow` | [SolverRows.hpp:25](../SolverRows.hpp#L25) | `kind` (`Joint = 0`, `Contact = 1`), `bodyA`, `bodyB`, `ordinal` |
| `rowLess` | [SolverRows.hpp:37](../SolverRows.hpp#L37) | the canonical total order |
| `rowIsLive` | [SolverRows.hpp:51](../SolverRows.hpp#L51) | "this block does work", shared by PGS, TGS, RowSet, NGS |
| `RowSet::build` | [SolverRows.hpp:70](../SolverRows.hpp#L70) | joints in index order, then manifolds in current order |
| `canonicalOrder` | [SolverRows.hpp:101](../SolverRows.hpp#L101) | row indices sorted by identity |
| `isCanonical`, `identityUnique` | [SolverRows.hpp:110](../SolverRows.hpp#L110) | the two shipped invariants |
| `struct ColorPlan` | [Schedule.hpp:21](../Schedule.hpp#L21) | `order`, `color_of_row`, `colors`, `largestClass` |
| `colorRows` | [Schedule.hpp:31](../Schedule.hpp#L31) | greedy, deterministic, immovable bodies spend nothing |
| `validateColoring` | [Schedule.hpp:69](../Schedule.hpp#L69) | direct conflict re-check |
| `World::rows()` / `movableBodies()` | [World.hpp:187](../World.hpp#L187) | the per-tick row buffer and the write predicate |

**Measured.** On 24 circles resting on one immovable floor: **24 rows, 1 colour** at full
width, versus **24 colours** if the floor were treated as a conflict. Independently,
`test_rows` shows immovable bodies are *bitwise untouched* under PGS and TGS with NGS and
sleeping on, over 200 ticks with five joint families anchored to a static floor, and with
zero allocations around every step.

**What is not proven, and must be closed first.** There are no threads in this repository,
and [PARALLEL.md](PARALLEL.md) names the seven residual blockers in the order they must be
closed: (1) no single row driver — `tryStep` still runs joints then contacts in separate
loops; (2) `refreshTemporal` is a second intra-tick write phase; (3) `movableBodies()`
allocates on every call; (4) `rows_` is a mutable `World` member with no per-tick epoch;
(5) position correction is a second, geometry-changing pass; (6) CCD and the island pass
stay serial by design; (7) the warm-start cache must be saved/restored *outside* any
parallel region. Storage preparation and a valid colouring are preconditions, not thread
safety.

**Exercise 21.1.** Take the 24-circle scene (or build any row of N circles on one floor),
print `plan.colors` and `plan.largestClass`, then replace the floor with *two* static floors
and re-run. Predict the colour count in each case before you look.

**Exercise 21.2.** With a two-box stack (two manifolds sharing the middle box as `bodyB` of
one and `bodyA` of the other), write down the rows by hand, colour them, and then run
`validateColoring`. Explain why `identityUnique` must hold for the schedule to mean
anything.

---

## 6. Storage, allocation and layout contracts

**Why this exists.** A frame that calls `malloc` an unpredictable number of times has an
unpredictable cost. The engine therefore offers a *prepared* mode in which a validated
step performs no allocation at all, and it makes the difference measurable with an
allocation counter. This section is the contract; [STORAGE.md](STORAGE.md) is the detailed
owner document.

### 6.1 The API and its failure semantics

```cpp
phys::World w;
// create bodies/shapes/joints here (setup may allocate)
w.prepare({5001, 100, 20000, 20000, 20004});   // bodies, joints, pairs, contacts, vertices
const auto status = w.tryStep(1.0 / 60);
if (status != phys::World::StepStatus::Complete) {
    // NOT a simulated tick: prepare larger limits outside the loop and retry.
}
```

| Rule | Detail |
| --- | --- |
| Limits are **logical counts**, not vector capacity | `prepare` may reserve more, but the contract is about the counts |
| `tryStep` returns a status; `step` throws | `step` throws `std::length_error` on any non-`Complete` status |
| Rejection is recoverable | bodies, forces/awake state, joint impulses, previous manifolds, contact history and `previousDt_` are preserved |
| Derived data may move | proxy/query scratch, partial pair output and profiling timings are *not* committed state; do not consume a partial pair list |
| Exceptions are outside the contract | invalid `dt`/joint indices/limits still throw; use `tryStep` for capacity, not as an exception guard |
| Unprepared worlds still work | they grow dynamically exactly as before |
| A copied `World` needs `prepare` again | vector copies need not retain reserved capacity |
| `AllPairs` mode ignores the pair limit | it streams the oracle instead of buffering pairs |
| Body/shape/joint construction may allocate | it is setup or input, not a step; never add/erase during a step |

The capacity checks run in this order in `tryStep`: bodies, joints, total polygon vertices,
then candidate pairs (via `Broadphase::exhausted()`), then contact manifolds (during
staging). Tree indices are `int`, so `prepare` rejects a body limit above `INT_MAX / 2`.

### 6.2 Who owns what

| Owner | Storage |
| --- | --- |
| `World` | bodies, joints; current and pending manifold vectors; sorted previous contacts (`cache_`); snapshots + flat local-vertex copies (`vertices_`); reusable CCD clamp bytes; the row buffer; the temporal row buffer |
| `Broadphase` | tight AABBs, proxy ids, sorted candidate pairs, tree query stack |
| `DynamicTree` | pooled nodes (2B−1) plus a free list |
| `Islands` | union-find parents, one byte-per-body flag buffer reused by wake and sleep, quiet-time scratch |
| `ContactPoint` / `Joint` | the only accumulated impulses in the engine |

The **second manifold vector is deliberate**: `nextManifolds_` is the staging boundary that
makes a contact-capacity failure recoverable without rolling back body state. The warm-start
cache is *pointer-free* (`CachedManifold` stores indices, not `RigidBody*`), and the
manifold pointers that `manifolds()` exposes are rebuilt every step, so they are transient.

### 6.3 Extra prepared storage

Excluding vector control blocks, allocator rounding and the public shapes/joints: two
`288·C` manifold buffers + `272·C` cache + `96·B` snapshots + `16·V` vertex copies + `B` CCD
bytes; broadphase `64·(2B−1)` nodes, `4·B` proxies, `32·B` tight bounds, `4·(2B−1)` query
scratch, `16·P` pairs; islands `8·B` parents + `B` flags + `8·B` times; TGS one `96·C`
temporal row. No cache-line padding is added: all of this is serial today and no false-sharing
claim follows.

### 6.4 Exact layouts (native GCC 13.2 MinGW x64)

These are **in-process layouts, not file or network formats**. 32-bit WASM `size_t` and
pointer fields differ and must be measured independently before any such claim; the
compile-time assertions in [Continuous.hpp](../Continuous.hpp#L78) and
[TemporalSolver.hpp](../TemporalSolver.hpp#L8) run on both.

| Record | Offsets (bytes) | Size / align |
| --- | --- | --- |
| `World::Capacity` | bodies 0, joints 8, pairs 16, contacts 24, vertices 32 | 40 / 8 |
| `World::CachedManifold` | contactPoints 0 (2×104), normal 208, penetration 224, friction 232, restitution 240, indexA 248, indexB 256, count 264, tail pad 268–271 | 272 / 8 |
| `World::Snapshot` | position 0, velocity 16, angle 32, angularVelocity 40, invMass 48, invInertia 56, radius 64, vertexOffset 72, vertexCount 80, type 88, tail pad 92–95 | 96 / 8 |
| `ContactPoint` | position 0, featureId 16, depth 24, rA 32, rB 48, massN 64, massT 72, targetVn 80, accN 88, accT 96 | 104 / 8 |
| `ContactManifold` | bodyA 0, bodyB 8, normal 16, penetration 32, contactPoints 40, indexA 248, indexB 256, count 264, friction 272, restitution 280 | 288 / 8 |
| `DynamicTree::Node` | bounds 0, parent 32, left 36, right 40, height 44, next 48, body 56 | 64 / 8 |
| `SweepPose` | position 0, rot 16 | 48 / 8 |
| `TemporalPoint` | localA 0, localB 16, separation 32, bounce 40 | 48 / 8 |
| `TemporalContact` | points[2] at 0 | 96 / 8 |

[test_world_storage.cpp](../test_world_storage.cpp) prints these values in its receipt, so
the table can be re-checked on any host rather than trusted.

### 6.5 How "zero allocations" is actually verified

[test_allocation_counter.cpp](../test_allocation_counter.cpp) replaces the global
`operator new`/`new[]`/`delete` and increments `testAllocations` while
`countTestAllocations` is true. Tests toggle the flag around the code under test and assert
the count stays at zero. Two examples worth knowing:

* `test_world_storage` counts allocations around **every** `tryStep`, including the first,
  over driven contact/joint trajectories for tree/all-pairs × sleeping on/off, plus capacity
  failures and retries.
* `test_ccd_allocations` proves the sweep *borrows* geometry: it compares `timeOfImpact`
  against an oracle that deliberately copies bodies, and fails if the counter does not
  observe the oracle's copies while observing none in the real cast.

The standalone convenience overload `advanceContinuous(bodies, dt, maxEvents)` allocates its
clamp scratch (it constructs a `std::vector<char>`); `World` always passes its own
`ccdScratch_`, which is the allocation-free path.

**Exercise 22.1.** Add a sixth entry to a `prepare` call for your own scene and drive it
with `tryStep` for 600 steps with `sleeping = true`. Assert zero allocations per step. Then
remove one body's vertex from your capacity budget and describe exactly what the returned
status is and what state is preserved.

---

## 7. Scenes and the browser build

### 7.1 Scenes are plain `World` factories

That is deliberate: a scene that only exists inside a renderer cannot be regression-tested.
There are two levels.

* [Scenes.hpp](../Scenes.hpp) — minimal, test-oriented factories: `makeStack(rows, warm)`,
  `makePyramid(rows, warm)`, `makeChain(links)`.
* [Showcase.hpp](../Showcase.hpp) — presentation scenes, listed by
  [`sceneNames`](../Showcase.hpp#L18) with [`kSceneCount = 10`](../Showcase.hpp#L17)
  and dispatched by [`makeShowcase(scene, trigger)`](../Showcase.hpp#L280).

| Scene | Factory | Interesting return value |
| --- | --- | --- |
| Newton's cradle | [`makeCradle`](../Showcase.hpp#L30) | world |
| Momentum row | [`makeMomentumRow`](../Showcase.hpp#L56) | world (no gravity, no joints) |
| Domino run | [`makeDominoRun`](../Showcase.hpp#L74) | world |
| Wrecking pyramid | [`makeWreckingPyramid`](../Showcase.hpp#L90) | world |
| Ragdoll | [`makeRagdoll`](../Showcase.hpp#L108) | world |
| Suspension bridge | [`makeBridge`](../Showcase.hpp#L135) | world |
| Motorized car | [`CarRig`](../Showcase.hpp#L162) / `makeCarRig` | world + chassis/wheel indices |
| Rube-Goldberg | [`RubeMachine`](../Showcase.hpp#L206) / `makeRubeGoldberg` | world + flag/ball indices, `hasBall` |
| Stress 5000 | [`makeStress`](../Showcase.hpp#L246) | world |
| Bullet vs wall | [`makeBulletScene`](../Showcase.hpp#L266) | world |

Factories that carry a *claim* return the indices the test needs (`CarRig`, `RubeMachine`),
and causal scenes take a flag so the claim can be falsified: `makeRubeGoldberg(trigger)`
and `makeCradle(balls, liftFirstBall)` exist so the suite can assert that *nothing happens
without the trigger*.

**Adding a scene** (five steps, in this order):

1. Write the factory in `Scenes.hpp` (minimal) or `Showcase.hpp` (presentation), returning
   a `World` by value.
2. Add its name to `sceneNames` and a case to `makeShowcase`, and bump `kSceneCount`.
3. Add assertions to `test_engine.cpp`: at least a finiteness check plus a *claim*
   (conservation, causality, travel distance, settling). The showcase smoke test already
   guarantees every factory stays finite for 30 steps and that timings are non-negative —
   that is a floor, not a claim.
4. Run the suite. `benchmark_showcase` picks up `kSceneCount` automatically.
5. Add the browser metadata in [../web/scenes.js](../web/scenes.js) and test its camera
   inverse, spawning, dragging and scene cleanup.

### 7.2 The browser build

Nine assets: `index.html`, `index.js`, `index.wasm`, `showcase.css`, and five scripts —
`core.js` (DOM handles, camera transform, formatting), `scenes.js` (per-scene metadata and
editorial copy), `render.js` (flat-buffer decode + paint), `instrument.js` (the telemetry
controller) and `showcase.js` (input, tuning, loop). [../web/build.sh](../web/build.sh)
compiles the WASM and copies the static assets into `build/web/`; keep all nine together,
because WASM is not loaded from `file://`.

The C++ surface is a small `extern "C"` API in [../web/main.cpp](../web/main.cpp):
`select_scene`, `spawn_box`/`spawn_circle`, `tick_js`, `toggle_pause`, `step_once`,
`set_slow`, `toggle_debug`, `is_paused`, `debug_enabled`, `frame_buffer`/`frame_size`,
`stats_buffer`/`stats_size`, `profile_enabled`/`set_profile`, `iterations`/`set_iterations`,
`position_correction`/`set_position_correction`, `sleeping_enabled`/`set_sleeping`,
`gravity_y`/`set_gravity`, `solver_mode`, `sim_steps`, `begin_drag`/`update_drag`/`end_drag`.

The frame buffer is a flat `float` array decoded by `render.js`; the second, additive
`stats_buffer` carries per-stage timings and the live solver configuration for the
dashboard. Two facts about this interface matter for correctness claims:

* the browser **reads** `solver_mode` but never writes `SolverConfig::mode`, so the TGS path
  is unreachable from the page (the tuning panel writes `iterations`, `positionCorrection`
  and `World::sleeping`);
* the page's step cost excludes canvas paint, so no rendered-FPS claim comes from it.

**Exercise 23.1.** Add a scene to `Showcase.hpp` (for example a single ball on a ramp),
wire it into `sceneNames`/`makeShowcase`, add one finiteness and one claim assertion, run
`test_engine`, and then add its `web/scenes.js` entry with a camera. What breaks first if
you forget step 5?

**Exercise 23.2.** Read `web/render.js` and `web/instrument.js` and describe the exact byte
layout the JavaScript assumes. Then check that assumption against the C++ writer in
`web/main.cpp`. Is the layout documented anywhere, or only implied by both sides?

---

## 8. Verifying: the suites, the receipt, and the limits

### 8.1 What runs

Five executables are registered with CTest in [../CMakeLists.txt](../CMakeLists.txt):
`engine`, `ccd_allocations`, `world_storage`, `temporal`, `rows`. Four of them (all but
`engine`) link the allocation counter, so "no allocation" is part of their contract, not a
separate tool.

```bash
# the whole thing
cmake -S . -B build/cmake -DCMAKE_BUILD_TYPE=Release && cmake --build build/cmake
ctest --test-dir build/cmake --output-on-failure

# one suite, without CMake (equivalent evidence for the same source)
g++ -std=c++20 -O2 -Wall -Wextra -Wpedantic -Werror -I. test_rows.cpp test_allocation_counter.cpp -o build/test_rows.exe
./build/test_rows.exe
```

`test_engine.cpp` uses one macro, `CHECK(cond, fmt, ...)`
([../test_engine.cpp:14](../test_engine.cpp#L14)): it prints `  ok:   ...` with the measured
values on success and `  FAIL: ...` and increments `g_failed` on failure. `main` returns
non-zero if anything failed, so the **exit status is the pass/fail signal** — never pipe it
through a filter without preserving that status (`set -o pipefail`).

### 8.2 What each section of `test_engine` proves

| Section (in run order) | The claim it tests |
| --- | --- |
| mass properties | box/polygon/circle inertia against closed forms; CW input and offsets recentred |
| symplectic Euler | bounded oscillator energy over 1000 s; explicit Euler shown diverging |
| box dropped on ground | rest height, flat settling, at rest |
| circle restitution | first rebound height vs the ideal |
| Coulomb friction | hold vs slide on both sides of `tan θ` |
| 3-box stack | height, no lateral drift, at rest |
| AABB geometry | inclusive touching, rejection, fat containment, circle/rotated bounds |
| dynamic tree | balanced sorted insertion, retained fat proxy, teleport, removal, free-list reuse |
| broadphase oracle | identical contact sets vs all-pairs and a tight-AABB oracle; invariants under mutation |
| world oracle | bit-identical trajectories, tree vs all-pairs, over 600 steps |
| persistent feature IDs | id stability under tangential motion, impulses genuinely seeding velocity, cold start, teleport invalidation |
| warm start | 20-second 20-box stack and 15-row pyramid bounds |
| joints | pendulum period, chain error, motor speed and limit, mouse, prismatic, weld |
| NGS and sleeping | overlap reduction without added velocity; island sleep/wake; floor isolation; `dt` rejection |
| continuous casts | circle and tilted polygon bullets at 200/1000/10000 m/s: wall stop, rest, idle nets; spinning-plate preservation; unrelated-body continuation; analytic TOI |
| smooth-force convergence | measured step-halving ratios for symplectic/Verlet/RK4 |
| fuzz | fixed-seed 2000-step elastic fuzz: finite, deterministic, no energy blow-up |
| showcase smoke | every scene factory finite for 30 steps; timings non-negative |
| momentum/cradle scenes | conservation, transfer, causality (`testMomentumConservation`, `testPendulumCradle`, `testDominoCascade`) |
| ragdoll/bullet/car | limits and settling, bullet rest state, car travel and suspension |
| Rube-Goldberg, wrecking pyramid | causality with and without the trigger; at rest then wrecked |
| stress 5000 | settling and sleep (`5000/5001` dynamic bodies asleep, `max |v| = 0`) |
| 100-plank bridge | anchor error with position projection (`0.010674`) |

**Current counts (measured on this checkout).** A clean optimized rerun of
`test_engine` prints **142** `ok:` lines and exits `0` (fresh receipt:
[../build/fs_engine.log](../build/fs_engine.log), from the same command as §2). The historical receipt
[../build/final_tests.log](../build/final_tests.log) contains **143** because it still
includes two sleeping-throughput lines that have since moved to
[benchmark_sleeping](../benchmark_sleeping.cpp); the correctness half of those two stayed in
the suite. If you are reading a different number, re-run and compare the *sections*, not
just the total.

**Determinism is a tested property, not a hope.** `testWorldOracle` asserts bit-identical
trajectories between the tree and the all-pairs broadphase over 600 steps, and `testFuzz`
asserts that a fixed seed reproduces its trajectory exactly. That is what lets you treat a
changed `ok:` line as a real behavior change.

### 8.3 What is **not** verified here

Say these out loud before quoting any result:

* **Sanitizers are not executed.** `.github/workflows/ci.yml` configures ASan/UBSan, and
  local MinGW linking fails for missing runtimes; there is no sanitizer receipt.
  Memory/UB correctness currently rests on the allocation counter plus behavioural bounds.
* **No remote CI has run.** The workflow exists; it has not executed on GitHub.
* **`cmake` and `ctest` are absent from `PATH` on this host** (measured, §2). The five
  suites are therefore compiled directly; that is the same code and the same assertions,
  but the CMake/CTest configuration itself is only as verified as
  [STORAGE.md](STORAGE.md) records it.
* **No rendered-FPS number exists.** Step cost excludes canvas paint, in the browser and
  natively. `benchmark_active` measures simulation stages only.
* **Timing assertions are relational.** Host load moved driven figures by more than 2×
  between two runs of the browser dashboard suite, so its assertions compare *quantities*
  (sleeping cheaper than awake, gauge equals its step, stage sum reconciles) rather than
  millisecond thresholds. Benchmarks own their own exit criteria.
* **TGS is native-only in practice.** It is compiled for wasm32 and cross-run in Node with
  the same `0 failed` result, but the page never selects it.
* **CCD is a bounded prototype.** Off-centre impacts, joints during a cast, connected-body
  exclusion and swept-tree acceleration are absent; the walls in the tests are not a
  general guarantee.
* **Long chains are not ship-quality.** The ragdoll's `0.034975 m` anchor error is the
  worst warm-started joint in the project and is printed as an `info:` line on purpose.
* **The 1000:1 TGS configuration needs 1000 sweeps per slice.** Evidence of reached bounds,
  not cheap convergence, and not throughput.
* **`build/` receipts are local.** They are not guaranteed to exist in a clean clone and
  are not curated release evidence.

**Exercise 24.1.** Run all five suites on your machine and record the counts next to the
ones above. Then deliberately break something small (for example set `slop = 0`) and
confirm that a *specific* assertion fails with measured values rather than the whole run
collapsing.

**Exercise 24.2.** Write one new assertion of the form "a claim plus a falsification":
for example, that a box pushed off a ledge falls and that the same world without the push
does not move. Follow §9's loop end to end; only the physics is different.

---

## 9. Worked walkthrough: add per-body gravity scale, end to end

This is the smallest complete feature that still exercises the whole pipeline: a new body
property, a real change to the force integrator, a new assertion, a rebuild, and a
documentation update. It is deliberately not a new solver term — §5.10's Exercise 17.2 is
the next step up from here.

**The feature.** `RigidBody::gravityScale`, a per-body multiplier on world gravity.
`1.0` is normal, `0` is weightless (a floating platform), and a negative value falls upward
(a zero-gravity scene can still be stirred, a platformer can have a lift).

**The contract, decided before writing code.** Write this down first; every later decision
follows from it.

| Question | Answer | Why |
| --- | --- | --- |
| Default value | `1` | otherwise every existing scene changes behavior |
| Where does it apply? | in `integrateForces` | that is the only place gravity is applied, and both PGS and TGS call it |
| Static bodies? | irrelevant — the integrator already skips them | `isStatic()` returns before the velocity update |
| Sleeping? | no interaction | a sleeping body is never force-integrated, and this is not a disturbance |
| CCD? | no interaction | `advanceContinuous` never applies gravity; it moves poses only |
| Cache invalidation? | **none needed** | cached geometry and impulses do not depend on future gravity |
| Interface changes? | none | header-only engine, no ABI, no build files |

### Step 1 — the state, in the header that owns it

Open [RigidBody.hpp](../RigidBody.hpp) and find the body-state block (the `bullet` line at
[RigidBody.hpp:51](../RigidBody.hpp#L51)). Insert the field immediately before it:

```cpp
    // Multiplies world gravity for this body: 1 = normal, 0 = weightless,
    // negative = inverted. Read by integrateForces, so it also reaches every
    // TGS per-slice force pass without World knowing about it.
    real gravityScale = 1;
```

Two habits to notice: the default is written in the declaration (so `RigidBody b;` and all
three factories get it for free), and the comment states the *policy* plus the reason it is
placed here rather than in `World`.

### Step 2 — the physics, in the one function that applies gravity

In [Integrator.hpp:24](../Integrator.hpp#L24), the velocity update currently reads:

```cpp
    b.velocity += (b.force * b.invMass + gravity) * dt;
```

Change it to:

```cpp
    b.velocity += (b.force * b.invMass + gravity * b.gravityScale) * dt;
```

That is the entire physics change. `gravity` stays an acceleration (it is still *not*
multiplied by `invMass`); only its per-body scaling is new. Because `World::step` and
`World::stepTemporal` both call this function — the latter once per slice — PGS and TGS
agree by construction, and no call site needs to change.

### Step 3 — the test, in the suite that owns the claim

Add a function to [test_engine.cpp](../test_engine.cpp) and call it from `main`. The one
convention to follow is that a `CHECK` message states the claim *and* prints the measured
value:

```cpp
static void testGravityScale() {
    std::printf("[per-body gravity scale]\n");
    World w;
    w.gravity = {0, -9.81};
    auto floating = RigidBody::makeCircle(0.5, 1, {0, 2});
    floating.gravityScale = 0;                 // weightless: no contact, no force
    auto lifted   = RigidBody::makeBox(0.5, 0.5, 1, {3, 2});
    lifted.gravityScale = -1;                  // inverted gravity
    const std::size_t f = w.add(floating);
    const std::size_t l = w.add(lifted);
    for (int i = 0; i < 60; ++i) w.step(1.0 / 60);
    CHECK(near(w.bodies[f].position.y, 2, 1e-9),
          "gravityScale=0 holds its height: y=%.6f", w.bodies[f].position.y);
    CHECK(w.bodies[l].position.y > 2.6,
          "gravityScale=-1 falls upward: y=%.6f", w.bodies[l].position.y);
}
```

and in `main`, next to the other scene-level calls:

```cpp
    testBridgeProjection();
    testGravityScale();
```

Why these two assertions and not more: the first is exact (a weightless body with no
contact cannot move, so `1e-9` is legitimate), and the second is a *causal* claim with a
wide margin (after 1 s of inverted gravity a body falls *up* by roughly `½gt² ≈ 4.9 m`, so
`> 2.6` cannot pass by accident). Neither depends on host timing.

### Step 4 — build, run, and read the result

```bash
g++ -std=c++20 -O2 -Wall -Wextra -Wpedantic -Werror -I. test_engine.cpp -o build/test_engine.exe
./build/test_engine.exe > build/gravity_scale.log 2>&1; echo "exit=$?"
grep -c 'ok:' build/gravity_scale.log        # expect 144 (142 + 2 new checks)
grep -A3 'gravity scale' build/gravity_scale.log
tail -2 build/gravity_scale.log
```

`-Werror` is not optional: the suite is compiled with warnings-as-errors everywhere, so an
unused variable or a narrowing conversion fails the build rather than hiding. What this
exact change printed when it was exercised against a scratch copy of this checkout (receipt:
[../build/fs_gravity_scale.log](../build/fs_gravity_scale.log); the scratch copy, not this
tree, carries the edit until you make it):

```
[per-body gravity scale]
  ok:   gravityScale=0 holds its height: y=2.000000
  ok:   gravityScale=-1 falls upward: y=6.986750

ALL TESTS PASSED (0 failed)
```

`6.986750` rather than the textbook `2 + ½·9.81 = 6.905` is the *integrator's* first-order
error at `dt = 1/60` for a constant force — symplectic Euler overshoots by `½·g·dt·t`, and
the exact value `2 + ½g t(t + dt) = 6.9868…` matches to all printed digits. That is a
useful habit: before calling a discrepancy a bug, compute the discretization error.

### Step 5 — document it where the next reader will look

A feature is not done when it compiles. Add, in this order:

1. a row to the state list in §3.4 and to its code map;
2. one sentence in §5.2 next to the gravity term (the derivation is unchanged);
3. an exercise if it raises a question a reader should be able to answer;
4. nothing in the measured tables, because this change adds no measurement. If it *had*
   changed a cost or a bound, the number belongs in [BENCHMARKS.md](BENCHMARKS.md), which
   owns all timing tables, never in two places.

### Step 6 — what would change if the feature were physics-level instead

This feature touches one line of integration. A solver-level feature (a new contact term, a
new joint, a new material rule) follows the same loop but with two additions:

* **the suite's *existing* rows are the regression test for the default**: if your new term
   is opt-in, `SolverConfig{}` must keep its old value and the whole suite must stay green
   *unchanged*. That is exactly how PGS survived the TGS work (§5.12).
* **the plan of record gets a row**: for multi-track work, record the outcome in
   [FEATURE_ACCEPTANCE.md](FEATURE_ACCEPTANCE.md) and the state in
   [../PHYSICS_ENGINE_GUIDE.md](../PHYSICS_ENGINE_GUIDE.md) §5.

### Step 7 — optional: expose it to the browser

The tuning panel is the live surface. To add a slider you add three small things and touch
nothing else:

1. a getter/setter pair in [../web/main.cpp](../web/main.cpp) beside
   `gravity_y`/`set_gravity`, calling `g_world.bodies[...]` … or, for a *world-wide* default,
   a loop that stamps `gravityScale` on every body;
2. a control in [../web/shell.html](../web/shell.html) plus its wiring in
   [../web/showcase.js](../web/showcase.js);
3. a dashboard read-back if the effect shows up in `stats_buffer` (gravity itself already
   does).

The rule the panel follows: **write to the engine, then re-read it** after every change,
scene switch and restart, so a control can never drift from the solver. And do not add a
`solver_mode` write: the page intentionally cannot reach TGS.

### Step 8 — reverting

There is no Git metadata in this checkout, so a scratch copy is your undo:
`cp RigidBody.hpp Integrator.hpp test_engine.cpp /tmp/` before you start. (Adding a
repository is one of the outstanding tasks in
[../PHYSICS_ENGINE_GUIDE.md](../PHYSICS_ENGINE_GUIDE.md) §6.)

**Exercise 25.1.** Make the walkthrough change yourself and get 144 green assertions. Then
add a *third* assertion that uses `gravityScale` together with a contact: a ball with
`gravityScale = 0` resting on a box with `gravityScale = 1` should stay put while the box
falls away — decide what the assertion should check and why that is a harder claim than
the two above.

**Exercise 25.2.** Extend the feature to a *world-level* default (for example
`World::gravityScale`) and decide, in writing, which one should win for a body that has
both. Then find how many places in the engine read `gravity` and argue whether your design
can be made consistent with all of them.

---

## 10. Where to look when something is wrong

| Symptom | First file | What to check |
| --- | --- | --- |
| A fast body passes through a wall | [Continuous.hpp](../Continuous.hpp) | Is `body.bullet` set? Is `ccdStats.skipped` growing (the pair was touching at `t = 0`)? Is `initiallyTouching` swallowing the pair? |
| Contacts flicker on and off at rest | [Solver.hpp](../Solver.hpp) | `restitutionThreshold` (bounce must be off at low speed) and `slop` (a small overlap keeps contacts persistent) |
| Bodies jitter or drift at rest | [Solver.hpp](../Solver.hpp), [World.hpp](../World.hpp) | `baumgarte`/`positionBeta` too large, `slop` too small, or the `targetVn` push term fighting restitution |
| Stacks sway or sag | [Solver.hpp](../Solver.hpp) | warm starting enabled, `blockSolve` on for two-point manifolds, enough `iterations`; check the block-solve determinant guard |
| Energy grows in a closed scene | [World.hpp](../World.hpp), [Solver.hpp](../Solver.hpp) | Baumgarte injection, restitution firing below the threshold, or NGS moving bodies into each other |
| A stack sinks into the ground | [World.hpp](../World.hpp) | NGS enabled but `positionIterations` too low, or penetration deeper than the `clamp(..., 0.2)` cap |
| Joints stretch under load | [Joint.hpp](../Joint.hpp) | iteration count, warm-start ratio, `maxForce`/`maxTorque` caps; long chains are documented as not ship-quality |
| A settled scene never sleeps | [Islands.hpp](../Islands.hpp) | velocities sampled after the force pass (must be the previous post-solve values), non-zero `driveTorque` on a member, or a motor/mouse joint holding the island active |
| Sleeping bodies wake every frame | [Solver.hpp](../Solver.hpp), [World.hpp](../World.hpp) | the sleeping guards in `prepare`, the warm-start apply loop, `solveIteration` and `correctPositions` must all use `rowIsLive`; erasing their manifolds breaks the island graph |
| Frames are slow | [World.hpp](../World.hpp) `timings` | set `profiling = true`, then compare `broadphase` vs `solve`; a settled scene should be dominated by proxy work, not narrowphase |
| Contacts appear at the wrong place | [Collision.hpp](../Collision.hpp) | reference/incident face selection and the A → B normal convention; a flipped normal makes the solver pull |
| A body spins up for no reason | [RigidBody.hpp](../RigidBody.hpp), [Joint.hpp](../Joint.hpp) | `driveTorque` is persistent (unlike `torque`), and `lockAngle = false` on a prismatic joint leaves rotation free on purpose |
| A TGS scene behaves like PGS | [World.hpp](../World.hpp) | `SolverConfig::mode`, `substeps`, and the mode-switch invalidation (`previousMode_` forces a cache clear) |
| A compliant contact settles at the wrong depth | [TemporalSolver.hpp](../TemporalSolver.hpp) | `compliance` is per contact *point*, so a two-point manifold is two springs in parallel: `depth − slop = α·m·g` per point |
| A rejected step lost your state | [World.hpp](../World.hpp) | capacity rejection preserves simulation state by contract (§6.1); re-check you read the status instead of ignoring it |
| The demo shows a blank canvas | [../web/showcase.js](../web/showcase.js) | check all nine browser assets, then console/network errors, frame cadence and step cost; the shell holds structure, not the frame pump |

---

## 11. Maintaining this document (it is meant to keep growing)

This file is part of the deliverable, not a snapshot. When the engine gains a feature, the
document gains three things and a row:

| Where | What to add | Why |
| --- | --- | --- |
| §5 (the right subsection) | **Intuition**, **Derivation**, and the exact symbol | the derivation is the part that cannot be re-derived from the code |
| §3 or §5 code map | the symbol, file and line it lives at | so the reader can check the claim instead of trusting it |
| the same subsection | **What is not proven** | an unfinished thing that is taught as done is the most expensive kind of error |
| Appendix A | one row: concept → symbol → file#line | the index is what makes the document usable as a reference |
| the relevant exercise | a question that the feature makes answerable | exercises are how a reader converts reading into ability |

And, outside this file, at most these, whichever apply:

* [BENCHMARKS.md](BENCHMARKS.md) if a *number* changed (it owns every timing table);
* [TGS.md](TGS.md), [PARALLEL.md](PARALLEL.md) or [STORAGE.md](STORAGE.md) if the change
  touches substepping, rows/colours, or capacity/layout;
* [../PHYSICS_ENGINE_GUIDE.md](../PHYSICS_ENGINE_GUIDE.md) §3/§4/§5 for a convention, an
  owner or a phase status;
* [FEATURE_ACCEPTANCE.md](FEATURE_ACCEPTANCE.md) for a tracked outcome;
* §2 and §8's counts, if the suite's assertion count changed — always with the receipt path
  you actually ran, never a copied number.

### 11.1 Keep the claims checkable

Every `ok:` count and measured value in this document came from a receipt in `build/` or a
table in [BENCHMARKS.md](BENCHMARKS.md). When you change the engine:

1. re-run the affected suite and record the new count;
2. update the number here and note what it was before (the historical `143` versus current
   `142`, with the reason, is the model);
3. if a number in two documents disagrees, [BENCHMARKS.md](BENCHMARKS.md) wins for timings
   and the suite itself wins for assertions;
4. never delete a failing criterion to make a document true. The sleeping and CCD
   benchmarks exit non-zero on purpose.

### 11.2 Check every link and every `#L` anchor

This document cites files by line, so the cheapest possible rot is a code change that moves
a symbol. Paste the block below into `check_docs.py` and run `python check_docs.py` from the
repository root. It exits non-zero on a missing file, an out-of-range line, or a `#L` anchor
whose link text names a symbol that is not on that exact line. Bare-number and file-path
labels are checked only for resolvability and for landing on a non-blank line, which is a
deliberate limitation — it catches a moved symbol or a typo, not a symbol that moved
*together* with its anchor. On the revision you are reading it reports `links: OK` and
exits `0`.

```python
import pathlib, re, sys

DOC = pathlib.Path('docs/FROM_SCRATCH.md')
doc = DOC.read_text(encoding='utf-8')
bad = []

for label, target in re.findall(r'\[([^\]]+)\]\(([^)]+)\)', doc):
    if target.startswith(('http://', 'https://', 'mailto:')):
        continue
    path_part, _, frag = target.partition('#')
    path = (DOC.parent / path_part).resolve() if path_part else DOC.resolve()
    if not path.exists():
        bad.append(f'missing file: {label} -> {target}')
        continue
    if not frag.startswith('L'):
        continue
    first = int(re.match(r'L(\d+)', frag).group(1))
    lines = path.read_text(encoding='utf-8', errors='replace').splitlines()
    if first > len(lines):
        bad.append(f'line out of range: {label} -> {target} ({len(lines)} lines)')
        continue
    text = lines[first - 1]
    # Link text is either a symbol (checkable) or a bare number / file path (not).
    symbol = re.sub(r':\d+$', '', label.split('::')[-1].strip('`')).split('(')[0].strip()
    if not symbol or symbol[0].isdigit() or '.' in symbol:
        # A bare line reference still must land on real code, not a blank line.
        if not text.strip():
            bad.append(f'blank line: {target}')
        continue
    if symbol not in text:
        bad.append(f'symbol not on {target}: looking for {symbol!r}, found {text.strip()[:60]!r}')

print('links:', 'OK' if not bad else f'{len(bad)} problem(s)')
for line in bad:
    print(' -', line)
sys.exit(1 if bad else 0)
```

Run it after every feature. It is weak on purpose — it checks *resolvability*, not meaning —
and it is exactly strong enough to catch the failure this document is most exposed to.

### 11.3 Changing a convention

The conventions in §1.1 and the layouts in §6.4 are referenced from several documents. If
you change one (for example `real` from `double` to `float`, or the normal direction),
change it here *and* in [../PHYSICS_ENGINE_GUIDE.md](../PHYSICS_ENGINE_GUIDE.md) §3, then
re-measure every layout in §6.4 on the target platform, because those numbers are
ABI-specific and will change.

---

## Appendix A. Concept → symbol → place

The one-page index. Section numbers are in this document; anything unmeasured or unfinished
is labelled in the text, not here.

| Concept | Symbol | Where |
| --- | --- | --- |
| Scalar type / epsilon | `real`, `kEps` | [Vec2.hpp:13](../Vec2.hpp#L13) |
| Vector type, 90° rotation, cross products | `Vec2`, `perp`, `cross` | [Vec2.hpp:15](../Vec2.hpp#L15), [36](../Vec2.hpp#L36), [54](../Vec2.hpp#L54) |
| Rotation matrix, Cramer solve | `Mat22`, `rotation`, `solve` | [Mat22.hpp:19](../Mat22.hpp#L19), [23](../Mat22.hpp#L23), [50](../Mat22.hpp#L50) |
| Bounds, fat proxies, SAH cost | `AABB`, `fatten`, `perimeter`, `computeAABB` | [AABB.hpp:10](../AABB.hpp#L10), [21](../AABB.hpp#L21), [28](../AABB.hpp#L28), [35](../AABB.hpp#L35) |
| Body state, static policy, guarded writes | `RigidBody`, `isStatic`, `applyImpulse`, `translate` | [RigidBody.hpp:30](../RigidBody.hpp#L30), [63](../RigidBody.hpp#L63), [84](../RigidBody.hpp#L84), [97](../RigidBody.hpp#L97) |
| Mass properties | `makeCircle`, `makeBox`, `makePolygon` | [RigidBody.hpp:109](../RigidBody.hpp#L109), [124](../RigidBody.hpp#L124), [137](../RigidBody.hpp#L137) |
| Material point velocity | `velocityAt` | [RigidBody.hpp:103](../RigidBody.hpp#L103) |
| Contact data + layout | `ContactPoint`, `ContactManifold` | [Manifold.hpp:9](../Manifold.hpp#L9), [26](../Manifold.hpp#L26) |
| Symplectic integration | `integrateForces`, `integrateVelocities` | [Integrator.hpp:24](../Integrator.hpp#L24), [36](../Integrator.hpp#L36) |
| Smooth-force study | `SymplecticPolicy`, `VerletPolicy`, `RK4Policy` | [IntegratorStudy.hpp:6](../IntegratorStudy.hpp#L6), [13](../IntegratorStudy.hpp#L13), [24](../IntegratorStudy.hpp#L24) |
| SAT, clipping, manifolds | `findMaxSeparation`, `clip`, `collidePolygons`, `collidePolygonCircle`, `collideCircles` | [Collision.hpp:39](../Collision.hpp#L39), [54](../Collision.hpp#L54), [154](../Collision.hpp#L154), [103](../Collision.hpp#L103), [79](../Collision.hpp#L79) |
| PGS solver, config | `ContactSolver`, `SolverConfig`, `solveIteration`, `SolverMode` | [Solver.hpp:64](../Solver.hpp#L64), [48](../Solver.hpp#L48), [136](../Solver.hpp#L136), [46](../Solver.hpp#L46) |
| Warm start / cache | `restoreImpulses`, `saveImpulses`, `cacheChanged` | [World.hpp:402](../World.hpp#L402), [424](../World.hpp#L424), [387](../World.hpp#L384) |
| NGS position correction | `correctPositions`, `Joint::correctPosition` | [World.hpp:334](../World.hpp#L334), [Joint.hpp:161](../Joint.hpp#L161) |
| Joints | `Joint`, `JointType`, factories | [Joint.hpp:8](../Joint.hpp#L8), [6](../Joint.hpp#L6), [22](../Joint.hpp#L22) |
| Sleeping / islands | `Islands`, `build`, `wakeConnected`, `sleep`, thresholds | [Islands.hpp:21](../Islands.hpp#L21), [26](../Islands.hpp#L26), [50](../Islands.hpp#L50), [70](../Islands.hpp#L70), [9](../Islands.hpp#L9) |
| TGS substepping | `TemporalPoint`, `TemporalContact`, `prepareTemporal`, `temporalSoftness`, `refreshTemporal`, `solveTemporal`, `World::stepTemporal` | [TemporalSolver.hpp:8](../TemporalSolver.hpp#L8), [12](../TemporalSolver.hpp#L12), [19](../TemporalSolver.hpp#L19), [39](../TemporalSolver.hpp#L39), [47](../TemporalSolver.hpp#L47), [80](../TemporalSolver.hpp#L80), [World.hpp:200](../World.hpp#L200) |
| CCD | `sweepRadius`, `separatingGap`, `closingSpeedBound`, `timeOfImpact`, `CCDStats`, `advanceContinuous` | [Continuous.hpp:68](../Continuous.hpp#L68), [105](../Continuous.hpp#L105), [133](../Continuous.hpp#L133), [149](../Continuous.hpp#L149), [183](../Continuous.hpp#L183), [194](../Continuous.hpp#L194) |
| Rows and colouring | `SolverRow`, `rowLess`, `rowIsLive`, `RowSet`, `colorRows`, `validateColoring` | [SolverRows.hpp:25](../SolverRows.hpp#L25), [37](../SolverRows.hpp#L37), [51](../SolverRows.hpp#L51), [70](../SolverRows.hpp#L70), [Schedule.hpp:31](../Schedule.hpp#L31), [69](../Schedule.hpp#L69) |
| Prepared storage | `Capacity`, `prepare`, `tryStep`, `StepStatus` | [World.hpp:50](../World.hpp#L50), [56](../World.hpp#L56), [75](../World.hpp#L75), [53](../World.hpp#L53) |
| Broadphase + tree | `Broadphase`, `findPairs`, `DynamicTree`, `insert`, `query` | [Broadphase.hpp:10](../Broadphase.hpp#L10), [21](../Broadphase.hpp#L21), [DynamicTree.hpp:8](../DynamicTree.hpp#L8), [131](../DynamicTree.hpp#L131), [42](../DynamicTree.hpp#L42) |
| Scenes | `makeStack`, `makePyramid`, `makeChain`, `kSceneCount`, `makeShowcase` | [Scenes.hpp:5](../Scenes.hpp#L5), [17](../Scenes.hpp#L17), [31](../Scenes.hpp#L31), [Showcase.hpp:17](../Showcase.hpp#L17), [280](../Showcase.hpp#L280) |
| Suite macro | `CHECK` | [../test_engine.cpp:14](../test_engine.cpp#L14) |
| Allocation counter | `testAllocations`, `countTestAllocations` | [../test_allocation_counter.cpp:5](../test_allocation_counter.cpp#L5) |
| Worked feature | `RigidBody::gravityScale` | §9 (added by the reader) |
