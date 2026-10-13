# Scene catalogue

Every scene in `sceneNames` (`Showcase.hpp`) is a plain `World` factory, so the same
definition drives the headless tests, the benchmarks, the SFML demo and the browser
build. Nothing here exists only in the renderer.

How to reach each scene:

| Host | How |
| --- | --- |
| SFML demo | keys `1`-`9` load scenes 1-9, key `0` loads scene 10 (momentum row) - `demo/main.cpp` |
| Browser build | navigator or keys `1`-`9` / `0`; metadata/camera in [showcase.js](../web/showcase.js), operation in [SHOWCASE](SHOWCASE.md) |
| Headless | `makeShowcase(scene)` in `benchmark_showcase.cpp`, `test_engine.cpp`, or a scene argument to `demo_broadphase` (`bridge`, `domino`) |

Scene 7 is the only scene that enables sleeping (`w.sleeping = true`). Warm starting is
on for all scenes (the `SolverConfig` default); NGS position correction is opt-in and is
turned on for scenes 2-7. Only scene 8 sets `bullet`, so CCD is idle everywhere else.

---

## 0 - Newton's cradle

Factory: `makeCradle(5, true)`.

One static pivot circle (r 0.05) at `(0, 3)`, five dynamic balls (r 0.5, m 1) at
`y = 2`, `x = -2 ... 2`, `restitution = 1`, `friction = 0`, each hung from the pivot by a
distance joint of rest length 1.0 whose local anchor is `(x, 0)` on the pivot - the pivot
is unrotated, so that local point is exactly above the ball it carries. 40 solver
iterations. The left ball starts lifted by 0.7 rad (40.1 deg) at `(-2.644, 2.235)` and is
released from rest.

Principles: elastic collision chain with equal masses, momentum transfer through
sequential impulses, distance constraints solved alongside contacts in the same
iteration loop, warm starting carrying the chain's impulse state between frames.

Watch for: the left ball swings in and stops dead while the right ball leaves with the
incoming speed.

Stresses: simultaneous joint + contact coupling, restitution at 1.0 (the worst case for
the accumulated-impulse clamp). Weak spot: the visual cradle is a five-link pendulum, not
a true Hertzian contact chain, so its far-ball speed is approximate; the strict claim is
made on scene 9 instead.

Coverage: `testPendulumCradle()` - "far ball is driven by the chain: peak speed=2.1354 m/s",
"far ball swings out: reached x=2.6398 (anchor 2.1396)". Joint machinery generally is
covered by `testJoints()` ("small-angle pendulum period=2.8333 s", "10-link revolute chain
anchor error=0.000035").

## 1 - Domino run

Factory: `makeDominoRun(45)`.

Static floor (120 x 1) at `(0, -0.5)`, then 45 dominoes of `0.18 x 1.4` m, m 1, spaced
0.65 m apart from `x = -18` to `x = 10.6` at `y = 0.7`. 20 solver iterations. The first domino is given `angularVelocity = -2` rad/s;
the row must do the rest.

Principles: sequential-impulse stacking through a chain of 45 touching-but-unwelded
bodies, friction (default mu 0.4) as the hinge that makes each plate tip instead of
slide, warm starting keeping a leaning domino's contact impulses stable frame to frame,
and box-on-box manifold clipping as the plates land on each other.

Watch for: one flick of the first plate topples all 45 without any scripted help.

Stresses: large ordered contact chains, manifold reuse while bodies rotate fast. Weak
spot: the cascade is slow in simulated time - only one plank has fallen after 1 s and the
row is still not fully down after 6 s - and the *shape* of the wave (planks leaning on
each other, spacing collapse) is not measured, only the stand-up count and the causality.

Coverage: `testDominoCascade()` - "every plank starts upright: 45/45", "the far plank
cannot know about the nudge yet: |angle|=0.000000 rad" (after one second), "the wave
propagates one plank at a time: standing 44 -> 33 -> 14 -> 0" (at 1 s, 3 s, 6 s, 12 s),
"the last plank falls flat: |angle|=1.5708 rad", "without the nudge the row is inert for
10 s: max |angle|=0.000000000 rad". `demo_broadphase -- domino` loads it for manual
inspection.

## 2 - Wrecking pyramid

Factory: `makeWreckingPyramid(20)`.

`makePyramid(20)` builds a static ground plus 210 unit boxes (row `r` holds `20 - r`
boxes, all `restitution = 0`). A static anchor (r 0.1) sits at `(-6, 24)` with a 25 kg
ball (r 1.2) on a 19 m distance joint, released at 45 deg from `(-19.436, 10.564)`; the
rope's lowest point `(-6, 5)` passes through row 4 of the pile. 40 iterations, NGS on.
213 bodies total.

Principles: the hardest known stability case in the engine - a tall frictionless-ish
pyramid stacked to 20 rows under sequential impulses; warm starting for stack
convergence; NGS position correction removing overlap without adding kinetic energy;
distance constraint carrying a large mass ratio (static anchor vs 25 kg ball).

Watch for: the pile sits perfectly still while the ball swings in, then scatters.

Stresses: contact-count growth, iteration convergence, mass-ratio extremes. Weak spot:
permanent residual penetration of one slop distance is expected; the pile is measured by
residual velocity, not by absolute position.

Coverage: `testWreckingPyramid()` - "ball has not reached the pile yet: x=-13.402",
"pyramid is at rest while the ball swings in: max |v|=0.00234 m/s", "pyramid is wrecked
after the impact: max drift=8.5180 m". Tall-stack convergence generally:
`testTallScenes()` - "stack retains height: top=19.3878 ideal=19.5",
"pyramid retains height: top=14.3791 ideal=14.5".

## 3 - Ragdoll

Factory: `makeRagdoll()`.

Static floor plus seven bodies: a 0.9 x 1.4 m torso (m 3) at `(0, 6)`, a head circle
(r 0.4, m 1), two arms of 1 x 0.25 m (m 1) and two legs of 0.3 x 1.4 m (m 1). Five revolute
joints with angle limits: neck `(-0.5, 0.5)`, shoulders `(-1.5, 1.5)`, hips
`(-0.8, 0.8)`. 20 iterations, NGS on. This is the scene to drag with the left mouse
button; the mouse joint is a soft distance constraint with a damping ratio.

Principles: revolute constraints solved as a 2x2 block, speculative angle limits that
stop at the target angle without adding energy, soft constraints (`gamma`/`beta` from
frequency and damping ratio) for the mouse grab, and jointed-body islanding.

Watch for: drag the torso around - the arms and legs follow, and the limits stop the
joints folding through the body. Left alone the figure falls about 4 m (the torso starts
at `y = 6`, the head at `y = 7.2`) and ends standing on the floor at `y = 2.094` and
`3.235` respectively.

Stresses: multi-joint coupling at 20 iterations (the fewest of the jointed scenes). Weak
spot: this scene has the largest warm-started joint error measured anywhere in the
showcase set - `max_joint_error = 0.034975` in `build/showcase.csv`, over three times the
bridge's 0.010674 - because the shoulder and hip anchors are offset from the limb
centroids and the limbs swing hard on landing. The scene does not enable sleeping, so the
settled figure stays awake and keeps paying for the joints.

Coverage: `testRagdoll()` - "revolute limits hold: worst relative angle=1.5002 rad
(limit 1.5)", "the ragdoll settles to rest: max |v|=0.000000 m/s", "the ragdoll rests on
the floor instead of hovering: lowest point y=-0.00726", "the head stays above the torso
after landing: 3.235 > 2.094", and the printed line "info: ragdoll anchor error 0.034975 m
(worst warm-started joint in the project)". The individual joint behaviours are asserted
separately in `testJoints()` ("motor respects angular limit: angle=0.300000", "mouse
spring converges to target", "prismatic constrains intended degrees of freedom", "weld
constrains intended degrees of freedom").

## 4 - Suspension bridge

Factory: `makeBridge(100)`.

Two static anchor circles (r 0.1) at `(-20, 5)` and `(20, 5)`, plus 100 planks of
`0.4 x 0.15` m at `x = -19.8 + 0.4 i`, linked end to end by revolute joints 0.2 m from
each plank centre. 100 solver iterations, 60 position iterations, NGS on - the heaviest
iteration budget in the project. 102 bodies.

Principles: rigid-limit position projection on a 100-link constraint chain (velocity
iterations alone let a long chain stretch), revolute anchor compliance, and the cost of
solving a long serial joint chain in one island.

Watch for: the deck sags by a centimetre and never tears open, however hard it is
loaded.

Stresses: position-projection convergence over 99 chained joints. Weak spot: the
iteration counts are tuned for this scene specifically; a chain this long is exactly
where a full LCP or a sparse direct joint solve would beat sequential impulses.

Coverage: `testBridgeProjection()` - "100-plank bridge projection max anchor error=0.010674"
(threshold 0.02). Joint anchor error is also sampled by `benchmark_showcase.cpp` for
every revolute/weld scene, and the 100-link chain *without* projection is kept as a
recorded failure in `build/joints.csv` (3.021928 m at 40 sweeps).

## 5 - Motorized car

Factory: `makeCar()` / `makeCarRig(drive = -5, springFrequency = 2.5)`.

Static road 300 x 1 m, plus three low terrain boxes (two 4 x 0.3 m at `x = 18` and
`x = 30` tilted +/-0.05 rad, one 3 x 0.6 m step at `x = 44`), a 3.2 x 0.5 m chassis
(m 6, friction 0.6) and two wheel circles (r 0.45, m 1, friction 1.5). Each wheel is
mounted by a prismatic joint along the chassis-local y axis with `lockAngle = false`, so
the wheel slides vertically and stays free to spin; a soft distance joint (rest 0.7 m,
2.5 Hz, damping ratio 1.0) is the spring, and `driveTorque = -5` is a persistent wheel
motor that survives every step. 20 iterations, 20 NGS position iterations. 7 bodies.

Principles: a constrained degree-of-freedom split - the prismatic joint removes one
translational axis but must leave rotation free (`lockAngle`), which is what makes the
axle a suspension rather than a weld; soft constraint springs; persistent torque as a
motor; friction as traction (mu 1.5 at the contact patch, 0.6 on the chassis).

Watch for: the wheels spin the whole time while the chassis stays level and the springs
compress and extend over the bumps.

Stresses: mixed prismatic + soft-distance + contact solving on the same bodies. Weak
spot: torque is applied every step for the whole run, which is a motor, not an engine -
there is no gearbox, no rolling resistance and no wheel-vs-chassis contact.

Driving is entirely internal: neither the SFML demo nor `benchmark_showcase.cpp` applies
torque from outside any more, because `RigidBody::driveTorque` persists across steps.
The scene is self-contained in all three front ends.

Coverage: `testMotorizedCar()` - "wheels spin freely through the prismatic axle
(lockAngle=false): peak |w|=63.32 rad/s, turned 2.63 rad", "car drives itself across the
terrain: 48.90 m", "car never tips over: max |angle|=0.322 rad", "car finishes level:
0.0038 rad", "suspension compresses and extends under load: 0.0082..0.7153 m (rest 0.70)".

## 6 - Rube-Goldberg machine

Factory: `makeRubeMachine(trigger)` / `makeRubeGoldberg(trigger)`.

Four stages on a static floor: a static ramp (5.2 x 0.3 m) at `(-12.5, 2.2)` tilted
-0.30 rad, eight dominoes of `0.18 x 1.3` m spaced 0.62 m from `x = -9.4`, a weighted
hammer (0.3 x 2.0 m, m 3) hanging from a static pivot at `(-4.34, 3.1)` on a 1 m distance
joint, and a flag (0.25 x 1.7 m) at `(-3.19, 0.85)`. The trigger is a 2 kg ball (r 0.2,
restitution 0.05) at `(-14.3, 3.35)`; `makeShowcase(scene, trigger)` passes the switch
through, so the same scene can be built armed or inert. 25 iterations, NGS on. 14 bodies
with the ball, 13 without.

Principles: pure causal chaining - rolling contact on a slope, momentum transfer into a
domino line, impact into a pendulum, and a distinct final body whose state is the
machine's output. Nothing about the outcome is scripted; if any stage loses energy in
the wrong place the chain dies.

Watch for: with the ball the flag ends flat; rebuild without the ball and the machine
never moves at all.

Stresses: long causal chains, momentum transfer across bodies of different mass (2 kg
ball, 3 kg hammer, dominoes of m 1), and static-vs-dynamic friction on a shallow ramp.

Coverage: `testRubeGoldbergCausality()` - "with the trigger the machine completes: flag
angle=-1.5684 rad" (fully flat, i.e. -90 deg) and "without the trigger nothing moves:
flag angle=0.0000 rad".

## 7 - Stress: 5000 boxes

Factory: `makeStress(5000, true, 500)`.

5,000 unit boxes of `0.5 x 0.5` m (m 1) in 500 columns and 10 rows, 0.55 m apart,
starting at `y = 0.3`, on a static floor of width 275 + 40 m, under normal gravity.
Sleeping is enabled, 10 iterations, 6 NGS position iterations. 5,001 bodies - the only
scene with sleeping, and the only one where the step cost is the point.

Principles: broadphase scaling on a 5,000-body field, island construction from the
contact graph, sleep thresholds and island-wide wake-up, and the difference between the
transient cost (everything awake) and the settled cost (everything asleep).

Watch for: the field settles, the boxes turn grey, and the HUD step time drops - 5,000
bodies simulating is cheaper than 5,000 bodies waking up.

Stresses: dynamic AABB tree maintenance at 5,001 proxies, and the sleeping pass. Weak
spot: this is a wide, shallow field chosen so it *can* settle; a tall pile with the same
iteration budget jitters forever and never sleeps (the factory comment says so
explicitly).

Coverage: `testStressSleeping()` - "settled field sleeps: 5000/5001 dynamic bodies
asleep", "sleeping stops residual drift: max |v|=0.000000", "sleeping cuts steady-state
cost: 21.5722 ms -> 15.1470 ms per step" and the printed line "5001-body settled step
15.1470 ms (66.0 simulated FPS equivalent)". The denominator includes the static floor;
all 5,000 dynamics sleep. Timing compares first 30 steps with 120 after 600 settling
steps, not controlled sleeping-on/off poses or rendered throughput. Sustained active
throughput for this field is measured separately in
[BENCHMARKS](BENCHMARKS.md#active-interaction-load-5000-bodies-awake), which owns all
timing tables, including the browser-driven stress run. Island rules generally:
`testStabilityUpgrades()` - "contact-connected stack sleeps as one island", "external
force wakes the entire connected island", "static floor does not merge independent
islands".

## 8 - Bullet vs wall (CCD)

Factory: `makeBulletScene(speed = 900)`.

No gravity. A static wall 0.05 x 12 m at `(0, 6)` and a thin bullet plate of
`0.10 x 0.03` m at `(-12, 6)` rotated 0.12 rad, `bullet = true`, `restitution = 0`,
`velocity = (900, 0)`. At 900 m/s the plate covers 15 m per 1/60 s step, 300 times the
wall thickness, so narrowphase sampling alone cannot possibly see the wall.

Principles: conservative advancement - sub-stepping each moving pair to its time of
impact - and the contract the engine guarantees: a touching pair is a contact, never a
miss; impulses are one-sided (push only, `lambda >= 0`); and the impact impulse is
applied **at the centre of mass**. That last rule is deliberate: conservative
advancement bounds *when* a pair touches, not *where*, so the sweep removes the relative
normal motion without inventing a lever arm and a torque. An earlier version applied the
impulse at an approximate contact point and a 3 mm plate picked up ~8e3 rad/s from a
6 mm offset, then held a motionless contact point while its centre crept into the wall
at 72 m/s. The sweep omits off-center torque; later discrete manifolds do not recover the correct
instantaneous angular response. Joint re-solving and connected-body exclusions during
casts are also absent. The per-body motion clamp and the positional safety net still exist,
but they are now nets: they report themselves through `ccdStats.clamped` / `pushedOut`
and are asserted idle in the six nonspinning wall cases. An event-cap CSV alone does
not prove clamp/push-out counters are zero.

Watch for: the fastest object on screen travels 15 m per frame, stops dead on the wall,
and the rest of the world keeps simulating normally while it happens.

Stresses: sweep robustness for both shape kinds, time-of-impact convergence for rotating
pairs, and the interaction between the swept path and the discrete solver. Weak spot:
the candidate pairs are still generated by a per-pair cast rather than a swept-tree
query, so a scene full of bullets would be quadratic; impact friction is applied at the
centre of mass, so a swept impact cannot impart spin of its own.

Coverage: `testBulletScene()` on the factory itself - "the bullet closes 15 m of distance
in one step: x=-12.0000 -> -0.076436", "the bullet stops before the wall instead of
appearing behind it: x=-0.076436115", "the bullet comes to rest at the wall:
|v|=0.000000000 |w|=0.000000000", "the impact is a resolved event, not a capped budget:
events=1 capped=0", "the resting bullet never creeps through the wall and never moves the
wall: x=-0.076436115". The mechanism is asserted in `testCCD()` on equivalent worlds -
for every combination of {circle, thin tilted polygon} x {200, 1000, 10000} m/s:
"stops before 0.02m wall: x=-0.050000 / -0.050799 events=1 pushed=0", "ends the step out
of the wall (depth=0.000000)", "<shape> impact absorbs the normal velocity:
vx=0.000000000", "<shape> comes to rest with no invented spin: |v|=0.000000000
|w|=0.000000000", "<shape> resolves inside the sweep with no fallback: clamped=0
pushedOut=0 unconverged=0"; plus "spinning thin bullet does not tunnel: x=-0.310636",
"spinning plate is held without losing its spin: |v|=0.000000000 w=60.000000",
"unrelated body keeps integrating during a bullet step: x=0.050000000", "already-touching
pair is skipped by CCD and pushed out discretely: x=0.400000", "unobstructed bullet
travels the full step: x=8.000000000", "moving/moving circle TOI matches analytic root:
0.15000000". `benchmark_ccd` sweeps the same shapes 100 times each, reports
`capped_trials = 0` for all six rows in `build/ccd.csv`, and exits 1 if any trial leaves
the bullet past `x > 0.1`.

## 9 - Momentum row

Factory: `makeMomentumRow(5, 5, 0.01)`, reached with the `0` key.

Five frictionless balls (r 0.5, m 1) in a row at `y = 0` spaced 1.01 m apart, zero
gravity, `restitution = 1`, `friction = 0`, 30 iterations. The first ball is moved to
`x = -2` and given `velocity = (5, 0)`.

Principles: this is the strictest single claim in the project - with no gravity, no
friction and no constraints, total linear momentum is an exact conserved quantity, so
momentum drift is a direct measure of solver error rather than a visual impression.

Watch for: the incoming ball stops dead and the far ball leaves at 5 m/s, with the three
middle balls never visibly moving.

Stresses: accumulated normal-impulse clamping under restitution 1.0, manifold generation
for exactly-touching circles, and determinism. Weak spot: nothing here exercises
rotation or friction, so it says nothing about angular momentum.

Coverage: `testMomentumConservation()` - "total x momentum conserved: 5.000000000 ->
5.000000000", "incoming ball stops: vx=0.000000000", "far ball leaves with the incoming
speed: 5.000000 m/s", "far ball keeps the momentum: 5.000000". The wider randomized
version is `testFuzz()` - "fuzz has no energy blow-up: max/initial=1.00000000" and
"fixed seed produces bit-identical repeated trajectories".

---

## Feature matrix

`warm` = warm starting, `sleep` = sleeping enabled for the scene, `joints` = constraint
types present, `NGS` = non-linear Gauss-Seidel position correction, `tree` = dynamic AABB
tree broadphase (the default for every scene), `CCD` = a body with `bullet = true`.

| # | Scene | warm | sleep | joints | NGS | tree | CCD |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 0 | Newton's cradle | yes | no | distance | no | yes | no |
| 1 | Domino run | yes | no | - | no | yes | no |
| 2 | Wrecking pyramid | yes | no | distance | yes | yes | no |
| 3 | Ragdoll | yes | no | revolute + limits, mouse | yes | yes | no |
| 4 | Suspension bridge | yes | no | revolute chain | yes | yes | no |
| 5 | Motorized car | yes | no | prismatic (free spin) + soft distance | yes | yes | no |
| 6 | Rube-Goldberg | yes | no | distance | yes | yes | no |
| 7 | Stress 5000 | yes | yes | - | yes | yes (5,001 proxies) | no |
| 8 | Bullet vs wall | yes | no | - | no | yes | yes |
| 9 | Momentum row | yes | no | - | no | yes | no |

## Measured cost and behaviour

This historical summary is transcribed from `build/showcase.csv`; the authoritative
full tables/methodology are in [BENCHMARKS](BENCHMARKS.md). `mean_ms` includes the timed
harness's finiteness scans and joint-error measurements, not just one fixed physics step;
`max_joint_error` is the worst revolute/weld anchor gap seen over the run;
`awake_end` counts dynamic bodies still awake in the last frame; `ccd_events`,
`ccd_capped` and `ccd_clamped` are the sweep's own counters.

| # | Scene | bodies | frames | mean_ms | total_ms | max_joint_error | awake_end | ccd |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0 | Newton's cradle | 6 | 240 | 0.004888 | 1.173 | 0.000000 | 5 | 0 |
| 1 | Domino run | 46 | 240 | 0.082018 | 19.684 | 0.000000 | 45 | 0 |
| 2 | Wrecking pyramid | 213 | 240 | 2.527189 | 606.525 | 0.000000 | 211 | 0 |
| 3 | Ragdoll | 7 | 240 | 0.018766 | 4.504 | 0.034975 | 6 | 0 |
| 4 | Suspension bridge | 102 | 240 | 0.789273 | 189.425 | 0.010674 | 100 | 0 |
| 5 | Motorized car | 7 | 240 | 0.009571 | 2.297 | 0.000000 | 3 | 0 |
| 6 | Rube-Goldberg | 14 | 240 | 0.058080 | 13.939 | 0.000000 | 11 | 0 |
| 7 | Stress 5000 | 5001 | 600 | 13.689410 | 8213.646 | 0.000000 | 0 | 0 |
| 8 | Bullet vs wall | 2 | 240 | 0.003877 | 0.931 | 0.000000 | 1 | 1 event |
| 9 | Momentum row | 5 | 240 | 0.001796 | 0.431 | 0.000000 | 5 | 0 |

Reading the table:

* `awake_end` equals the dynamic body count for scenes 0-4, 6 and 9 because those scenes
  do not enable sleeping - being awake there means "not sleeping", not "still moving".
  Scene 5 ends with 3 awake (chassis plus two wheels), which is every dynamic body it has.
* Scene 7 is the one row that settles: after 600 frames (10 s) `awake_end = 0`, so all
  5,000 dynamic bodies are asleep (the 5,001st body is the static floor) and the end of
  that run is genuinely cheap. Its `mean_ms`
  nevertheless covers the whole 600-frame settle window, so it is **not** a steady-state
  cost - the steady-state figure is the one `testStressSleeping()` prints after settling
  (15.1470 ms per step, reached from 21.5722 ms in the transient).
* `max_joint_error` distinguishes the sampled jointed scenes: the bridge
  chain projects to 0.010674 m, the ragdoll - whose limb anchors sit off the limb
  centroids - reaches 0.034975 m, and the scenes whose only joint is a distance
  constraint report 0.000000 (that column only samples revolute and weld joints).
* Scene 8 reports `ccd_events = 1`, `ccd_capped = 0`, `ccd_clamped = 0`.
  This table does not include `pushedOut` or `unconverged`; those need separate assertions,
  not inference from an event-cap column.
* Scene 2 is the most expensive non-stress scene at 2.53 ms per step for 213 bodies,
  which is the price of a 20-row pyramid at 40 velocity iterations and the default 10
  position iterations. Its 240-frame window covers both phases of the scene: the ball
  swings in for roughly the first 2.2 s (a 19 m pendulum quarter period) and the pile is
  scattered for the rest, so the mean is not a single-state measurement.

## Assertion coverage per scene

Every scene now has at least one named assertion, quoted in its section above:

| # | Scene | Dedicated test |
| --- | --- | --- |
| 0 | Newton's cradle | `testPendulumCradle()` |
| 1 | Domino run | `testDominoCascade()` |
| 2 | Wrecking pyramid | `testWreckingPyramid()` |
| 3 | Ragdoll | `testRagdoll()` |
| 4 | Suspension bridge | `testBridgeProjection()` |
| 5 | Motorized car | `testMotorizedCar()` |
| 6 | Rube-Goldberg | `testRubeGoldbergCausality()` |
| 7 | Stress 5000 | `testStressSleeping()` |
| 8 | Bullet vs wall | `testBulletScene()` (factory) + `testCCD()` (mechanism) |
| 9 | Momentum row | `testMomentumConservation()` |

What is still not asserted:

* `testShowcaseSmoke()` remains the only guard on the *factories as a whole*: it builds
  all ten scenes and steps each for 30 frames, asserting finite state ("all 10 showcase
  scene factories produce finite states"). A scene that renders nothing, or renders the
  wrong thing, would still pass it.
* Nothing measures the appearance of a scene. There is no rendered-output assertion and
  no image comparison anywhere in the project; the numbers above are state, not pixels.
* The optional-behaviour paths inside a scene are covered indirectly at best: the mouse
  joint is asserted in `testJoints()` rather than on a dragged ragdoll, and the
  `demo_broadphase` replay exports are not part of the suite.
