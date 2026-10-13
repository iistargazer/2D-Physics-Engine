# Warm starting: seeding the solver with last frame's answer

## The observation

A resting stack solves almost the same problem every frame. Gravity adds a small
velocity, and the solver must undo it. If the contact could be told what impulse
it needed last frame, the iteration would start near the answer instead of at
zero. Sequential impulses applied to a good starting point converge in far fewer
sweeps, and stability improves dramatically because the under-converged residual
that causes slow sinking is removed.

## Matching contacts between frames

Reusing an impulse is only valid if it belongs to the same geometric constraint.
`World` keys the cache by the ordered body index pair:

    std::map<BodyPair, CachedManifold> cache_;

and each cached point stores a `featureId` plus its `accN` and `accT`. The
feature ID is the topology encoding described in the clipping note: reference
owner, reference face, incident face, and endpoint or clipping side. Positions
change freely; the identifier does not.

Two guards are applied before reuse in `World::restoreImpulses`:

* the stored manifold normal must still agree, `dot(stored.normal, m.normal) >= 0.95`;
* the point's `featureId` must match exactly.

If the normal reversed, the contact is no longer the same constraint. If the
feature changed, the impulse belongs to a different lever arm.

## Restitution must be computed before seeding

This is the subtle ordering rule. Restitution is defined by the *approach* speed
of the collision. If cached impulses are applied first, the approach speed is
already partly cancelled, and the bounce computed from it is too small.

`ContactSolver::prepare` therefore runs in two phases. The first loop computes
`targetVn` for every contact point using the untouched incoming velocities. Only
after that loop completes does the second phase apply the cached impulses:

    const Vec2 p = m.normal * c.accN + t * c.accT;
    m.bodyA->applyImpulse(-p, c.rA);
    m.bodyB->applyImpulse( p, c.rB);

with `t = (m.normal.y, -m.normal.x)`. The equal-and-opposite pair preserves
linear momentum exactly.

## Scaling, expiry and invalidation

Impulses approximate `force * dt`, so the cached values are rescaled by the
timestep ratio `dt / previousDt` in `restoreImpulses` and in `Joint::prepare`.
The shipped demos use a fixed 1/60 step, where the ratio is one.

`World::saveImpulses` clears the cache and repopulates it from the manifolds that
actually exist this frame. Contacts that separated simply disappear, so a cached
impulse can never resurrect a dead contact.

Public mutation of `World::bodies` is the dangerous case: indices can be reused
by a different body. `World::validateCache` compares the last post-step snapshot
of position, velocity, angle, angular velocity, inverse mass, inverse inertia and
the shape's vertices and radius. Any difference clears the contact cache, resets
joint impulses, and wakes sleepers. Forces and torques are deliberately excluded,
since applying a force is a normal input and should not throw away the solution.

## What it changes, measured

`benchmark_stability.cpp` compares cold and warm starting on the same scenes.
Scalar-only warm starting still leaves the 20-box column swaying; warm starting
combined with the two-point block solve holds it. The numbers live in the
project guide and README, and the raw CSV is written to `build/stability.csv`.
Warm starting is not a correctness fix on its own -- it is a convergence
accelerator that makes an already correct iteration finish in fewer sweeps.
