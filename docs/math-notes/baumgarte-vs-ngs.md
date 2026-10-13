# Fixing penetration: velocity bias versus position projection

## The problem

The contact solver works on velocities. A contact whose velocity constraint is
exactly satisfied can still be geometrically overlapping, because nothing in the
velocity solve looks at position. Left alone, errors accumulate and stacks sink
into each other.

Two standard repairs exist, and this engine implements both, with Baumgarte as
the default and NGS as an opt-in (`SolverConfig::positionCorrection`).

## Baumgarte stabilisation

Inject the error into the velocity constraint as a bias. If the position error is
`C = -depth` and it should decay exponentially with rate `beta / dt`, demand

    C' = -(beta / dt) * C

which as a separation target on the normal speed becomes

    target vn >= (beta / dt) * max(depth - slop, 0)

In `ContactSolver::prepare`:

    const real push = (cfg_.positionCorrection ? 0 : cfg_.baumgarte / dt)
                    * std::max(c.depth - cfg_.slop, real(0));
    c.targetVn = std::max(bounce, push);

`beta` is typically 0.2. `slop` is a deliberate allowance (0.01 here) that lets
shapes stay slightly overlapped so contacts persist frame to frame instead of
flickering on and off at exactly zero depth. Restitution and push-out are
combined with `std::max`, not added: bouncing away and being pushed apart are not
independent effects.

The cost is physical. The bias is a real velocity that the solver then integrates
into position. For a body resting on the ground it means spurious kinetic energy
appears and then has to be removed again, and when `beta` is raised for faster
correction, the injected energy grows with it.

## Nonlinear Gauss-Seidel position correction

Instead of moving velocity, move position. After the velocity solve and
integration, `World::correctPositions` runs `positionIterations` passes. Each pass
recomputes the contact geometry with `collide`, and for each point applies

    correction = positionBeta * clamp(depth - slop, 0, 0.2) / K
    A.position -= n * A.invMass   * correction
    B.position += n * B.invMass   * correction
    A.setAngle(A.angle - A.invInertia * (rA x n) * correction)
    B.setAngle(B.angle + B.invInertia * (rB x n) * correction)

`K` is the same effective mass used by the velocity solve. Because the geometry is
recomputed between corrections, this is nonlinear Gauss-Seidel rather than a
single linear projection. Velocities are never touched, so no kinetic energy is
invented; only potential energy changes as positions move. Corrections are capped
per pass so a deep overlap cannot teleport a body in one iteration.

Rigid joints get the same treatment through `Joint::correctPosition`. Soft
distance springs (frequency > 0) and the mouse joint are skipped, since forcing
them to their exact rest length would defeat the purpose of a soft constraint.

## The measured comparison

From the stability benchmark, one zero-gravity box overlapping the ground,
single step, ten position passes:

| Correction | Kinetic energy added | Residual depth |
| --- | --- | --- |
| Baumgarte | 2.5992 J | 0.162000 |
| NGS | 0 J | 0.022215 |

Baumgarte adds kinetic energy where NGS adds none, and one NGS step already
leaves a much smaller residual penetration. The figures are transcribed in [BENCHMARKS](../BENCHMARKS.md) from
[stability.csv](../../build/stability.csv).

NGS is not free conservation. Moving positions changes potential energy, changing
positions can create or destroy contacts, and finite iterations leave residual
error. It is a better trade, not a perfect one.

## Why Baumgarte is still the default

The position pass costs extra narrowphase work per iteration and changes the
sequence in which poses are finalised, which complicates anything that inspects
the world mid-step. The default path stays closest to the original design and to
the existing regression tests; the benchmark retains both configurations rather
than silently switching one for the other.
