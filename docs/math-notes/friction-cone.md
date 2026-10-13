# Coulomb friction as a clamped tangent impulse

## The physical model

Coulomb friction says the tangential force cannot exceed the normal force scaled
by the friction coefficient:

    |Ft| <= mu * Fn

As impulses over a timestep this becomes `|lambda_t| <= mu * lambda_n`. The shape
of that inequality is why it is called the friction cone. In 2D the "cone" is a
pair of rays; the admissible tangential impulse is the interval
`[-mu*lambda_n, +mu*lambda_n]`.

## Choosing the tangent

The engine uses

    t = (n.y, -n.x)

which is `n` rotated by -90 degrees, a unit vector perpendicular to the normal.
The sign convention only has to be consistent: a positive `lambda_t` slides `B`
along `+t` and `A` along `-t`, and the effective mass is derived with the same
`t`.

Because the effective mass along `t` differs from the one along `n` in general,
`prepare` computes a separate `massT` from

    K_t = mA^-1 + mB^-1 + IA^-1 (rA x t)^2 + IB^-1 (rB x t)^2

## The solve

Inside `ContactSolver::solveIteration`, before the normal pass:

    const Vec2 vrel = B.velocityAt(c.rB) - A.velocityAt(c.rA);
    real lambda = -dot(vrel, t) * c.massT;
    const real maxF = m.friction * c.accN;
    const real newAcc = clamp(c.accT + lambda, -maxF, maxF);

The first line asks for the impulse that would cancel the relative tangential
speed at this point. The second line clamps the running total into the cone,
which is what makes the update a projection rather than an unconstrained solve.
Note the asymmetry: a perfectly tangential impulse that exactly cancels `vt` is
what static friction looks like; when the clamp bites, the contact slides and the
residual tangential motion is genuine kinetic friction.

## Why friction runs first

The normal pass is the one that enforces non-penetration, and it uses `accN`
which the friction pass reads. Solving friction first means the last word in each
sweep belongs to the normal constraint, so the friction bound is never computed
from a stale normal impulse that is about to grow. The `maxF` seen by the next
iteration is therefore always based on a normal impulse that already satisfies
the current velocity state.

## Material mixing

`prepare` combines the two bodies' coefficients with a geometric mean:

    m.friction = sqrt(A.friction * B.friction);

Restitution uses `std::max(A.restitution, B.restitution)`. Both are conventions,
not derivations: the geometric mean punishes a slippery contact only mildly,
which tends to feel reasonable, and taking the maximum restitution prevents a
bouncy body from being damped by a static floor that was authored with the
default value.

## Scope

There is no rolling friction, no cone twisting, and no anisotropic friction. The
cone is the isotropic Coulomb model in the tangent direction only. Block solving
is applied to the normal direction and to the coupling between the two contact
points' normal impulses; the tangential impulses remain scalar per point despite
sharing the same `k12` coupling, which is a known simplification.
