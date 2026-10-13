# Effective mass: turning an impulse into a change in constraint velocity

## The question it answers

If I push body `B` and pull body `A` with an impulse of magnitude `lambda` along
the contact normal, how much does the relative normal speed change? The answer is
`lambda * K`, and `K` is the effective inverse mass of the constraint. The
impulse that achieves a target relative speed `v*` is then

    lambda = (v* - vn) / K

where `vn` is the current relative normal speed.

## Derivation

An impulse `P = lambda * n` applied at lever arm `r` relative to the center of
mass changes the velocities as

    v'   = v + m^-1 P
    w'   = w + I^-1 (r x P)

The velocity of the material point at `r` is `v + w x r`, so the relative normal
speed at the contact changes by

    delta vn = lambda * [ mA^-1 + mB^-1
                        + IA^-1 (rA x n)^2
                        + IB^-1 (rB x n)^2 ]

Everything in brackets is a scalar assembled from inverses only. That is why
static bodies need no special case: their `invMass` and `invInertia` are zero, so
they contribute nothing and cannot be accelerated.

This is the scalar case of the general identity

    K = J M^-1 J^T

where `J` is the constraint Jacobian and `M` is the block-diagonal mass matrix of
both bodies.

## Where it is computed

`ContactSolver::prepare` computes it once per contact point per frame, because
the lever arms and the normal are fixed during the solve:

    const real rnA = cross(c.rA, n), rnB = cross(c.rB, n);
    const real kN = A.invMass + B.invMass
                  + A.invInertia * rnA * rnA + B.invInertia * rnB * rnB;
    c.massN = kN > 0 ? 1 / kN : 0;

`c.massN` stores the reciprocal `1 / K`, so the inner loop is one multiply instead
of a divide. The guard against zero matters for two static bodies, which cannot
occur in normal use but would otherwise produce a division by zero.

The tangential version replaces `n` with `t = (n.y, -n.x)`. Because
`K_n + K_t` is not constant, friction needs its own `massT`; it is not derived
from `massN`.

## The 2x2 case

A revolute joint constrains a two-vector, so the effective mass becomes a matrix.
`Joint::prepare` builds it from the same building blocks:

    Kxx = m^-1 + IA^-1 rA.y^2 + IB^-1 rB.y^2
    Kxy =     - IA^-1 rA.x rA.y - IB^-1 rB.x rB.y
    Kyy = m^-1 + IA^-1 rA.x^2 + IB^-1 rB.x^2

and the impulse is `K^-1 * (-Cdot - bias)`, solved with `Mat22::solve` (Cramer's
rule). `Joint::safeSolve` checks the determinant first and returns zero for a
singular block, which happens when both bodies are static or the constraint has
no effect.

## The two-contact block

Two contact points on the same pair of bodies are not independent: an impulse at
one changes the velocity at the other. The coupling term is

    K12 = mA^-1 + mB^-1
        + IA^-1 (rA1 x n)(rA2 x n)
        + IB^-1 (rB1 x n)(rB2 x n)

which is symmetric, giving the `Mat22 K{k11, k12, k12, k22}` block in
`ContactSolver::solveIteration`. Ignoring `K12` is what makes a scalar
point-by-point solve alternate between the two supports and rock a stack; solving
the block together removes that error.
