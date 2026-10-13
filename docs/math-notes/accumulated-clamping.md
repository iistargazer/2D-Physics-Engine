# Clamp the total, not the increment

## The constraint

A contact can push but never pull. Written as a complementarity condition on the
accumulated normal impulse `lambda` and the post-solve normal speed `vn'`:

    lambda >= 0,      vn' >= 0,      lambda * vn' = 0

Sequential impulses (Gauss-Seidel) visits one constraint at a time and asks: what
additional impulse would make this constraint satisfied, given every other
constraint's current state? The answer is an increment `delta`, and the correct
update is

    new = max(old + delta, 0)
    applied = new - old
    old = new

The thing being clamped is `old + delta`, the running total.

## Why clamping the increment is wrong

Take a one-dimensional chain of constraint updates where the first sweep
overshoots. Suppose a contact has accumulated 5 units and a later sweep computes
`delta = -3`, because another contact changed the velocity field underneath it.

Clamping the increment to `max(delta, 0)` discards the `-3` entirely. The
accumulated impulse stays at 5 and the overestimate is now permanent. The total
clamp instead computes `new = max(5 - 3, 0) = 2` and applies `-3`, which is
exactly the correction the system asked for.

A negative *applied* impulse is therefore a normal, necessary event. It is not a
bug and it does not mean the contact is pulling: the applied value is the
difference between two non-negative totals, and the total remains `>= 0`.

## Convergence view

Sequential impulses is Gauss-Seidel on a linear complementarity problem. Its
convergence guarantee comes from projecting the iterate back onto the feasible
set `lambda >= 0` after each update, which is precisely what the total clamp
does. Per-increment clamping changes the iteration into something that is not the
projected Gauss-Seidel map and has no such guarantee; the standard failure mode
is a stack that slowly sticks because every early overestimate is frozen in.

## In the code

`ContactSolver::solveIteration` applies this to both directions. Friction first:

    const real maxF = m.friction * c.accN;
    const real newAcc = clamp(c.accT + lambda, -maxF, maxF);
    lambda = newAcc - c.accT;
    c.accT = newAcc;

then the normal direction:

    const real newAcc = std::max(c.accN + lambda, real(0));
    lambda = newAcc - c.accN;
    c.accN = newAcc;

The friction bound uses the *current* `accN`, so the cone widens and narrows as
the normal impulse converges. That coupling is why friction is solved before the
normal in each sweep: the normal update gets the last word on non-penetration.

## Interaction with warm starting

Warm starting is only possible because the total is what is stored. `c.accN` and
`c.accT` persist between frames, are re-applied in `prepare`, and then the
sweeps adjust them as totals. If only increments were kept, there would be no
quantity to carry across frames.

`prepare` also clamps the recovered tangential value back into the current
friction cone, since the contact normal and the normal impulse may both have
changed since the values were cached.
