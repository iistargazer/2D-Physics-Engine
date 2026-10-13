# Joint Jacobians

A joint is a constraint `C = 0` on the relative pose of two bodies. The velocity
constraint is `Cdot = J v`, where `v` collects both bodies' linear and angular
velocities and `J` is the Jacobian. Solving `J M^-1 J^T lambda = -Cdot` gives the
impulse that removes the constraint violation, which is the same `K = J M^-1 J^T`
identity used by contacts.

Everything below is implemented in `Joint::prepare` (assemble `K`, apply cached
impulses) and `Joint::solve` (per-sweep updates), with anchors stored in body-local
space as `localA` and `localB` and rotated out once per frame into `rA_` and `rB_`.

## Distance

Constraint, with `d = pB + rB - pA - rA` and rest length `L`:

    C = |d| - L

Its Jacobian, with `u = d / |d|`, is the row

    J = [ -u,  -rA x u,  u,  rB x u ]

so the effective inverse mass is the scalar

    K = mA^-1 + mB^-1 + IA^-1 (rA x u)^2 + IB^-1 (rB x u)^2

and the update is `lambda = -(Cdot + bias) / K`. A distance joint is bilateral:
it resists both stretching and compression, so unlike a contact there is no
clamping and `impulse` accumulates freely.

### Soft distance joints

Setting `frequency > 0` turns the joint into a spring-damper. If
`m_eff = 1 / K`, the spring and damper constants follow from the target angular
frequency `w = 2 pi f` and damping ratio `zeta`:

    k = m_eff * w^2
    c = 2 * zeta * m_eff * w

Implicit integration of that spring over a timestep yields the two softness
coefficients used in the code:

    gamma = 1 / (dt * (c + dt * k))
    beta  = dt * k * gamma

and the solve becomes

    delta = -(Cdot + beta * C + gamma * lambda) / (K + gamma)

storing `1 / (K + gamma)` in `scalarMass_`. A stiff spring (high `f`) approaches
the rigid case; lower frequencies visibly stretch under load, which is the point.

## Revolute (point to point)

Constraint, a two-vector:

    C = pB + rB - pA - rA

Differentiating gives one row per coordinate, and `K` becomes the symmetric 2x2
matrix

    Kxx = mA^-1 + mB^-1 + IA^-1 rA.y^2 + IB^-1 rB.y^2
    Kxy =            - IA^-1 rA.x rA.y - IB^-1 rB.x rB.y
    Kyy = mA^-1 + mB^-1 + IA^-1 rA.x^2 + IB^-1 rB.x^2

The impulse is `K^-1 (-Cdot - bias)` via `Mat22::solve`, guarded by
`Joint::safeSolve` against a singular block. The bias term `d * (0.2 / dt)` is
Baumgarte on the anchor error, which is what keeps a chain from slowly pulling
apart.

### Angle limits

Limits are unilateral, so they use the same treat-as-total clamping as contacts,
with separate accumulators for the lower and upper bound. With
`angle = B.angle - A.angle - referenceAngle` and gap `angle - lower`:

    lowBias = (lowGap < 0 ? 0.2 : 1) * lowGap / dt
    lowerImpulse = max(0, lowerImpulse - angularMass * (wRel + lowBias))

where `angularMass = 1 / (IA^-1 + IB^-1)`. The asymmetric bias is speculative: a
positive gap is converted straight into a separation velocity so a fast joint
cannot cross the limit within one step, while an actual violation is corrected at
the gentler 0.2 rate. The upper limit mirrors this with the sign of the relative
angular velocity reversed. Only `Revolute` uses limits in this engine.

### Motor

A motor drives the relative angular velocity toward `motorSpeed` and is strength
limited rather than exact:

    motorImpulse = clamp(old - angularMass * (wRel - motorSpeed),
                         -dt * maxMotorTorque, dt * maxMotorTorque)

The clamp is a torque budget expressed as an impulse budget, which is why a weak
motor cannot hold a heavy arm against gravity.

## Prismatic

A prismatic joint forbids relative motion along the perpendicular of a rail axis
attached to body `A`:

    C = d . n(A),     n = A.rot * localAxis,     perp = n.perp()

Because the normal rotates with `A`, differentiating produces an extra term: the
lever arm of `A` effectively becomes `rA + d`. Hence

    JwA = -(rA + d) x perp
    JwB =   rB      x perp

and

    K = mA^-1 + mB^-1 + IA^-1 JwA^2 + IB^-1 JwB^2

Omitting the `d` term is the classic bug: the constraint then behaves correctly
only while `A` does not rotate.

## Weld

`Weld` reuses the revolute point constraint and adds the angular constraint
`C = B.angle - A.angle - referenceAngle` with effective mass
`1 / (IA^-1 + IB^-1)`. The result is solved sequentially rather than as one
coupled 3x3 block, which is why the guide records it as a basic implementation
rather than a production weld.

## How joints enter the loop

`World::step` prepares every joint once, then alternates joint solves and contact
sweeps inside the shared iteration loop:

    for (iteration ...) { for (joint : joints) joint.solve(bodies); solver.solveIteration(manifolds_); }

Joints go first so contacts, which are the constraints that must never be
violated, get the final correction in each sweep. Joint impulses are rescaled by
the timestep ratio and reset whenever `World::validateCache` detects that the body
set changed underneath them.
