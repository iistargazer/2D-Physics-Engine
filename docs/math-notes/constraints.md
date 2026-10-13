> Overview note. Topic-by-topic versions live in [effective-mass.md](effective-mass.md), [accumulated-clamping.md](accumulated-clamping.md), [friction-cone.md](friction-cone.md), [warm-starting.md](warm-starting.md), [baumgarte-vs-ngs.md](baumgarte-vs-ngs.md), and [joint-jacobians.md](joint-jacobians.md); the full index is [README.md](README.md).

# Constraints: from velocity errors to impulses

## Effective mass

An impulse P=λn at r changes v by m⁻¹P and angular velocity by I⁻¹(r×P). Substitute both changes into the relative contact velocity. Its normal component changes by Kλ, where

    K = mA⁻¹ + mB⁻¹ + IA⁻¹(rA×n)² + IB⁻¹(rB×n)².

Thus the impulse targeting normal speed v* is λ=(v*−vn)/K. Static bodies contribute zero inverse mass/inertia. This is the scalar form of K=JM⁻¹Jᵀ.

## Accumulated clamping

Normal impulses may push but not pull. Constrain the **total**, not each increment:

    new = max(old + delta, 0); applied = new − old.

A negative applied increment is valid: it removes an earlier overestimate. Clamping every increment would make overestimates irreversible. Sequential impulses are Gauss–Seidel updates for the contact complementarity problem.

## Friction cone

For t=(ny,−nx), repeat the effective-mass derivation along t. Coulomb friction restricts accumulated impulse to |λt|≤μλn. The friction coefficient is the geometric material mean; its bound changes when the normal impulse changes. This engine solves friction before normal so the final nonpenetration update wins.

## Warm starting

Previous impulses approximate the new frame's solution. Match body pairs and geometric contact features, then seed velocities using P0=λn n+λt t. Compute **every** restitution target before seeding, otherwise the seed changes the measured incoming impact velocity. A normal change or invalid body identity invalidates the cache. Contact disappearance must expire cached impulses.

Impulses approximate force times dt, so a timestep ratio scales cached values. Our actual simulation uses fixed dt. Clipped point IDs record reference/incident features and clipping sides; world-point hashes would flicker as a box slides.

## Two-point block LCP

Two contact points share body velocities. Their effective mass is a symmetric 2×2 matrix with off-diagonal

    K12 = mA⁻¹+mB⁻¹+IA⁻¹(rA1×n)(rA2×n)+IB⁻¹(rB1×n)(rB2×n).

Given old impulse a, form b=vn−target−Ka. Solve w=Kx+b≥0, x≥0, xᵢwᵢ=0. Enumerate four active sets: both active, first only, second only, neither. Apply x−a. Near-singular blocks fall back to scalar solving. This removes the alternating support torque that destabilized our tall column.

## Baumgarte versus NGS

Baumgarte targets separation velocity β(depth−slop)/dt; it mixes geometric repair into physical kinetic velocity. NGS instead recomputes geometry and applies position-only λp=βp max(depth−slop,0)/K. It updates both translation and rotation without changing velocity.

Measured overlap experiment: Baumgarte added 2.5992 J; NGS added zero kinetic energy. NGS is not perfect energy conservation: moving positions changes potential energy, contacts can change, and finite iterations leave residual error. Corrections are capped and iterated. Joint position projection follows the same principle; soft springs deliberately are not projected to rigid rest length.

## Joint Jacobians

Distance: C=|pB+rB−pA−rA|−L. Its Jacobian is [-u, −rA×u, u, rB×u]; it yields the scalar K above. Bilateral impulses need no normal-style clamping.

Revolute: C=pB+rB−pA−rA is a 2-vector. Its K has diagonals m⁻¹+ΣI⁻¹ry² and m⁻¹+ΣI⁻¹rx², off-diagonal −ΣI⁻¹rx ry. Solve Kλ=−Cdot−bias. Motors cap total angular impulse by dt·maximum torque. Angle limits are unilateral constraints with separate lower/upper accumulated impulses.

For a soft distance spring, k=m_eff ω² and c=2ζm_eff ω. Implicit integration gives γ=1/[dt(c+dt k)] and β=dt kγ; solve deltaλ=−(Cdot+βC+γλ)/(K+γ). The mouse joint uses the corresponding 2D spring matrix and a force-times-dt impulse cap.

For a prismatic joint, the normal rotates with body A. Differentiating C=d·n(A) adds d to A's angular lever arm: JwA=−(rA+d)×n. Omitting this term produces the wrong constraint for a rotating rail.
