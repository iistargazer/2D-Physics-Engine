> Overview note. Topic-by-topic versions live in [sat.md](sat.md), [clipping.md](clipping.md), and [aabb-tree-sah.md](aabb-tree-sah.md); the full index is [README.md](README.md).

# Collision geometry: proofs before code

## SAT

Two disjoint convex sets admit a separating line. For two polygons in 2D, the separating axes can be chosen from face normals: these are the boundary directions of their Minkowski difference. Project all vertices onto each normal. Any disjoint projection rejects contact.

The face-based implementation computes sᵢ=minⱼ nᵢ·(bⱼ−aᵢ). Positive s means a gap. Select the maximum signed separation (least negative penetration), checking faces of both polygons. Near ties favor A's face to reduce reference-feature flicker.

Circles introduce Voronoi regions: the nearest polygon feature is either a face interior or a vertex. A vertex requires a radial circle/vertex axis, not just polygon normals.

## Clipping

The selected reference face bounds a strip between two side planes. Choose the incident edge most anti-parallel to the reference normal, then clip it against those planes. For signed distances d1,d2, a crossing point uses α=d1/(d1−d2) and p=p1+α(p2−p1). Keep clipped points behind the reference face. Their individual depths matter: a tilted box's two depths differ.

Two face-contact points represent separated supports and therefore different torque arms. One point cannot reproduce the same support distribution. Feature identity follows original endpoints or the side plane that created an intersection.

## Conservative advancement

A separating projection gap d is no larger than true shape distance. Every point on a rigid shape moves at no more than |v|+|ω|r_max. Consequently a relative closing-speed upper bound is

    V = |vB−vA| + |ωA|rA + |ωB|rB.

Advancing by 0.9 d/V remains before contact under constant linear/angular velocities. Recompute axes at the advanced pose. This is conservative but can converge slowly near grazing or rotating contact.

The current engine uses this as an **opt-in CCD prototype**. Swept normal and Coulomb-clamped tangential impulses use the centre of mass (`r=0`), avoiding fabricated spin but omitting correct off-center torque. Per-bullet motion clamps and positional push-outs are reported separately from event caps; unrelated bodies continue, with no global freeze. Specific circle/tilted-polygon wall tests pass with idle nets, while a separate spinning plate retains its spin. Joint re-solving, connected-body filtering during casts, swept-tree acceleration and general oblique/moving impact acceptance remain unfinished. See [the current status ledger](../../PHYSICS_ENGINE_GUIDE.md#5-current-phase-status); do not describe this as completed general CCD.
