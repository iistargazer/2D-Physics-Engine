# Clipping an incident edge, and why resting boxes get two points

## Why one contact point is not enough

A box resting flat on the floor touches along an entire edge. A single point
contact can only produce a force along one line of action; the solver would apply
it at an arbitrary spot and the box would slowly rotate or rock. Two points at
the ends of the overlapping edge segment give the solver two independent lever
arms, which is exactly what it needs to cancel the support torque.

## Choosing the reference face

`collidePolygons` computes `findMaxSeparation` for both bodies. The body with the
larger (least negative) separation owns the **reference face** -- the face that
does the pushing. The other body supplies the **incident edge**: the edge whose
world normal is most anti-parallel to the reference normal,

    incFace = argmin over i of  dot(refN, inc.worldNormal(i))

If the two separations differ by less than `kFaceTolerance`, `A` wins. Ties must
break the same way every frame or the contact normal flips and impulses are lost.

## Clipping against the side planes

The reference edge from `v1` to `v2` defines a strip bounded by two planes with
normal `side = normalize(v2 - v1)`:

    dot(-side, p) <= -dot(side, v1)      dot(side, p) <= dot(side, v2)

`detail::clip` is one Sutherland-Hodgman pass against a single half-plane. For
the two endpoints with signed distances `d1` and `d2`:

* if `d <= 0` the endpoint is inside the half-plane and is kept;
* if `d1 * d2 < 0` the segment crosses the plane and the intersection is

      p = p1 + alpha * (p2 - p1),   alpha = d1 / (d1 - d2)

Two passes are applied, first against one side plane and then the other. If
either pass leaves fewer than two points, the poly-poly test reports no contact.

## Keeping the points that are actually penetrating

After clipping, each candidate point gets its depth against the reference face:

    depth = dot(refN, v1) - dot(refN, p)

Points with `depth >= 0` lie behind the reference plane and become contact
points, up to a maximum of two. `m.penetration` records the deepest of them for
diagnostics, but the solver uses each point's own `depth`. On a tilted box the
two depths genuinely differ; substituting the maximum for both would miscompute
the correction and the torque.

## Feature identity for warm starting

Each clipped point carries a `featureId` built from the reference owner, the
reference face index, the incident face index, and the endpoint or clipping side
that produced the point. Points that come from an original endpoint keep that
endpoint's identity as they slide; points created by an intersection keep the
identity of the side plane that created them.

This is deliberate. A hash of the world-space position would change every frame
as a box slides, so cached impulses would never match and warm starting would do
nothing. Encoding the *topology* keeps the identity stable while the geometry
moves, which is exactly what a persistent manifold needs.

## Normal direction

The manifold normal is defined as `A -> B`. When `B` owned the reference face the
code flips the sign of `refN` before storing it. Every consumer -- the solver,
the position pass, the CCD impact impulse -- relies on that single convention;
getting it wrong silently reverses the sign of every impulse.
