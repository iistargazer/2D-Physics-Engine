# The separating axis theorem, and why 2D needs so few axes

## The idea in one line

Two convex shapes are disjoint exactly when some line exists that puts them on
opposite sides. That line is a certificate: instead of testing every pair of
points, you test a small finite set of candidate directions.

## The theorem

Let `A` and `B` be convex subsets of the plane. Then `A ∩ B = ∅` if and only if
there is a unit vector `n` and a number `c` with

    n · a <= c <= n · b   for all a in A, b in B.

Equivalently, the projections of `A` and `B` onto `n` do not overlap. The set of
directions you must check is finite for polygons: the Minkowski difference
`A - B` is a convex polygon, and the supporting directions of its edges are the
face normals of `A` and of `B`. Three dimensions additionally needs edge-edge
cross-product axes; in 2D the cross product of two edge directions is a scalar,
so there are no extra axes to test.

## The face-based test used in code

`Collision.hpp` implements the face form rather than projecting intervals. For
each face `i` of `A` with outward unit normal `n_i` and vertex `v_i`:

    s_i = min over vertices b_j of B of  n_i · (b_j - v_i)

`s_i > 0` means every vertex of `B` lies in front of that face: a separating
axis, and `collidePolygons` returns false immediately. Otherwise the largest
`s_i` is the least negative overlap, and that face is the best candidate
reference face. `detail::findMaxSeparation` returns both the value and the face
index.

The same computation with the roles swapped gives `sepA` and `sepB`. The larger
value wins; `detail::kFaceTolerance` (1e-4) biases near-ties toward `A` so that
the chosen reference face does not flicker between frames.

Note the asymmetry: the engine compares separations and takes the maximum, not a
minimum overlap per axis. Maximizing the signed separation is the same
optimization stated on the distance scale, and it keeps the arithmetic free of
absolute values near zero.

## Circles and Voronoi regions

Circles have no faces, so the axis set changes. `collidePolygonCircle` puts the
circle center in the polygon's local frame and finds the face with the largest
signed separation `sep`. If `sep < kEps` the center is inside the polygon. If
the center projects outside the segment `v1..v2`, the nearest feature is a
vertex and the normal is radial:

    n = normalize(center - v)

Otherwise the nearest feature is the face plane and the normal is the face
normal. Two circles reduce to the single axis between centers:

    n = (B.position - A.position) / |B.position - A.position|

## Conventions the rest of the engine depends on

The manifold normal always points from `A` to `B`. `collidePolygonCircle` is
written for a polygon first, so the dispatch in `collide` negates the normal
when the circle is body `A`. Everything downstream -- impulse signs, friction
tangent `t = (n.y, -n.x)`, effective mass -- assumes this.

## What this does not do

SAT here is convex-only and generates one normal per pair. Concave shapes are
not supported and are not silently approximated. The candidates come from
`Broadphase`; SAT is only the exact test applied to pairs that already share
loose bounds.
