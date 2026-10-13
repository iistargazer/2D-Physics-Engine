# Dynamic AABB trees: spend geometry only where it matters

An exact collision test is valuable only when two shapes could meet. With N bodies, testing every unordered pair costs N(N−1)/2 narrowphase calls—even if every body is far apart. Broadphase removes that wasted work without changing contact physics.

## A conservative certificate

For each world-space polygon vertex pᵢ, take componentwise minima and maxima. For a circle with center c and radius r, bounds are c ± (r,r). A shape S is contained in its AABB A. Therefore

    A ∩ B = ∅  ⇒  S_A ∩ S_B = ∅.

The converse is false: rotated rectangles can have overlapping boxes while their shapes are separated. Treat broadphase output as *candidates*. Use inclusive interval overlap so touching bounds cannot discard a contact that narrowphase accepts.

## Trade small spatial slack for temporal coherence

Expand tight bounds by margin m:

    A_fat = [a_min − (m,m), a_max + (m,m)].

As long as a moved shape's tight bounds remain inside the old fat bounds, the leaf can stay where it is. A larger margin reduces updates but makes more tree branches overlap queries. We use m = 0.1 world units and recheck actual tight bounds before emitting a pair. Rotations require new tight bounds just as translations do.

This is not a swept volume or CCD. A fast object can still cross a thin wall between discrete steps.

## The hierarchy and a 2D cost model

Every leaf holds a fat bound; every parent holds the exact union of its children. Query overlap failure at a parent rejects all leaves below it.

Use boundary length as a surface-area surrogate in two dimensions:

    P(A) = 2(width + height),
    ΔP = P(A ∪ L) − P(A),

where L is the incoming leaf. This is a heuristic for how expensive an enlarged branch will be to query, not an exact probability law. It depends on scene/query distributions.

At an internal node, enlarging its bounds contributes inherited cost 2ΔP. For each child C, compare:

    cost(C) = inherited + P(C ∪ L)                 if C is a leaf,
    cost(C) = inherited + P(C ∪ L) − P(C)          otherwise.

Descend toward the cheaper child, breaking ties deterministically toward the left. Our implementation deliberately descends all the way to a leaf rather than allowing a new parent above a tall subtree. This makes insertion an AVL-compatible height change, allowing one local balancing operation per ancestor. It is a practical greedy SAH variant, not a globally optimal spatial partition.

## Rotations preserve geometry

If a node's right subtree is more than one level taller, promote that right child. Attach its shorter grandchild beneath the former root; mirror this for a taller left subtree. No leaf is lost or duplicated. Recompute heights and bound unions from the affected children. Our invariant checks verify parents, exact unions, heights, reachability, and local height balance after insertion, movement, and removal.

A balanced height does not guarantee good spatial bounds. Both height and overlap cost matter.

## Correctness before speed

Tree queries use tight bounds against conservative node bounds. At leaves, recheck tight-vs-tight overlap. Emit only i < j, omit static/static pairs, and sort lexicographically. The original solver saw pairs in that same order, so changing traversal order cannot change Gauss–Seidel ordering.

Tests compare both candidate sets and accepted narrowphase pair sets against all-pairs oracles over fixed-seed moving scenes. A full-world comparison confirms bit-identical trajectories for 600 steps.

## What complexity can we honestly claim?

Balanced insertion/removal has O(log N) path work. This implementation computes every body's bounds and queries every body each frame; query cost depends on spatial overlap. Sorting P output pairs costs O(P log P). If all shapes overlap, P itself is Θ(N²)—no spatial index can output that set in linear time.

The measurements in the README show where the tree pays off and where small-N brute force is cheaper. Initial construction, long steady-state reinsertion workloads, sleeping stacks, mixed-size scenes, and solver-dominated stress tests deserve separate profiling before further optimization.
