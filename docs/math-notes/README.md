# Math notes

Short derivations, one topic per file, written to be readable on their own. Each
note states the intuition, derives the result, and then names the function or
struct in this repository that implements it.

| Note | Topic |
| --- | --- |
| [sat.md](sat.md) | Separating axis theorem for 2D convex polygons and circles, and why 2D needs only face normals |
| [clipping.md](clipping.md) | Reference and incident face selection, Sutherland-Hodgman clipping, and why face contacts produce two points |
| [effective-mass.md](effective-mass.md) | `K = J M^-1 J^T`: the scalar contact form, the 2x2 revolute block, and the two-contact coupling term |
| [accumulated-clamping.md](accumulated-clamping.md) | Why the running total impulse is clamped rather than each increment, and what that buys convergence |
| [friction-cone.md](friction-cone.md) | The Coulomb cone, the tangent basis, solve ordering, and material mixing |
| [warm-starting.md](warm-starting.md) | Persistent manifolds, stable feature IDs, impulse seeding, restitution ordering, and cache invalidation |
| [baumgarte-vs-ngs.md](baumgarte-vs-ngs.md) | Velocity bias versus nonlinear Gauss-Seidel position correction, with the measured energy and depth comparison |
| [joint-jacobians.md](joint-jacobians.md) | Jacobians and effective masses for distance, soft distance, revolute with limits and motor, prismatic, and weld |
| [aabb-tree-sah.md](aabb-tree-sah.md) | Fat AABB proxies, the perimeter cost heuristic, balancing rotations, and honest complexity claims |

Two longer overview notes cover the same ground in narrative form and are kept
for continuity:

* [collision.md](collision.md) - collision geometry and the conservative
  advancement prototype.
* [constraints.md](constraints.md) - constraints, warm starting, and the
  two-point block LCP.

Related engineering context lives in [../../PHYSICS_ENGINE_GUIDE.md](../../PHYSICS_ENGINE_GUIDE.md)
(status, conventions, measured results, and known limitations) and
[../../README.md](../../README.md) (build instructions and benchmark tables).

Whenever a note quotes a performance or energy figure, the value was measured on
the machine that produced `build/stability.csv` and `build/integrator_energy.csv`;
those raw files are the source of truth if a number ever disagrees.
