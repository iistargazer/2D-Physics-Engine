# Solver rows and the parallel-solve precondition

Status: the two preconditions for a parallel solve are **met and verified**; no
threads, jobs, queues or throughput claims exist yet, and none should be added
before the residual blockers below are closed.

Parallelism was blocked by two properties of the serial solver, not by missing
infrastructure:

1. **Row kernels wrote to immovable bodies.** `RigidBody::applyImpulse` had no
   guard, so a static body received `velocity += p * 0`. Numerically that is a
   no-op, but it is still a *write*: two rows sharing a floor raced on it, and a
   coloring that must treat the floor as a conflict needs one colour per row
   touching it. Every joint helper and the NGS projection had the same shape.
2. **Constraint rows had no stable identity or order.** The kernels iterated
   `manifolds_` and `joints` directly by pointer, so there was nothing a schedule
   could be computed against.

## Structure

| Module | Owns |
|---|---|
| [RigidBody](../RigidBody.hpp) | Body state, and the single *immovable-body policy*: `isStatic()`, plus guarded `applyImpulse` / `applyAngularImpulse` / `translate`. |
| [Manifold](../Manifold.hpp) | `ContactPoint` geometry cache and the accumulated **normal/tangent impulses**. The only contact-impulse owner. |
| [Joint](../Joint.hpp) | Joint state and the only joint impulse accumulators. Writes only through the guarded `RigidBody` helpers. |
| [Solver](../Solver.hpp) | PGS serial contact kernels. |
| [TemporalSolver](../TemporalSolver.hpp) | TGS serial anchor-row kernels. |
| [SolverRows](../SolverRows.hpp) | Row addressing: `SolverRow` identity, `RowSet`, canonical order, and `rowIsLive` - the one definition of "this block does work", shared by PGS, TGS and the row set. |
| [Schedule](../Schedule.hpp) | Deterministic colouring over body conflicts and its validation. Pure: it computes a schedule, it never executes one. |
| [World](../World.hpp) | Bodies, prepared capacity, contact cache, the per-tick row buffer, and tick orchestration. |

Data flow, unchanged in shape:

```
bodies -> broadphase -> narrowphase -> manifolds
       -> RowSet (indices only)  -> serial kernels (PGS or TGS) -> ContactPoint/Joint impulses
       -> integrator -> bodies
```

Impulses keep exactly one owner (`ContactPoint`, `Joint`); `RowSet` holds indices
and ordinals only, never pointers or impulses, so it stays valid across `bodies`
reallocation and cannot become a second source of truth.

## What is now guaranteed

- **Immovable bodies are bitwise untouched.** Verified for PGS and TGS, with NGS
  projection and sleeping on, over 200 ticks with five joint families anchored to
  the static floor, contact rows live throughout, and an allocation counter around
  every step ([test_rows.cpp](../test_rows.cpp)).
- **Rows have unique identity, and the serial order is already the canonical
  order.** `isCanonical(rows)` holds, and the dynamic-tree and all-pairs
  broadphases produce the *same* rows in the *same* order, because
  [Broadphase](../Broadphase.hpp) sorts candidate pairs and World sorts frozen
  contacts. Tree traversal order therefore cannot leak into a schedule.
- **The row list is exactly the solve input.** [test_rows.cpp](../test_rows.cpp)
  recomputes the expected rows independently from `manifolds()`/`joints` and
  compares them element-wise, so the addressing seam cannot drift from what the
  serial kernels iterate - the property a coloured replay has to preserve.
- **A valid deterministic colouring exists.** On 24 circles resting on one
  immovable floor: 24 rows, **1 colour** at full width, versus **24 colours** if
  the floor were treated as a conflict. That gap is the whole value of the guard.
  The colour count does not depend on contact enumeration order.

## Residual blockers

Named against the code, in the order they must be closed:

1. **No single row driver.** `World::tryStep` still runs `for (joint) solve; ...`
   then a separate contact kernel per iteration. A colored driver needs one
   dispatch that switches on `SolverRow::kind`.
2. **The TGS refresh is a second intra-tick write phase.** `refreshTemporal`
   rewrites manifold geometry and re-seeds warm impulses before each substep
   solve, so TGS has two write phases per slice that must each be colored (or
   fused per row) rather than one.
3. **`movableBodies()` allocates.** A driver needs a pre-reserved, stable
   movable view maintained with the capacity budget, not an on-demand vector.
4. **`rows_` is a mutable World member.** One tick owns it. A driver that can
   outlive the tick needs a per-tick epoch/token so a later tick cannot rebuild
   the row buffer under running work.
5. **Position correction is a second, geometry-changing pass.** `correctPositions`
   re-runs narrowphase per manifold and writes poses, so a schedule is valid only
   for one NGS iteration.
6. **CCD and the island pass stay serial.** `advanceContinuous` resolves
   time-of-impact events in order, and `Islands` are union-find passes that
   bracket the solve; neither belongs inside a parallel region.
7. **Warm-start cache ordering.** `restoreImpulses` scans `cache_` assuming
   `(indexA, indexB)` order and `saveImpulses` writes it for all manifolds, so both
   must complete before/after the parallel region - they already do, but a driver
   must not move them inside it.

## Reproduce

```bash
set -euo pipefail
g++ -std=c++20 -O2 -Wall -Wextra -Wpedantic -Werror -I. test_rows.cpp test_allocation_counter.cpp -o build/parallel/test_rows.exe
./build/parallel/test_rows.exe
```

Behavior neutrality for this restructuring was checked against receipts captured
before it: `test_engine`, `test_temporal`, `test_world_storage` and
`test_ccd_allocations` outputs are byte-identical, allocation counts stay at zero,
and no physical bound moved.
