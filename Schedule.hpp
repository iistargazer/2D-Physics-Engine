#pragma once
// ============================================================================
// Deterministic row schedule - the mechanical next step for a parallel solver.
//
// Two rows conflict when they WRITE the same movable body. An immovable body is
// only ever read by a solver row (RigidBody guards every write), so a floor body
// touched by thousands of rows is FREE: it contributes no conflict and therefore
// no colour. Naively treating it as a conflict is what would force as many
// colours as there are contacts; that is precisely why eliminating static writes
// had to land before this file could be useful.
//
// Everything here is pure, deterministic and thread-free: it computes and
// validates a schedule, it does not execute one.
// ============================================================================
#include "SolverRows.hpp"
#include <cstdint>
#include <vector>

namespace phys {

struct ColorPlan {
    std::vector<std::uint32_t> order;         // canonical row order that was coloured
    std::vector<std::uint32_t> color_of_row;  // aligned with the input row list
    std::uint32_t colors = 0;                 // number of colour classes used
    std::uint32_t largestClass = 0;           // widest class, i.e. parallel width
};

// Greedy colouring in canonical order: deterministic, and never worse than
// (max body degree + 1) colours. `movable[i]` must be true exactly for bodies the
// solver may write.
inline ColorPlan colorRows(const std::vector<SolverRow>& rows,
                           const std::vector<char>& movable) {
    ColorPlan plan;
    plan.order = canonicalOrder(rows);
    plan.color_of_row.assign(rows.size(), 0);
    // Colours already spent on each movable body, in assignment order. A body's
    // degree bounds this list, so it stays tiny even for large scenes.
    std::vector<std::vector<std::uint32_t>> used(movable.size());
    std::vector<std::uint32_t> classSize;
    const auto movableAt = [&movable](std::uint32_t i) {
        return i < movable.size() && movable[i];
    };
    for (const std::uint32_t rowIndex : plan.order) {
        const SolverRow& row = rows[rowIndex];
        const bool aMovable = movableAt(row.bodyA), bMovable = movableAt(row.bodyB);
        // Mark every colour either body already spent, then take the lowest free.
        std::vector<char> taken(plan.colors + 1, 0);
        const auto mark = [&](std::uint32_t body) {
            if (!movableAt(body)) return;
            for (const std::uint32_t colour : used[body])
                if (colour < taken.size()) taken[colour] = 1;
        };
        mark(row.bodyA); mark(row.bodyB);
        std::uint32_t colour = 0;
        while (colour < taken.size() && taken[colour]) ++colour;
        plan.color_of_row[rowIndex] = colour;
        if (colour == plan.colors) ++plan.colors;
        if (colour >= classSize.size()) classSize.resize(colour + 1, 0);
        plan.largestClass = std::max(plan.largestClass, ++classSize[colour]);
        // An immovable body is read-only, so it records nothing.
        if (aMovable) used[row.bodyA].push_back(colour);
        if (bMovable) used[row.bodyB].push_back(colour);
    }
    return plan;
}

// A colour class is only schedulable if no two of its rows write the same
// movable body. This is the property the parallel driver will depend on.
inline bool validateColoring(const std::vector<SolverRow>& rows,
                             const std::vector<char>& movable,
                             const ColorPlan& plan) {
    if (plan.color_of_row.size() != rows.size()) return false;
    const auto movableAt = [&movable](std::uint32_t i) {
        return i < movable.size() && movable[i];
    };
    const auto conflicts = [&](const SolverRow& a, const SolverRow& b) {
        return (a.bodyA == b.bodyA && movableAt(a.bodyA))
            || (a.bodyA == b.bodyB && movableAt(a.bodyA))
            || (a.bodyB == b.bodyA && movableAt(a.bodyB))
            || (a.bodyB == b.bodyB && movableAt(a.bodyB));
    };
    for (std::size_t i = 0; i < rows.size(); ++i)
        for (std::size_t j = i + 1; j < rows.size(); ++j)
            if (plan.color_of_row[i] == plan.color_of_row[j] && conflicts(rows[i], rows[j]))
                return false;
    for (const std::uint32_t colour : plan.color_of_row)
        if (colour >= plan.colors) return false;
    return true;
}

} // namespace phys
