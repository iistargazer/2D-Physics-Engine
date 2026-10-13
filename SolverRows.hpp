#pragma once
// ============================================================================
// Solver rows - the addressing seam between the serial reference and any future
// parallel driver (see docs/PARALLEL.md).
//
// A row is one solver BLOCK, not one contact point: a manifold solves its 1-2
// points together (the 2-point block is a joint 2x2 LCP), and a joint solves as
// one row. Splitting a manifold into per-point rows would change the solve.
//
// Identity is body INDICES plus an ordinal, never pointers, so a schedule
// computed from a row list stays valid for the whole tick and is reproducible
// across runs and across `bodies` reallocation.
//
// This header owns no body state and no impulses: RigidBody, ContactPoint and
// Joint remain the only owners. It is pure addressing.
// ============================================================================
#include "Manifold.hpp"
#include "Joint.hpp"
#include <algorithm>
#include <cstdint>
#include <vector>

namespace phys {

struct SolverRow {
    enum class Kind : std::uint8_t { Joint = 0, Contact = 1 };

    Kind kind = Kind::Contact;
    std::uint32_t bodyA = 0, bodyB = 0;   // indices into World::bodies
    std::uint32_t ordinal = 0;            // joint index, or manifold index
};

// Total order over row identity. Deliberately independent of enumeration order:
// this is what makes a parallel schedule reproducible. The serial solve order
// already equals it because Broadphase sorts candidate pairs and World sorts
// frozen contacts, so `isCanonical` holds for the shipped pipeline.
inline bool rowLess(const SolverRow& a, const SolverRow& b) {
    if (a.kind != b.kind) return a.kind < b.kind;      // joints before contacts
    if (a.bodyA != b.bodyA) return a.bodyA < b.bodyA;
    if (a.bodyB != b.bodyB) return a.bodyB < b.bodyB;
    return a.ordinal < b.ordinal;
}

inline bool rowIdentityEqual(const SolverRow& a, const SolverRow& b) {
    return a.kind == b.kind && a.bodyA == b.bodyA && a.bodyB == b.bodyB
        && a.ordinal == b.ordinal;
}

// A block whose two bodies cannot move is skipped by every serial kernel, so it
// is not a row: the list is exactly the set of blocks that will write.
inline bool rowIsLive(const RigidBody& A, const RigidBody& B) {
    return !((A.isStatic() || !A.awake) && (B.isStatic() || !B.awake));
}

// Canonical, index-stable list of every constraint block in one tick.
class RowSet {
public:
    void reserve(std::size_t n) { rows_.reserve(n); }
    void clear() { rows_.clear(); }

    // Serial solve order: joints in index order, then manifolds in their current
    // order. That is exactly the order the serial kernels iterate.
    void build(const std::vector<ContactManifold>& contacts,
               const std::vector<Joint>& joints,
               const std::vector<RigidBody>& bodies) {
        rows_.clear();
        for (std::uint32_t j = 0; j < joints.size(); ++j) {
            const Joint& joint = joints[j];
            if (joint.a >= bodies.size() || joint.b >= bodies.size()) continue;
            if (!rowIsLive(bodies[joint.a], bodies[joint.b])) continue;
            SolverRow row;
            row.kind = SolverRow::Kind::Joint;
            row.bodyA = static_cast<std::uint32_t>(joint.a);
            row.bodyB = static_cast<std::uint32_t>(joint.b);
            row.ordinal = j;
            rows_.push_back(row);
        }
        for (std::uint32_t m = 0; m < contacts.size(); ++m) {
            const ContactManifold& manifold = contacts[m];
            if (manifold.bodyA == nullptr || manifold.bodyB == nullptr) continue;
            if (manifold.count <= 0) continue;
            if (!rowIsLive(*manifold.bodyA, *manifold.bodyB)) continue;
            SolverRow row;
            row.kind = SolverRow::Kind::Contact;
            row.bodyA = static_cast<std::uint32_t>(manifold.indexA);
            row.bodyB = static_cast<std::uint32_t>(manifold.indexB);
            row.ordinal = m;
            rows_.push_back(row);
        }
    }

    const std::vector<SolverRow>& rows() const { return rows_; }
    std::size_t size() const { return rows_.size(); }

private:
    std::vector<SolverRow> rows_;
};

// Row indices sorted by identity: the order a parallel driver must colour in so
// that the schedule does not depend on enumeration order.
inline std::vector<std::uint32_t> canonicalOrder(const std::vector<SolverRow>& rows) {
    std::vector<std::uint32_t> order(rows.size());
    for (std::uint32_t i = 0; i < order.size(); ++i) order[i] = i;
    std::sort(order.begin(), order.end(),
        [&rows](std::uint32_t a, std::uint32_t b) { return rowLess(rows[a], rows[b]); });
    return order;
}

// True when the row list is already in canonical order (the shipped invariant).
inline bool isCanonical(const std::vector<SolverRow>& rows) {
    for (std::size_t i = 1; i < rows.size(); ++i)
        if (rowLess(rows[i], rows[i - 1])) return false;
    return true;
}

// Every (kind, bodyA, bodyB, ordinal) must appear once: a duplicate would mean a
// block is solved twice or two blocks share an identity the schedule relies on.
inline bool identityUnique(const std::vector<SolverRow>& rows) {
    std::vector<SolverRow> sorted = rows;
    std::sort(sorted.begin(), sorted.end(), rowLess);
    for (std::size_t i = 1; i < sorted.size(); ++i)
        if (rowIdentityEqual(sorted[i], sorted[i - 1])) return false;
    return true;
}

} // namespace phys
