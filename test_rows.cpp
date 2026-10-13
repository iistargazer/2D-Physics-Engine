// Serial-reference harness for the parallel-solve track (docs/PARALLEL.md).
//
// It proves the two things the parallel work depends on before any thread exists:
//   1. every solver row write is inert on an immovable body, bit for bit, so a
//      shared floor costs no colour and no race;
//   2. rows have unique, index-stable identity in a deterministic order, and a
//      valid deterministic colouring exists over body conflicts.
// It also keeps the allocation contract honest through the new row pass.
#include "Schedule.hpp"
#include "Scenes.hpp"
#include <cstdio>
#include <cstring>
#include <vector>

extern std::size_t testAllocations;
extern bool countTestAllocations;
using namespace phys;

static int failures = 0;
#define CHECK(condition, message) \
    do { if (!(condition)) { std::fprintf(stderr, "FAIL: %s\n", message); ++failures; } } while (false)

static bool sameBits(real a, real b) { return std::memcmp(&a, &b, sizeof(real)) == 0; }

// Bitwise, not `==`: an immovable body must not be touched at all, so turning a
// -0.0 into 0.0 would also count as a change.
static bool untouched(const RigidBody& a, const RigidBody& b) {
    return sameBits(a.position.x, b.position.x) && sameBits(a.position.y, b.position.y)
        && sameBits(a.velocity.x, b.velocity.x) && sameBits(a.velocity.y, b.velocity.y)
        && sameBits(a.angle, b.angle) && sameBits(a.angularVelocity, b.angularVelocity)
        && sameBits(a.rot.m00, b.rot.m00) && sameBits(a.rot.m01, b.rot.m01)
        && sameBits(a.rot.m10, b.rot.m10) && sameBits(a.rot.m11, b.rot.m11)
        && sameBits(a.force.x, b.force.x) && sameBits(a.force.y, b.force.y)
        && sameBits(a.torque, b.torque) && sameBits(a.driveTorque, b.driveTorque)
        && sameBits(a.sleepTime, b.sleepTime)
        && a.awake == b.awake && a.invMass == b.invMass && a.invInertia == b.invInertia;
}

static bool measuredStep(World& w, real dt) {
    testAllocations = 0; countTestAllocations = true;
    const auto status = w.tryStep(dt);
    countTestAllocations = false;
    return status == World::StepStatus::Complete && testAllocations == 0;
}

// ---- 1. immovable bodies are never written ---------------------------------
// Every joint family that can anchor to a static body, in both solver modes,
// with NGS projection and sleeping on. 200 ticks, checking many times a frame.
static void immovableUntouched(SolverMode mode, bool positionCorrection, bool sleeping, const char* label) {
    // The stack keeps real contact rows against the immovable floor; the rig is
    // held clear of the floor by joints only, so the joint and contact kernels
    // are exercised at the same time without one disturbing the other.
    World w = makeStack(4);
    for (int i = 0; i < 4; ++i) w.add(RigidBody::makeCircle(0.3, 1, {-10.0 - 3.0 * i, 2.5}));
    w.solver.config().mode = mode;
    w.solver.config().substeps = 4;
    w.solver.config().positionCorrection = positionCorrection;
    w.sleeping = sleeping;
    Joint revolute = Joint::revolute(0, 5, {-10, 2.5}, {0, 0});
    revolute.motor = true; revolute.motorSpeed = 2; revolute.maxMotorTorque = 10;
    revolute.limits = true; revolute.lower = -1; revolute.upper = 1;
    w.joints.push_back(revolute);
    w.joints.push_back(Joint::weld(0, 6, {-13, 2.5}, {0, 0}));
    Joint slide = Joint::prismatic(0, 7, {-16, 2.5}, {}, {1, 0});
    slide.motor = true; slide.motorSpeed = 3; slide.maxMotorTorque = 50;
    w.joints.push_back(slide);
    w.joints.push_back(Joint::distance(0, 8, {-19, 2.5}, {}, 0.5, 3, 0.7));
    w.joints.push_back(Joint::mouse(5, {}, {2, 2}));
    w.prepare({12, 6, 40, 40, 32});

    std::vector<RigidBody> before = w.bodies;
    bool untouchedOk = true, allocationOk = true, bodiesKnown = true;
    for (int f = 0; f < 200; ++f) {
        // A persistent nudge on the top box keeps the contact rows live, so the
        // sleeping variant is exercised with rows actually being solved.
        w.bodies[4].applyForce({0.1, 0}, w.bodies[4].position);
        allocationOk = measuredStep(w, 1.0 / 60) && allocationOk;
        for (std::size_t i = 0; i < w.bodies.size(); ++i)
            if (w.bodies[i].isStatic()) untouchedOk = untouched(before[i], w.bodies[i]) && untouchedOk;
        for (const auto& row : w.rows().rows())
            bodiesKnown = row.bodyA < w.bodies.size() && row.bodyB < w.bodies.size() && bodiesKnown;
    }
    CHECK(allocationOk, "prepared step with rows and joints allocates nothing");
    CHECK(untouchedOk, label);
    CHECK(bodiesKnown, label);
    // Coverage of both kernels: the stack keeps floor contacts, and at least the
    // motor/mouse joints on the awake rig keep a joint row live.
    std::size_t contactRows = 0, jointRows = 0;
    for (const auto& row : w.rows().rows()) {
        if (row.kind == SolverRow::Kind::Contact) ++contactRows; else ++jointRows;
    }
    CHECK(contactRows >= 4, label);
    CHECK(jointRows >= 1, label);
    std::printf("IMMOVABLE %-24s static_bodies=%d ticks=200 rows=%zu contact_rows=%zu joint_rows=%zu\n",
                label,
                int(std::count_if(w.bodies.begin(), w.bodies.end(),
                                  [](const RigidBody& b) { return b.isStatic(); })),
                w.rows().size(), contactRows, jointRows);
}

// ---- 2. row identity is unique, stable and deterministically ordered -------
static bool sameRowIdentities(const std::vector<SolverRow>& a, const std::vector<SolverRow>& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i)
        if (!rowIdentityEqual(a[i], b[i])) return false;
    return true;
}

static void rowIdentity() {
    const auto build = [](BroadphaseMode mode) {
        World w = makeStack(6);
        w.broadphaseMode = mode;
        w.joints.push_back(Joint::revolute(0, 3, {0, 3}, {0, 0}));
        w.prepare({8, 2, 60, 60, 40});
        for (int f = 0; f < 60; ++f) measuredStep(w, 1.0 / 60);
        return w;
    };
    World tree = build(BroadphaseMode::DynamicTree);
    World allPairs = build(BroadphaseMode::AllPairs);
    const auto& rows = tree.rows().rows();

    CHECK(!rows.empty(), "a loaded stack produces rows");
    CHECK(identityUnique(rows), "row identities are unique");
    CHECK(isCanonical(rows), "serial row order is already the canonical order");
    CHECK(rows.size() == tree.joints.size()
              + std::count_if(tree.manifolds().begin(), tree.manifolds().end(),
                              [](const ContactManifold& m) { return m.count > 0; }),
          "one row per joint plus one per live manifold");
    // Tree traversal order must not leak into the schedule.
    CHECK(sameRowIdentities(rows, allPairs.rows().rows()),
          "dynamic-tree and all-pairs enumeration produce identical rows in identical order");

    // Independent oracle: the row list must equal the solve input in solve order,
    // recomputed here from the world's own vectors instead of from RowSet::build,
    // so the addressing seam cannot silently drift from what the kernels solve.
    std::vector<SolverRow> expected;
    const auto live = [](const RigidBody& A, const RigidBody& B) {
        return !((A.isStatic() || !A.awake) && (B.isStatic() || !B.awake));
    };
    for (std::uint32_t j = 0; j < tree.joints.size(); ++j) {
        const Joint& joint = tree.joints[j];
        if (!live(tree.bodies[joint.a], tree.bodies[joint.b])) continue;
        SolverRow row;
        row.kind = SolverRow::Kind::Joint;
        row.bodyA = static_cast<std::uint32_t>(joint.a);
        row.bodyB = static_cast<std::uint32_t>(joint.b);
        row.ordinal = j;
        expected.push_back(row);
    }
    for (std::uint32_t m = 0; m < tree.manifolds().size(); ++m) {
        const ContactManifold& manifold = tree.manifolds()[m];
        if (manifold.count <= 0) continue;
        if (!live(*manifold.bodyA, *manifold.bodyB)) continue;
        SolverRow row;
        row.kind = SolverRow::Kind::Contact;
        row.bodyA = static_cast<std::uint32_t>(manifold.indexA);
        row.bodyB = static_cast<std::uint32_t>(manifold.indexB);
        row.ordinal = m;
        expected.push_back(row);
    }
    CHECK(sameRowIdentities(rows, expected), "row list is exactly the solve input in solve order");

    // An immovable body is not a member of any schedule, only a reader.
    const auto movable = tree.movableBodies();
    CHECK(movable.size() == tree.bodies.size() && movable[0] == 0 && movable[3] == 1,
          "movable flags name exactly the non-static bodies");
    bool rowsSkipImmovablePairs = true;
    for (const auto& row : rows)
        if (!movable[row.bodyA] && !movable[row.bodyB]) rowsSkipImmovablePairs = false;
    CHECK(rowsSkipImmovablePairs, "no row has two immovable bodies");
}

// ---- 3. a valid deterministic colouring exists -----------------------------
static void coloring() {
    auto scene = [] {
        World w;
        w.add(RigidBody::makeBox(200, 1, 0, {0, -0.5}));
        for (int i = 0; i < 24; ++i) {
            auto circle = RigidBody::makeCircle(0.5, 1, {-45.0 + 3.0 * i, 0.49});
            circle.restitution = 0;
            w.add(circle);
        }
        w.prepare({32, 0, 200, 200, 128});
        for (int f = 0; f < 30; ++f) measuredStep(w, 1.0 / 60);
        return w;
    };
    World w = scene();
    const auto& rows = w.rows().rows();
    const auto movable = w.movableBodies();
    CHECK(rows.size() == 24, "one contact row per circle resting on the floor");

    const ColorPlan guarded = colorRows(rows, movable);
    CHECK(validateColoring(rows, movable, guarded), "guarded schedule is conflict-free within each colour");
    CHECK(guarded.colors == 1 && guarded.largestClass == rows.size(),
          "rows sharing only an immovable floor fit in one colour, at full width");
    // Same rows, floor treated as a conflict: the naive cost this removal fixes.
    const std::vector<char> allMovable(movable.size(), 1);
    const ColorPlan naive = colorRows(rows, allMovable);
    CHECK(validateColoring(rows, allMovable, naive), "naive schedule is also valid");
    CHECK(naive.colors == rows.size(),
          "treating the immovable floor as a conflict costs one colour per row");

    // Determinism, and independence from contact enumeration order.
    const ColorPlan again = colorRows(rows, movable);
    bool stable = again.colors == guarded.colors && again.color_of_row == guarded.color_of_row;
    CHECK(stable, "colour assignment is deterministic across calls");
    std::vector<ContactManifold> reversed = w.manifolds();
    std::reverse(reversed.begin(), reversed.end());
    const std::vector<Joint> none;
    RowSet flipped;
    flipped.build(reversed, none, w.bodies);
    const ColorPlan flippedPlan = colorRows(flipped.rows(), movable);
    CHECK(flippedPlan.colors == guarded.colors,
          "colour count does not depend on contact enumeration order");
    CHECK(validateColoring(flipped.rows(), movable, flippedPlan), "reversed-enumeration schedule is valid");
    std::printf("SCHEDULE rows=%zu guarded_colors=%u naive_colors=%u largest_class=%u\n",
                rows.size(), guarded.colors, naive.colors, guarded.largestClass);
}

// ---- 4. bounds and empty input --------------------------------------------
static void edges() {
    World empty;
    empty.prepare({4, 2, 8, 8, 8});
    CHECK(measuredStep(empty, 1.0 / 60) && empty.rows().size() == 0, "empty world yields no rows");
    const auto movable = empty.movableBodies();
    const ColorPlan plan = colorRows(empty.rows().rows(), movable);
    CHECK(plan.colors == 0 && validateColoring({}, movable, plan), "empty schedule needs no colour");

    // Two immovable bodies never form a row, so a static/static stack is inert.
    World statics;
    statics.add(RigidBody::makeBox(4, 1, 0, {0, 0}));
    statics.bodies.back().position.y -= 0.1;
    statics.add(RigidBody::makeBox(4, 1, 0, {0, 0}));
    statics.prepare({4, 0, 8, 8, 8});
    for (int f = 0; f < 20; ++f) CHECK(measuredStep(statics, 1.0 / 60), "static/static step allocates nothing");
    CHECK(statics.rows().size() == 0, "a pair of immovable bodies contributes no row");
}

int main() {
    immovableUntouched(SolverMode::PGS, false, false, "PGS: static bodies bitwise untouched over 200 ticks");
    immovableUntouched(SolverMode::PGS, true, true, "PGS+NGS+sleeping: static bodies bitwise untouched");
    immovableUntouched(SolverMode::TGS, false, false, "TGS: static bodies bitwise untouched over 200 ticks");
    immovableUntouched(SolverMode::TGS, true, true, "TGS+NGS+sleeping: static bodies bitwise untouched");
    rowIdentity();
    coloring();
    edges();
    std::printf("%s row/schedule harness (%d failed); SolverRow %zu/%zu ColorPlan %zu/%zu\n",
                failures ? "FAIL" : "PASS", failures, sizeof(SolverRow), alignof(SolverRow),
                sizeof(ColorPlan), alignof(ColorPlan));
    return failures ? 1 : 0;
}
